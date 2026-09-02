#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inject a slowly changing sine-wave frequency into VirtualDriverApp.

Install the dependency once:
    python -m pip install "pymodbus>=3.10,<4"

PyModbus project:
    https://github.com/pymodbus-dev/pymodbus

Run until Ctrl+C:
    python pump_frequency_sine_test.py

The script writes FC06 to holding register 0x2001 and then uses FC03 to
read the value back. VirtualDriverApp encodes frequency as Hz * 100.
"""

from __future__ import annotations

import argparse
import inspect
import logging
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Iterable

try:
    import pymodbus
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
except ImportError:
    print(
        "缺少 pymodbus，请先执行：\n"
        '  python -m pip install "pymodbus>=3.10,<4"',
        file=sys.stderr,
    )
    raise SystemExit(2)


# ---------------------------------------------------------------------------
# User-editable settings
# ---------------------------------------------------------------------------
MODBUS_HOST = "127.0.0.1"
MODBUS_PORT = 2502
PUMP_UNIT_IDS = (11, 22, 33, 44)  # P1, N1, P2, N2

FREQUENCY_REGISTER = 0x2001
FREQUENCY_SCALE = 100.0  # 0.01 Hz per register count
MIN_FREQUENCY_HZ = 35.0
MAX_FREQUENCY_HZ = 48.0

# A new sine sample is written every four seconds. Keep this within 3-5 s
# for the requested slow switching behavior.
UPDATE_INTERVAL_SECONDS = 8.0

# One complete sine-wave cycle takes 128 seconds. With a 4-second interval,
# this produces 32 evenly spaced samples and reaches both 35 Hz and 48 Hz.
# Increase this value to make
# the frequency change more slowly without changing the 4-second write rate.
SINE_PERIOD_SECONDS = 128.0
START_PHASE_DEGREES = 0.0

SOCKET_TIMEOUT_SECONDS = 6.0
RECONNECT_DELAY_SECONDS = 10.0
VERIFY_AFTER_WRITE = True

# 0 means run until Ctrl+C. Set a positive number for a finite test.
MAX_STEPS = 0


@dataclass(frozen=True)
class TestSettings:
    host: str
    port: int
    unit_ids: tuple[int, ...]
    minimum_hz: float
    maximum_hz: float
    update_interval_seconds: float
    sine_period_seconds: float
    start_phase_degrees: float
    timeout_seconds: float
    reconnect_delay_seconds: float
    verify_after_write: bool
    max_steps: int


def parse_unit_ids(value: str | Iterable[int]) -> tuple[int, ...]:
    if isinstance(value, str):
        try:
            unit_ids = tuple(
                int(item.strip(), 0)
                for item in value.split(",")
                if item.strip()
            )
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                "从站地址必须是逗号分隔的整数，例如 11,22,33,44"
            ) from exc
    else:
        unit_ids = tuple(value)

    if not unit_ids:
        raise argparse.ArgumentTypeError("至少需要一个从站地址")
    if any(unit_id < 1 or unit_id > 247 for unit_id in unit_ids):
        raise argparse.ArgumentTypeError("从站地址必须在 1～247 范围内")
    if len(set(unit_ids)) != len(unit_ids):
        raise argparse.ArgumentTypeError("从站地址不能重复")
    return unit_ids


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "通过 Modbus TCP FC06 向四台泵注入 35～48 Hz 正弦频率，"
            "并通过 FC03 回读校验。"
        )
    )
    parser.add_argument("--host", default=MODBUS_HOST, help="模拟器 IP 地址")
    parser.add_argument("--port", type=int, default=MODBUS_PORT, help="模拟器端口")
    parser.add_argument(
        "--units",
        type=parse_unit_ids,
        default=parse_unit_ids(PUMP_UNIT_IDS),
        help="泵从站地址，逗号分隔，默认 11,22,33,44",
    )
    parser.add_argument(
        "--min-hz", type=float, default=MIN_FREQUENCY_HZ, help="最低频率"
    )
    parser.add_argument(
        "--max-hz", type=float, default=MAX_FREQUENCY_HZ, help="最高频率"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=UPDATE_INTERVAL_SECONDS,
        help="写入间隔秒数，建议保持在 3～5 秒",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=SINE_PERIOD_SECONDS,
        help="完整正弦周期秒数",
    )
    parser.add_argument(
        "--phase",
        type=float,
        default=START_PHASE_DEGREES,
        help="起始相位角，单位为度",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=SOCKET_TIMEOUT_SECONDS,
        help="Modbus 请求超时秒数",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=RECONNECT_DELAY_SECONDS,
        help="断线重连等待秒数",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=MAX_STEPS,
        help="执行步数，0 表示持续运行",
    )
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="写入后不执行 FC03 回读校验",
    )
    return parser


def validate_settings(settings: TestSettings) -> None:
    if not settings.host.strip():
        raise ValueError("模拟器 IP 地址不能为空")
    if settings.port < 1 or settings.port > 65535:
        raise ValueError("端口必须在 1～65535 范围内")
    if settings.minimum_hz < 0:
        raise ValueError("最低频率不能小于 0 Hz")
    if settings.maximum_hz <= settings.minimum_hz:
        raise ValueError("最高频率必须大于最低频率")
    if settings.maximum_hz * FREQUENCY_SCALE > 0xFFFF:
        raise ValueError("最高频率转换后超过 16 位寄存器范围")
    if settings.update_interval_seconds <= 0:
        raise ValueError("写入间隔必须大于 0 秒")
    if settings.sine_period_seconds <= 0:
        raise ValueError("正弦周期必须大于 0 秒")
    if settings.timeout_seconds <= 0:
        raise ValueError("请求超时必须大于 0 秒")
    if settings.reconnect_delay_seconds < 0:
        raise ValueError("重连等待不能小于 0 秒")
    if settings.max_steps < 0:
        raise ValueError("执行步数不能小于 0")


def build_settings(arguments: argparse.Namespace) -> TestSettings:
    return TestSettings(
        host=arguments.host.strip(),
        port=arguments.port,
        unit_ids=arguments.units,
        minimum_hz=arguments.min_hz,
        maximum_hz=arguments.max_hz,
        update_interval_seconds=arguments.interval,
        sine_period_seconds=arguments.period,
        start_phase_degrees=arguments.phase,
        timeout_seconds=arguments.timeout,
        reconnect_delay_seconds=arguments.reconnect_delay,
        verify_after_write=not arguments.no_verify,
        max_steps=arguments.steps,
    )


def unit_id_keyword(method: Any, unit_id: int) -> dict[str, int]:
    """Support current pymodbus device_id and older 3.x slave APIs."""
    parameters = inspect.signature(method).parameters
    if "device_id" in parameters:
        return {"device_id": unit_id}
    if "slave" in parameters:
        return {"slave": unit_id}
    if "unit" in parameters:
        return {"unit": unit_id}
    raise RuntimeError("当前 pymodbus 版本无法识别从站地址参数")


def ensure_success(response: Any, operation: str) -> None:
    if response is None:
        raise RuntimeError(f"{operation} 未收到响应")
    if response.isError():
        raise RuntimeError(f"{operation} 返回 Modbus 异常：{response}")


def calculate_frequency(settings: TestSettings, step: int) -> tuple[float, float]:
    elapsed_seconds = step * settings.update_interval_seconds
    phase_radians = math.radians(settings.start_phase_degrees) + (
        2.0 * math.pi * elapsed_seconds / settings.sine_period_seconds
    )
    midpoint_hz = (settings.minimum_hz + settings.maximum_hz) / 2.0
    amplitude_hz = (settings.maximum_hz - settings.minimum_hz) / 2.0
    frequency_hz = midpoint_hz + amplitude_hz * math.sin(phase_radians)
    return frequency_hz, math.degrees(phase_radians) % 360.0


def write_and_verify(
    client: ModbusTcpClient,
    unit_id: int,
    frequency_hz: float,
    verify_after_write: bool,
) -> float | None:
    register_value = int(round(frequency_hz * FREQUENCY_SCALE))
    write_response = client.write_register(
        address=FREQUENCY_REGISTER,
        value=register_value,
        **unit_id_keyword(client.write_register, unit_id),
    )
    ensure_success(write_response, f"Unit {unit_id} FC06 写入")

    if not verify_after_write:
        return None

    read_response = client.read_holding_registers(
        address=FREQUENCY_REGISTER,
        count=1,
        **unit_id_keyword(client.read_holding_registers, unit_id),
    )
    ensure_success(read_response, f"Unit {unit_id} FC03 回读")
    if not getattr(read_response, "registers", None):
        raise RuntimeError(f"Unit {unit_id} FC03 回读不包含寄存器数据")

    confirmed_register = int(read_response.registers[0])
    if confirmed_register != register_value:
        raise RuntimeError(
            f"Unit {unit_id} 回读不一致：写入 {register_value}，"
            f"回读 {confirmed_register}"
        )
    return confirmed_register / FREQUENCY_SCALE


def create_client(settings: TestSettings) -> ModbusTcpClient:
    return ModbusTcpClient(
        settings.host,
        port=settings.port,
        timeout=settings.timeout_seconds,
    )


def run_test(settings: TestSettings) -> None:
    logger = logging.getLogger("pump-frequency-test")
    midpoint_hz = (settings.minimum_hz + settings.maximum_hz) / 2.0
    amplitude_hz = (settings.maximum_hz - settings.minimum_hz) / 2.0

    logger.info(
        "PyModbus 版本：%s",
        getattr(pymodbus, "__version__", "未知"),
    )
    logger.info("泵模拟器地址：%s:%d", settings.host, settings.port)
    logger.info("泵 Unit ID：%s", ", ".join(map(str, settings.unit_ids)))
    logger.info(
        "写入方式：FC06 保持寄存器 0x%04X，编码为 Hz × %.0f；"
        "FC03 回读校验=%s",
        FREQUENCY_REGISTER,
        FREQUENCY_SCALE,
        "开启" if settings.verify_after_write else "关闭",
    )
    logger.info(
        "频率公式：f(t) = %.3f + %.3f × sin(2πt / %.1f + %.1f°) Hz",
        midpoint_hz,
        amplitude_hz,
        settings.sine_period_seconds,
        settings.start_phase_degrees,
    )
    logger.info(
        "频率范围 %.2f～%.2f Hz，每 %.1f 秒更新；按 Ctrl+C 停止",
        settings.minimum_hz,
        settings.maximum_hz,
        settings.update_interval_seconds,
    )
    if not 3.0 <= settings.update_interval_seconds <= 5.0:
        logger.warning(
            "当前更新间隔 %.2f 秒不在建议的 3～5 秒范围内",
            settings.update_interval_seconds,
        )

    client: ModbusTcpClient | None = None
    step = 0
    next_step_at = time.monotonic()

    try:
        while settings.max_steps == 0 or step < settings.max_steps:
            if client is None:
                logger.info("正在连接 Modbus TCP 服务端……")
                client = create_client(settings)
                if not client.connect():
                    logger.error(
                        "连接 %s:%d 失败，%.1f 秒后重试",
                        settings.host,
                        settings.port,
                        settings.reconnect_delay_seconds,
                    )
                    client.close()
                    client = None
                    time.sleep(settings.reconnect_delay_seconds)
                    next_step_at = time.monotonic()
                    continue
                logger.info("Modbus TCP 已连接")

            delay_seconds = next_step_at - time.monotonic()
            if delay_seconds > 0:
                time.sleep(delay_seconds)

            frequency_hz, phase_degrees = calculate_frequency(settings, step)
            register_value = int(round(frequency_hz * FREQUENCY_SCALE))
            logger.info(
                "步骤 %06d | 相位 %7.2f° | 目标 %6.2f Hz | 寄存器值 %d",
                step + 1,
                phase_degrees,
                frequency_hz,
                register_value,
            )

            try:
                for unit_id in settings.unit_ids:
                    confirmed_hz = write_and_verify(
                        client,
                        unit_id,
                        frequency_hz,
                        settings.verify_after_write,
                    )
                    if confirmed_hz is None:
                        logger.info(
                            "Unit %3d | FC06 写入成功 | 目标 %6.2f Hz",
                            unit_id,
                            frequency_hz,
                        )
                    else:
                        logger.info(
                            "Unit %3d | FC06 写入成功 | FC03 回读 %6.2f Hz | OK",
                            unit_id,
                            confirmed_hz,
                        )
            except (ModbusException, OSError, RuntimeError) as exc:
                logger.error("本步骤通信失败：%s", exc)
                client.close()
                client = None
                logger.info(
                    "%.1f 秒后重连并重试当前步骤",
                    settings.reconnect_delay_seconds,
                )
                time.sleep(settings.reconnect_delay_seconds)
                next_step_at = time.monotonic()
                continue

            step += 1
            next_step_at += settings.update_interval_seconds
            logger.info(
                "步骤完成，下一次更新约 %.1f 秒后执行",
                max(0.0, next_step_at - time.monotonic()),
            )
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，测试已停止")
    finally:
        if client is not None:
            client.close()
        logger.info("Modbus TCP 连接已关闭，共完成 %d 个步骤", step)


def main() -> int:
    # Keep Chinese logs readable in Windows Terminal and redirected output.
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding="utf-8", errors="replace")

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout,
    )

    parser = build_argument_parser()
    try:
        settings = build_settings(parser.parse_args())
        validate_settings(settings)
        run_test(settings)
        return 0
    except (ValueError, argparse.ArgumentTypeError) as exc:
        logging.getLogger("pump-frequency-test").error("参数错误：%s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
