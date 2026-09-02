# VirtualDriverApp

变频器与传感器模拟器，支持在 Modbus RTU 与 Modbus TCP
服务端模式之间切换。四个模拟变频器继续使用原有 Unit ID
`11`、`22`、`33`、`44`，寄存器地址、数值缩放、故障注入、
水力模拟和累计功耗逻辑均由同一数据层提供。

## 通信架构

- `IModbusServer`：统一定义服务端的启动、停止、运行状态和端点描述。
- `SwitchableModbusServer`：根据配置创建 RTU 或 TCP 适配器，并负责平滑释放当前传输实例。
- `ModbusRtuServerAdapter`：复用原有串口接收与 CRC 处理实现。
- `FluentModbusTcpServerAdapter`：使用
  [FluentModbus](https://github.com/Apollo3zehn/FluentModbus)
  的 `ModbusTcpServer` 处理 TCP 协议、连接与 MBAP 帧。
- `IModbusDataStore` / `VirtualModbusDataStore`：隔离传输层和模拟设备寄存器数据层。

RTU 保持开放原有功能码 FC03（读保持寄存器）与 FC06（写单个
保持寄存器）。TCP 在 FC03、FC06 基础上增加 FC04（读输入寄存器）
用于读取四台泵累计功耗。TCP 适配器通过 FluentModbus 的
`RequestValidator` 执行地址和功能码校验，并通过 `RegistersChanged`
将客户端写入提交给原业务模型。

### TCP FC04 累计功耗输入寄存器

Modbus TCP 从站在每个已配置 Unit ID（`11`、`22`、`33`、`44`）上
提供相同的四泵累计功耗只读表。地址 `0～7` 共 8 个 16 位输入
寄存器，地址 `8` 是结束位置、不属于有效数据区：

| 地址 | UInt32 数据 | 字序 |
| --- | --- | --- |
| 0～1 | P1 正极泵累计功耗 | 高字在 0，低字在 1 |
| 2～3 | N1 负极泵累计功耗 | 高字在 2，低字在 3 |
| 4～5 | P2 正极泵累计功耗 | 高字在 4，低字在 5 |
| 6～7 | N2 负极泵累计功耗 | 高字在 6，低字在 7 |

每个 `UInt32` 占两个 Modbus 寄存器（4 字节），采用高字优先和
Modbus 大端字节序。寄存器值单位为 `Wh`，即原累计功耗
`kWh × 1000`，分辨率保持 `0.001 kWh`。超出 `UInt32` 范围时限制
在 `0～4294967295`；FC04 只在 TCP 从站模式开放，RTU 行为不变。

## 使用

1. 在主界面的“通信模式”中选择 `Modbus RTU` 或 `Modbus TCP`。
2. RTU 模式选择串口；串口参数保持 `9600 / 8 / None / 1`。
3. TCP 模式设置监听 IP 和端口。`0.0.0.0` 表示监听所有本机 IPv4 接口，默认端口为 `502`。
4. 点击“启动模拟器”。成功启动后配置会保存到当前用户设置，下次启动自动恢复。

运行时状态栏会显示当前模拟服务端的传输模式和实际端点。启动后可继续
修改模式或端点，并点击“应用通信配置”热切换；切换失败时程序会自动
回退到上一个可用配置，模拟业务与外部 DI/DO 通信保持运行。

## 压力/流量 Modbus TCP 直发

F→P、F→Q 解算后的压力和流量不再换算为 4–20 mA 后发送给
AI 信号发生器。勾选“启用 FC16 压力/流量直发”后，应用作为
Modbus TCP 主站连接界面中设置的目标 IP、端口和 Unit ID，使用
FC16 从保持寄存器地址 `0` 开始一次写入 8 个寄存器：

| 地址 | 参数 | 编码 |
| --- | --- | --- |
| 0 | P1 正极泵压力 | MPa × 10000 |
| 1 | P1 正极泵流量 | m³/h × 100 |
| 2 | N1 负极泵压力 | MPa × 10000 |
| 3 | N1 负极泵流量 | m³/h × 100 |
| 4 | P2 正极泵压力 | MPa × 10000 |
| 5 | P2 正极泵流量 | m³/h × 100 |
| 6 | N2 负极泵压力 | MPa × 10000 |
| 7 | N2 负极泵流量 | m³/h × 100 |

每个量占一个无符号 16 位保持寄存器，采用四舍五入并限制在
`0..65535`。压力分辨率为 `0.0001 MPa`，流量分辨率为
`0.01 m³/h`。目标连接中断时复用现有的弹性 TCP 客户端自动重连；
取消勾选后释放连接并重新开放目标参数编辑。

## 构建

项目目标框架为 .NET Framework 4.7.2，使用 Visual Studio/MSBuild
构建：

```powershell
msbuild VirtualDriverApp.sln /t:Build /p:Configuration=Release
```

FluentModbus NuGet 版本固定为 `5.3.2`，项目已有对应的
`packages.config` 和程序集引用。
