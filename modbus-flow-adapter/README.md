# Modbus 流量通信适配器

这是一个独立运行的 Go 程序，用于把 `VirtualDriverApp` 输出的四台泵流量，低延迟转发到另一台 Modbus TCP 设备。

## 通信方向

```text
VirtualDriverApp（TCP 主站，FC16）
              │  写保持寄存器 0..7
              ▼
本适配器（输入 TCP 从站）
              │  事件触发、仅保留最新值、复用长连接
              ▼
本适配器（输出 TCP 主站，FC16）
              │
              ▼
目标接口设备（TCP 从站）
```

输入沿用泵模拟器现有寄存器定义和 `0.01 m³/h` 比例，不改动模拟器业务逻辑：

| 输入协议地址（0 基） | 参数 | 输出字段 |
|---:|---|---|
| 1 | P1 流量 | S1 `positive_flow_m3h` |
| 3 | N1 流量 | S1 `negative_flow_m3h` |
| 5 | P2 流量 | S2 `positive_flow_m3h` |
| 7 | N2 流量 | S2 `negative_flow_m3h` |

默认输出映射如下。每项占两个 16 位保持寄存器，流量默认编码为 IEEE-754 `float32`、大端字节、高字在前。

| 子系统 | 字段 | 文档地址 | PDU 0 基地址 | 本程序是否写入 |
|---|---|---:|---:|---|
| S1 | `mode` | 41001-41002 | 1000-1001 | 否（Main Power 数据） |
| S1 | `current_A` | 41003-41004 | 1002-1003 | 否（Main Power 数据） |
| S1 | `positive_flow_m3h` | 41005-41006 | 1004-1005 | 是 |
| S1 | `negative_flow_m3h` | 41007-41008 | 1006-1007 | 是 |
| S2 | `mode` | 42001-42002 | 2000-2001 | 否（Main Power 数据） |
| S2 | `current_A` | 42003-42004 | 2002-2003 | 否（Main Power 数据） |
| S2 | `positive_flow_m3h` | 42005-42006 | 2004-2005 | 是 |
| S2 | `negative_flow_m3h` | 42007-42008 | 2006-2007 | 是 |

## 快速使用

需要 Go 1.22 或更高版本。

```powershell
cd D:\ProjectRepo\C#_OTHERS\VirtualDriverApp\modbus-flow-adapter
.\build.ps1
cd .\publish\windows-amd64
notepad .\config.yaml
.\modbus-flow-adapter.exe
```

程序默认读取可执行文件同目录的 `config.yaml`。也可以显式指定：

```powershell
.\modbus-flow-adapter.exe -config D:\config\flow-adapter.yaml
```

启动前只检查配置：

```powershell
.\modbus-flow-adapter.exe -config .\config.yaml -check
```

开发时可直接运行：

```powershell
go run .\cmd\flow-adapter -config .\config.yaml
```

## 配置说明

- `input.listen`：适配器输入从站监听的 IP 和端口。泵模拟器的“液压输出 TCP”目标要指向这里。
- `input.unit_id`：接收请求的 Unit ID，需与泵模拟器一致。
- `input.flow_scale`：输入 `uint16` 比例，默认 100，即寄存器 `4567` 表示 `45.67 m³/h`。
- `output.endpoint`：目标接口设备的 IP 和端口。
- `output.unit_id`：目标设备 Unit ID。
- `output.register_notation`：默认使用文档常见的 `4xxxx_one_based`；若目标设备手册给的是协议 0 基地址，改成 `raw_zero_based` 并同步修改映射。
- `output.encoding`：可选 `float32`、`uint32_scaled`、`int32_scaled`，字节序和字序均可配置。

正负极流量地址必须连续，这样每个子系统只需要一次 FC16 请求。S1 和 S2 地址段不连续，因此一帧数据固定发送两次 FC16。

## 低延迟和故障恢复

- 输入写请求直接触发转发，不采用定时轮询。
- 输出端建立后持续复用同一 TCP 连接。
- 默认队列容量为 1；输出短暂变慢或断线时自动丢弃旧帧，只保留最新流量。
- 连接失败采用 `100ms` 起步、最大 `2s` 的指数退避；重连成功后立即补发最新值。
- 每次成功转发可打印端到端应用层耗时；生产环境若更新频繁，可把 `log_each_update` 设为 `false`。

## 第三方库

Modbus TCP 客户端和服务端协议处理均使用开源库 [simonvetter/modbus](https://github.com/simonvetter/modbus)，本程序只实现寄存器映射、配置、编码和转发逻辑。
