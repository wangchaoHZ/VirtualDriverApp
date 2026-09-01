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

RTU 和 TCP 均只开放原有功能码 FC03（读保持寄存器）与
FC06（写单个保持寄存器）。TCP 适配器通过 FluentModbus 的
`RequestValidator` 保持相同的地址和功能码校验规则，并通过
`RegistersChanged` 将客户端写入提交给原业务模型。

## 使用

1. 在主界面的“通信模式”中选择 `Modbus RTU` 或 `Modbus TCP`。
2. RTU 模式选择串口；串口参数保持 `9600 / 8 / None / 1`。
3. TCP 模式设置监听 IP 和端口。`0.0.0.0` 表示监听所有本机 IPv4 接口，默认端口为 `502`。
4. 点击“启动模拟器”。成功启动后配置会保存到当前用户设置，下次启动自动恢复。

运行时状态栏会显示当前模拟服务端的传输模式和实际端点。启动后可继续
修改模式或端点，并点击“应用通信配置”热切换；切换失败时程序会自动
回退到上一个可用配置，模拟业务与外部 DI/DO/AI 通信保持运行。

## 构建

项目目标框架为 .NET Framework 4.7.2，使用 Visual Studio/MSBuild
构建：

```powershell
msbuild VirtualDriverApp.sln /t:Build /p:Configuration=Release
```

FluentModbus NuGet 版本固定为 `5.3.2`，项目已有对应的
`packages.config` 和程序集引用。
