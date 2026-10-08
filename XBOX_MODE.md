# Xbox Series mode (Windows PC, experimental)

基于 CLF184/DS5Dongle `33836f0`，为 Raspberry Pi Pico 2 W 新增控制器模式 `3: Xbox`。
本版本已通过编译和软件协议验证。修正 GIP 设备 ID 和 Windows 免认证接口声明后，
已收到用户关于 Windows 识别和扳机震动有效的实机反馈；不同游戏的兼容性和完整按键/四通道输出仍需继续验收。

## 使用

在 OLED Settings 的 `Ctrl` 项选择 `Xbox`，按三角键保存。保存成功后 USB 自动断开并重新枚举。
切回 `DS5`、`DSE` 或 `Auto` 使用同一菜单。已有配对、配置格式及默认 Auto 模式保留。
不增加切换模式的手柄快捷键。

Xbox 模式使用 USB GIP，身份为 `045E:0B12`（Xbox Series 控制器），含 `XGIP10` 驱动绑定描述符。
GIP Device ID 使用规范要求的 `0x0000FFFB` 前缀和每次启动生成的 32 位随机数，
USB serial 和 Hello 保持一致。元数据加入 Windows USB 免认证 GUID
`7a34ce77-7de2-45c6-8ca4-0042c08bd94a`，仅用于 Windows PC。
此模式仅暴露游戏控制器接口；需要原生 DualSense 音频、陀螺仪、触摸数据或配置 HID 时，使用 OLED 切回 Sony 模式。
这里的 Xbox Series 身份指手柄，并不表示实现了 Xbox Series S 主机或主机附件认证。

## 按键映射

| DualSense / Edge | Xbox |
| --- | --- |
| Cross / Circle / Square / Triangle | A / B / X / Y |
| D-pad（含斜方向） | D-pad |
| 左右摇杆 | 左右摇杆，Y 轴转换成 Xbox 坐标 |
| L3 / R3 | 左右摇杆按下 |
| L1 / R1 | LB / RB |
| L2 / R2 | LT / RT，0–255 转换为 0–1023 |
| Create | View |
| Options | Menu |
| PS | Guide，独立 GIP 虚拟键消息 |
| 静音键 | Share，Console Function Map 扩展 |
| 触摸板按下、滑动、陀螺仪 | 不映射 |

Edge 专用功能键和背键没有独立 Xbox 按键编号；本模式不增加背键映射。
Share 不属于经典 XInput / Windows.Gaming.Input.Gamepad 的标准按键位。

## 震动

接收 GIP `0x09` Direct Motor Command，保留左右普通震动和左右扳机震动四个独立通道。
普通震动使用 DualSense 兼容震动路径；Xbox 扳机震动转换为 DualSense `0x26` 自适应扳机振动，
覆盖十个扳机区间，使用 40 Hz 频率和八级幅度，零强度发送 `0x05` 解除效果。
两种手柄执行器不同，实际手感不是 Xbox 马达的物理复刻。
当前自适应扳机振动覆盖全部十个区域；扣下扳机进入作用区域后才有明显反馈，
完全松开时的效果与 Xbox 独立扳机震动马达不同。当前强度保持八档线性转换，未增加增益。

实现马达位图、10 ms 时长、脉冲间隔、重复次数、零时长全停、STOP/OFF/QUIESCE/RESET 全停和 USB 挂起停止。
蓝牙队列忙时保留最新输出并重试，避免丢失停止命令。
游戏必须通过支持四通道的 API（例如 Windows.Gaming.Input 或 GameInput）发送扳机震动。
仅发送经典两通道 XInput 震动的游戏不会自动产生扳机效果。

## 验证状态

- Pico 2 W Release 固件编译通过：Pico SDK 2.3.0、ARM GCC 13.2，TinyUSB 使用当前已安装的 SDK 版本。
- 临时软件验证通过：全部基础按键、摇杆端点/中心、所有扳机输入值、静音键 Share、触摸板未映射。
- GIP Hello 设备 ID 固定前缀、元数据的 Windows 免认证/Share GUID、分包/ACK/完成消息、ACK 超时回退、合并报文、Guide 按下/释放验证通过。
- 四路马达、位图部分更新、脉冲重复、停止、计时回绕验证通过；AddressSanitizer/UBSan 下随机解析 100000 个报文通过。
- 固件中没有 `test` / `tests` 目录。
- 修正版 Windows `045E:0B12` / `XGIP10` 枚举及 Xbox 驱动启动已观察到；用户已确认测试工具和游戏能产生扳机反馈。

刷入后通过 `joy.cpl` 检查基础输入，并在支持扳机震动的游戏或四通道 API 工具中
分别检查左右普通震动和左右扳机震动，确认各通道会启动、停止，左右不串位。
Share 和 Guide 需用支持它们的系统/游戏功能验收。
通过 OLED 切回 DS5 / DSE 复查原生功能。独立测试工具不属于固件源码交付。
若 Windows 未识别，请保留设备管理器的设备状态与硬件 ID，以及 USBPcap 枚举/初始化抓包供下一轮排查。

## 协议来源

- [Microsoft MS-GIPUSB](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-gipusb/e7c90904-5e21-426e-b9ad-d82adeee0dbc)：USB 枚举、Hello、分包和状态机。
- [GIP Device ID](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-gipusb/16b9702e-553e-46e3-b44f-03255cd24f90)：设备 ID 的固定前缀和启动随机数要求。
- [GIP Metadata Exchange](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-gipusb/405cbf7a-642f-4ae5-94a4-51b4dee057de)：元数据布局与游戏控制器 GUID。
- [MS-GIPUSB PDF](https://winprotocoldocs-bhdugrdyduf5h2e4.b02.azurefd.net/MS-GIPUSB/%5BMS-GIPUSB%5D.pdf)：第 5.1 节规定 Windows USB 免认证接口 GUID。
- [Direct Motor Command](https://learn.microsoft.com/en-us/openspecs/windows_protocols/ms-gipusb/ee8c5b28-e8da-4cc4-bb48-17781b8371af)：四路马达、位图、时长和重复字段。
- [Gamepad and vibration](https://learn.microsoft.com/en-us/windows/uwp/gaming/gamepad-and-vibration)：Windows 四通道游戏手柄 API。
- [Nielk1 DualSense trigger research](https://gist.github.com/Nielk1/6d54cc2c00d2201ccb8c2720ad7538db)：DualSense 振动效果的分区、幅度和频率编码。

## 重新编译

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
export PICO_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain
export PATH="$PICO_TOOLCHAIN_PATH/bin:$PATH"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DVERSION=xbox-series-dev
cmake --build build -j4
```

子模块须完整：`git submodule update --init --recursive`。默认构建目标为 Pico 2 W。
最终产物为 `build/ds5-bridge.uf2`。
