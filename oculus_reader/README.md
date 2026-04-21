`Oculus_Reader` 的主要作用是识别 `Quest` 的两个手柄，并将他们的信息发送到 $11005$ 端口。

信息包括手柄的坐标与旋转，以及所有的按键是否按下还有 `Index Trigger` 和 `Grip Trigger` 两个按键的按压程度（0~1）。

# 使用方法

先将电脑与 `Quest` 头显连接，然后运行：

```bash
bash guide.sh
```

之后终端可能会出现：

```bash
2G97C5ZH4T00SB	no permissions (missing udev rules? user is in the plugdev group); see [http://developer.android.com/tools/device.html]

Device is visible but could not be accessed.
Run `adb devices` to verify that the device is visible and accessible.
If you see "no permissions" next to the device serial, please put on the Oculus Quest and allow the access.
```

此时应该在 `Quest` 中点击 “通知”，之后点击 “检测到 USB” 使其关闭，最后再运行：

```bash
bash guide.sh
```

当终端出现形如以下样式的信息，则 `Oculus_Reader` 部分运行成功。

```bash
🚀 转发中... Quest -> BHL (UDP 11005)
重要提示:
1. 保持手柄静止后启动 BHL 控制脚本
2. 启动后按下手柄侧键/A键进行 IK 重置(Rebase)
3. 当前转发频率: 30Hz (已降低以适配 CAN 总线带宽)
发送中... 左手(BHL坐标系): X:-0.35, Y:0.67, Z:0.359
```

**切记后续使用 teleop 的时候不要关闭该 `Oculus_Reader` 的终端！**
