本仓库提供 Prime-Lite 机器人的动作录制、回放与遥操作代码，以及配套的环境配置说明。

# 动作录制 & 回放

## 使用方式

**该仓库的代码要求左胳膊接 CAN0，右胳膊接 CAN1。若不满足该要求，请看下方 “个性化配置”**

无假爪 Prime Lite 机器人录制回放动作使用方法：

``` bash
bash install.sh
cd no_gripper
python3 gui.py
```

有假爪的 Prime Lite 机器人录制回放动作使用方法：

``` bash
bash install.sh
cd have_gripper
python3 gui.py
```

在等待扫描完所有电机后，可使用弹出的 GUI 进行动作录制与动作回放，支持多次录制与回放历史录制动作。

## 个性化配置

**以左胳膊接的是 CAN2，右胳膊届的是 CAN1 为例，应对代码进行以下修改后再进行使用。**

1. `gui.py` 的第 $29,30$ 行中：

```py
run_command("python3 discover_actuators.py -c=can0", cwd=BASE_DIR)
run_command("python3 discover_actuators.py -c=can1", cwd=BASE_DIR)
```

修改为：

```py
run_command("python3 discover_actuators.py -c=can2", cwd=BASE_DIR)
run_command("python3 discover_actuators.py -c=can1", cwd=BASE_DIR)
```

2. `arm` 文件夹下的 `record_motion.py` 的第 $17$ 行：

```py
bus_configs = generate_bus_config(0, 1)
```

修改为：

```py
bus_configs = generate_bus_config(2, 1)
```

3. `arm` 文件夹下的 `replay_motion.py` 的第 $11$ 行：

```py
bus_configs = generate_bus_config(0, 1)
```

修改为：

```py
bus_configs = generate_bus_config(2, 1)
```

# 遥操作

对于 Quest 手柄遥操作，需要同时运行以下两个终端：

1. 在第一个终端中，按照 [Oculus Reader](https://github.com/pengyichen2026/Prime-Lite/tree/main/oculus_reader) 中的 “使用方法” 启动数据转发，将 Quest 手柄的位姿及按键信息发送至 UDP `11005` 端口。**遥操作期间，请保持该终端运行。**
2. 在第二个终端中，按照 [Teleop](https://github.com/pengyichen2026/Prime-Lite/tree/main/teleop) 中的 “CAN Bring-Up” 和 “Calibration (Real Robot)” 说明完成 CAN 接口初始化及必要的机器人校准，再按照 “SteamVR IK Teleop” 部分的说明启动手柄遥操作程序。

如需使用 键盘遥操作，请参阅 Teleop 中的 “Keyboard IK Teleop” 部分，无需启动 Oculus Reader。

# 致谢

感谢 [深度机智（DeepCybo）](https://deepcybo.top/) 与 [中关村学院（ZGCA）](https://www.bza.edu.cn/en/) 提供的帮助。
