# 录制动作 & 回放动作

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
