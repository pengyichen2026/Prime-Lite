## CAN Bring-Up

Bring up the default CAN interfaces:

```bash
source ./examples/canup.sh
```

The default mapping in this package is:

- `left arm -> can0`
- `right arm -> can1`

## Calibration (Real Robot)

Run:

```bash
uv run ./examples/connect_bhl_arm.py --calibrate --count 1
```

This will connect to the robot, run calibration if no saved LeRobot calibration exists yet, and write the calibration file under LeRobot's calibration directory:

```text
~/.cache/huggingface/lerobot/calibration/
```

## Keyboard IK Teleop

This repository also includes a second keyboard teleoperator that moves an end-effector target in Cartesian space and uses MuJoCo + Mink IK to solve joint positions for the real robot.

Default keys:

- Left arm:
- `w` / `s`: `+x / -x` forward/backward translation
- `a` / `d`: `+y / -y` left/right translation
- `r` / `f`: `+z / -z` up/down translation
- `q` / `e`: `+x` roll / `-x` roll
- `z` / `x`: `+z` yaw / `-z` yaw
- Right arm:
- `i` / `k`: `+x / -x` forward/backward translation
- `j` / `l`: `+y / -y` left/right translation
- `y` / `h`: `+z / -z` up/down translation
- `u` / `o`: `+x` roll / `-x` roll
- `n` / `m`: `+z` yaw / `-z` yaw
- `c`: reset both targets to the current measured end-effector poses
- `p`: request exit

Run the manual script with the MuJoCo viewer and frame markers:

```bash
uv run ./examples/teleop_bhl_arm_keyboard_ik.py \
  --stiffness 200 \
  --damping 25 \
  --soft-start-duration 2 \
  --ee-step 0.01 \
  --viewer
```

The viewer shows both the current end-effector frame and the commanded target frame for each arm. By default, the teleoperator looks for the BHL Arm MJCF at:

```text
./data/bhl_arm/mjcf/bhl_arm.xml
```

relative to this repository's parent workspace. If your MJCF lives elsewhere, pass `--xml-path /abs/path/to/bhl_arm.xml`.


## SteamVR IK Teleop

This repository also includes a SteamVR teleoperator that listens for controller pose deltas over UDP, rebases those deltas to the robot's current end-effector pose, and solves dual-arm IK with MuJoCo + Mink.

The expected controller payload per side is:

- `relative_location`: `[x, y, z]`
- `relative_orientation`: `[qw, qx, qy, qz]`

Run the manual real-robot teleop:

```bash
uv run ./examples/teleop_bhl_arm_steamvr_ik.py \
  --stiffness 200 \
  --damping 25 \
  --soft-start-duration 2 \
  --frequency 30 \
  --steamvr-host 0.0.0.0 \
  --steamvr-port 11005 \
  --viewer
```

This listens for SteamVR packets on UDP `0.0.0.0:11005` by default. The viewer shows current and target end-effector frames for both arms.

## Acknowledgement

We would like to thank [DeepCybo](https://deepcybo.top/) and [Zhongguancun Academy (ZGCA)](https://www.bza.edu.cn/en/) for the collaboration and support.
