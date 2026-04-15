import numpy as np

from robstride_dynamics import Motor

def generate_bus_config(left_can_bus: int, right_can_bus: int):
    return {"left": f"can{left_can_bus}", "right": f"can{right_can_bus}"}

is_arm_with_gripper = False

motor_without_gripper_configs = {
    "left_shoulder_pitch": Motor(id=11, model="rs-02"),
    "left_shoulder_roll": Motor(id=12, model="rs-00"),
    "left_shoulder_yaw": Motor(id=13, model="rs-00"),
    "left_elbow_pitch": Motor(id=14, model="rs-00"),
    "left_wrist_yaw": Motor(id=15, model="rs-05"),
    "left_wrist_roll": Motor(id=16, model="rs-05"),
    "left_wrist_pitch": Motor(id=17, model="rs-05"),
    "right_shoulder_pitch": Motor(id=21, model="rs-02"),
    "right_shoulder_roll": Motor(id=22, model="rs-00"),
    "right_shoulder_yaw": Motor(id=23, model="rs-00"),
    "right_elbow_pitch": Motor(id=24, model="rs-00"),
    "right_wrist_yaw": Motor(id=25, model="rs-05"),
    "right_wrist_roll": Motor(id=26, model="rs-05"),
    "right_wrist_pitch": Motor(id=27, model="rs-05"),
}

motor_with_gripper_configs = {
    "left_shoulder_pitch": Motor(id=11, model="rs-02"),
    "left_shoulder_roll": Motor(id=12, model="rs-00"),
    "left_shoulder_yaw": Motor(id=13, model="rs-00"),
    "left_elbow_pitch": Motor(id=14, model="rs-00"),
    "left_wrist_yaw": Motor(id=15, model="rs-05"),
    "left_wrist_roll": Motor(id=16, model="rs-05"),
    "left_wrist_pitch": Motor(id=17, model="rs-05"),
    "left_gripper": Motor(id=18, model="rs-05"),
    "right_shoulder_pitch": Motor(id=21, model="rs-02"),
    "right_shoulder_roll": Motor(id=22, model="rs-00"),
    "right_shoulder_yaw": Motor(id=23, model="rs-00"),
    "right_elbow_pitch": Motor(id=24, model="rs-00"),
    "right_wrist_yaw": Motor(id=25, model="rs-05"),
    "right_wrist_roll": Motor(id=26, model="rs-05"),
    "right_wrist_pitch": Motor(id=27, model="rs-05"),
    "right_gripper": Motor(id=28, model="rs-05"),
}

motor_without_gripper_master_configs = {
    "left_shoulder_pitch": Motor(id=11, model="rs-02"),
    "left_shoulder_roll": Motor(id=12, model="rs-00"),
    "left_shoulder_yaw": Motor(id=13, model="rs-00"),
    "left_elbow_pitch": Motor(id=14, model="rs-00"),
    "left_wrist_yaw": Motor(id=15, model="rs-05"),
    "left_wrist_roll": Motor(id=16, model="rs-05"),
    "left_wrist_pitch": Motor(id=17, model="rs-05"),
    "right_shoulder_pitch": Motor(id=21, model="rs-02"),
    "right_shoulder_roll": Motor(id=22, model="rs-00"),
    "right_shoulder_yaw": Motor(id=23, model="rs-00"),
    "right_elbow_pitch": Motor(id=24, model="rs-00"),
    "right_wrist_yaw": Motor(id=25, model="rs-05"),
    "right_wrist_roll": Motor(id=26, model="rs-05"),
    "right_wrist_pitch": Motor(id=27, model="rs-05"),
}

motor_without_gripper_slave_configs = {
    "left_shoulder_pitch": Motor(id=31, model="rs-02"),
    "left_shoulder_roll": Motor(id=32, model="rs-00"),
    "left_shoulder_yaw": Motor(id=33, model="rs-00"),
    "left_elbow_pitch": Motor(id=34, model="rs-00"),
    "left_wrist_yaw": Motor(id=35, model="rs-05"),
    "left_wrist_roll": Motor(id=36, model="rs-05"),
    "left_wrist_pitch": Motor(id=37, model="rs-05"),
    "right_shoulder_pitch": Motor(id=41, model="rs-02"),
    "right_shoulder_roll": Motor(id=42, model="rs-00"),
    "right_shoulder_yaw": Motor(id=43, model="rs-00"),
    "right_elbow_pitch": Motor(id=44, model="rs-00"),
    "right_wrist_yaw": Motor(id=45, model="rs-05"),
    "right_wrist_roll": Motor(id=46, model="rs-05"),
    "right_wrist_pitch": Motor(id=47, model="rs-05"),
}

motor_without_gripper_limits = {
    "left_shoulder_pitch":  (np.deg2rad(-180),  np.deg2rad(45)),
    "left_shoulder_roll":   (np.deg2rad(-10),  np.deg2rad(190)),
    "left_shoulder_yaw":    (np.deg2rad(-90),   np.deg2rad(90)),
    "left_elbow_pitch":     (np.deg2rad(-135),  np.deg2rad(0)),
    "left_wrist_yaw":       (np.deg2rad(-90),   np.deg2rad(90)),
    "left_wrist_roll":      (np.deg2rad(-40),   np.deg2rad(40)),
    "left_wrist_pitch":     (np.deg2rad(-45),   np.deg2rad(45)),
    "right_shoulder_pitch": (np.deg2rad(-180),  np.deg2rad(45)),
    "right_shoulder_roll":  (np.deg2rad(-190),  np.deg2rad(10)),
    "right_shoulder_yaw":   (np.deg2rad(-90),   np.deg2rad(90)),
    "right_elbow_pitch":    (np.deg2rad(-135),  np.deg2rad(0)),
    "right_wrist_yaw":      (np.deg2rad(-90),   np.deg2rad(90)),
    "right_wrist_roll":     (np.deg2rad(-40),   np.deg2rad(40)),
    "right_wrist_pitch":    (np.deg2rad(-45),   np.deg2rad(45)),
}

motor_with_gripper_limits = {
    "left_shoulder_pitch":  (np.deg2rad(-180),  np.deg2rad(45)),
    "left_shoulder_roll":   (np.deg2rad(-10),  np.deg2rad(190)),
    "left_shoulder_yaw":    (np.deg2rad(-90),   np.deg2rad(90)),
    "left_elbow_pitch":     (np.deg2rad(-135),  np.deg2rad(0)),
    "left_wrist_yaw":       (np.deg2rad(-90),   np.deg2rad(90)),
    "left_wrist_roll":      (np.deg2rad(-40),   np.deg2rad(40)),
    "left_wrist_pitch":     (np.deg2rad(-45),   np.deg2rad(45)),
    "left_gripper":         (np.deg2rad(0),     np.deg2rad(114)),
    "right_shoulder_pitch": (np.deg2rad(-180),  np.deg2rad(45)),
    "right_shoulder_roll":  (np.deg2rad(-190),  np.deg2rad(10)),
    "right_shoulder_yaw":   (np.deg2rad(-90),   np.deg2rad(90)),
    "right_elbow_pitch":    (np.deg2rad(-135),  np.deg2rad(0)),
    "right_wrist_yaw":      (np.deg2rad(-90),   np.deg2rad(90)),
    "right_wrist_roll":     (np.deg2rad(-40),   np.deg2rad(40)),
    "right_wrist_pitch":    (np.deg2rad(-45),   np.deg2rad(45)),
    "right_gripper":        (np.deg2rad(0),     np.deg2rad(380)),
}

motor_configs = motor_with_gripper_configs if is_arm_with_gripper else motor_without_gripper_configs
motor_limits = motor_with_gripper_limits if is_arm_with_gripper else motor_without_gripper_limits
motor_master_configs = motor_without_gripper_master_configs
motor_slave_configs = motor_without_gripper_slave_configs
