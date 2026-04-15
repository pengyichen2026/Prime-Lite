import json
import argparse
import numpy as np

from loop_rate_limiters import RateLimiter

from config import motor_configs, generate_bus_config
from arm import Arm
from api import prepare_arm, shutdown_arm

bus_configs = generate_bus_config(0, 1)

parser = argparse.ArgumentParser()
parser.add_argument("--name", "-n", type=str, default="", help="replay name")
args = parser.parse_args()
replay_name = args.name

np.set_printoptions(precision=4, linewidth=100)

# Can use this file to configure the calibration
# calibration = None
calibration = json.load(open("ude_calibration.json"))
arm = Arm(bus_configs, motor_configs, calibration)

recorded_motion = np.load(f"recorded_motion_{replay_name}.npy")

print(recorded_motion.shape)

rate = RateLimiter(frequency=50.0)

# tested up to kp 60, kd 2, 6 Nm
kp = 80.0
kd = 4.0
# kp = 40.0
# kd = 2.0


prepare_arm(arm, strict=True)

frame_index = 0

measured_torques = np.zeros(arm.num_joints, dtype=np.float32)

try:
    while True:
        positions, velocities = recorded_motion[frame_index]

        for motor_name, entry in arm.motors.items():
            bus, motor = entry
            index = arm.joint_names.index(motor_name)
            bus.write_operation_frame(motor_name, positions[index], kp, kd, velocity=velocities[index])
            _, _, torque, _ = bus.read_operation_frame(motor_name)
            measured_torques[index] = torque

        frame_index += 1

        if frame_index >= recorded_motion.shape[0]:
            break

        print(f"max torque: {np.abs(measured_torques).max():.2f} Nm")

        rate.sleep()

except KeyboardInterrupt:
    pass

shutdown_arm(arm)

recorded_motion = np.array(recorded_motion)

# save the recorded positions to a file

print("Program terminated.")
