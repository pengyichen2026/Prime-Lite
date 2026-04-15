import json
import argparse
import numpy as np
import os

from loop_rate_limiters import RateLimiter

from config import motor_configs, generate_bus_config
from arm import Arm
from api import prepare_arm, shutdown_arm

parser = argparse.ArgumentParser()
parser.add_argument("--name", "-n", type=str, default="", help="replay name")
args = parser.parse_args()
replay_name = args.name

bus_configs = generate_bus_config(0, 1)

np.set_printoptions(precision=4, linewidth=100)

calibration = json.load(open("ude_calibration.json"))
arm = Arm(bus_configs, motor_configs, calibration)

rate = RateLimiter(frequency=50.0)

kp = 0.0
kd = 0.0
# kd = 0.25

recorded_motion = []

prepare_arm(arm, torque_limits={"default": 1.0}, strict=True)

stop_file = "/tmp/stop_record.flag"
if os.path.exists(stop_file):
    os.remove(stop_file)

try:
    while not os.path.exists(stop_file):
        positions = np.zeros(arm.num_joints, dtype=np.float32)
        velocities = np.zeros(arm.num_joints, dtype=np.float32)

        for motor_name, entry in arm.motors.items():
            bus, motor = entry
            index = arm.joint_names.index(motor_name)
            bus.write_operation_frame(motor_name, 0.0, kp, kd)
            position, velocity, _, _ = bus.read_operation_frame(motor_name)
            positions[index] = position
            velocities[index] = velocity

        print(f"positions: {positions} \t velocities: {velocities}")

        recorded_motion.append([positions, velocities])

        rate.sleep()

except KeyboardInterrupt:
    pass

shutdown_arm(arm)

recorded_motion = np.array(recorded_motion)

# save the recorded positions to a file
np.save(f"recorded_motion_{replay_name}.npy", recorded_motion)

print("Program terminated.")
