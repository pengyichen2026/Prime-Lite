import json
import numpy as np

from loop_rate_limiters import RateLimiter
from config import motor_master_configs, motor_limits, is_arm_with_gripper, generate_bus_config
from arm import Arm

bus_configs = generate_bus_config(0, 1)
np.set_printoptions(precision=4, linewidth=100)
arm = Arm(bus_configs, motor_master_configs)
rate = RateLimiter(frequency=50.0)

measured_positions = np.zeros(arm.num_joints, dtype=np.float32)
measured_positions_min = np.zeros_like(measured_positions)
measured_positions_max = np.zeros_like(measured_positions)

measured_positions_min[:] = np.inf
measured_positions_max[:] = -np.inf

motor_directions = np.array([
    -1, -1, +1, -1, +1, -1, -1, -1,
    +1, -1, +1, +1, +1, +1, -1, -1,
]) if is_arm_with_gripper else np.array([
    -1, -1, +1, -1, +1, -1, -1,
    +1, -1, +1, +1, +1, +1, -1,
])

def flush_bus(bus):
    """Empty the CAN receive buffer."""
    try:
        while bus.channel_handler.recv(timeout=0):
            pass
    except:
        pass

try:
    while True:
        for motor_name, entry in arm.motors.items():
            bus, motor = entry
            index = arm.joint_names.index(motor_name)

            flush_bus(bus)

            try:
                bus.write_operation_frame(motor_name, 0.0, 0.0, 0.0)
                position, _, _, _ = bus.read_operation_frame(motor_name)
                measured_positions[index] = position
            except (AssertionError, RuntimeError):
                continue

        current_adjusted_positions = measured_positions * motor_directions

        measured_positions_min[:] = np.minimum(current_adjusted_positions, measured_positions_min)
        measured_positions_max[:] = np.maximum(current_adjusted_positions, measured_positions_max)

        print(current_adjusted_positions)

        rate.sleep()

except KeyboardInterrupt:
    pass

for _, bus in arm.buses.items():
    bus.disconnect()

lower_limits = np.array([motor_limits[motor_name][0] for motor_name in arm.joint_names])
upper_limits = np.array([motor_limits[motor_name][1] for motor_name in arm.joint_names])

print(f"measured_positions_min: {measured_positions_min}")
print(f"measured_positions_max: {measured_positions_max}")

lower_offset = measured_positions_min - lower_limits
upper_offset = measured_positions_max - upper_limits

print(f"lower_offset: {lower_offset}")
print(f"upper_offset: {upper_offset}")

calculated_calibration = np.stack([lower_offset, upper_offset], axis=1).mean(axis=1)

print(f"calculated_calibration: {calculated_calibration}")

calculated_calibration *= motor_directions

calibration = {}

for motor_name, entry in arm.motors.items():
    bus, motor = entry
    index = arm.joint_names.index(motor_name)
    calibration[motor_name] = {
        "id": motor.id,
        "direction": int(motor_directions[index]),
        "homing_offset": float(calculated_calibration[index]),
    }

with open("ude_calibration.json", "w") as f:
    json.dump(calibration, f, indent=4)

print("Program terminated.")
