from robstride_dynamics import Motor, RobstrideBus


class Arm:
    def __init__(self, bus_configs: dict[str, str], motor_configs: dict[str, Motor], calibration: dict[str, dict] | None = None):
        self._buses: dict[str, RobstrideBus] = {}
        self._motors: dict[str, tuple[RobstrideBus, Motor]] = {}

        self._joint_names = list[str](motor_configs.keys())

        registered_motors = {}
        for side, channel in bus_configs.items():
            motors_on_this_bus = {name: motor for name, motor in motor_configs.items() if name.startswith(side)}
            bus = RobstrideBus(
                channel=channel,
                motors=motors_on_this_bus,
                calibration=calibration,
            )
            bus.connect()
            self._buses[side] = bus

            for motor_name, motor in motors_on_this_bus.items():
                registered_motors[motor_name] = (bus, motor)

        # reorder motors by joint names
        for name in self._joint_names:
            entry = registered_motors.get(name)
            if entry is None:
                raise ValueError(f"Motor {name} not found")
            self._motors[name] = entry

    @property
    def buses(self) -> dict[str, RobstrideBus]:
        return self._buses

    @property
    def motors(self) -> dict[str, tuple[RobstrideBus, Motor]]:
        return self._motors

    @property
    def joint_names(self) -> list[str]:
        return self._joint_names

    @property
    def num_joints(self) -> int:
        return len(self._joint_names)
