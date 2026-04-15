from __future__ import annotations

import struct
import time
from typing import Callable, Optional

import numpy as np
from robstride_dynamics import ParameterType
from robstride_dynamics import RobstrideBus
from robstride_dynamics.protocol import CommunicationType
from robstride_dynamics.table import (
    MODEL_MIT_POSITION_TABLE,
    MODEL_MIT_VELOCITY_TABLE,
    MODEL_MIT_TORQUE_TABLE,
)

from arm import Arm

ErrorHandler = Callable[[str, Exception], None]


def _default_error_handler(message: str, exc: Exception) -> None:
    print(f"Warning: {message}: {exc}")


def _resolve_torque_limit(motor_model: str, torque_limits: Optional[dict[str, float]]) -> float:
    if torque_limits:
        if motor_model in torque_limits:
            return torque_limits[motor_model]
        if "default" in torque_limits:
            return torque_limits["default"]
    return 5.0 if motor_model == "rs-05" else 10.0


def _ids_for_side(motor_configs: dict, side: str) -> list[int]:
    return [motor.id for name, motor in motor_configs.items() if name.startswith(side)]


def _probe_channel_ids(channel: str, ids: list[int], *, timeout: float = 0.03) -> set[int]:
    bus = RobstrideBus(channel=channel, motors={})
    try:
        bus.connect(handshake=False)
    except Exception:
        return set()

    found = set()
    try:
        for device_id in ids:
            response = bus.ping_by_id(device_id, timeout=timeout)
            if response is not None:
                found.add(device_id)
    finally:
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
    return found


def _pick_best_channel(
    channels: list[str],
    target_ids: list[int],
    used_channels: set[str],
    *,
    min_hits: int = 2,
) -> tuple[str, int, dict[str, int]]:
    scores = {}
    for channel in channels:
        if channel in used_channels:
            continue
        scores[channel] = len(_probe_channel_ids(channel, target_ids))

    if not scores:
        raise RuntimeError("No available CAN channels to probe")

    best_channel, best_score = max(scores.items(), key=lambda item: item[1])
    if best_score < min_hits:
        raise RuntimeError(f"Unable to identify channel for IDs {target_ids}, scores={scores}")
    return best_channel, best_score, scores


def autodetect_master_slave_bus_configs(
    master_motor_configs: dict,
    slave_motor_configs: dict,
    *,
    candidate_channels: Optional[list[str]] = None,
    min_hits_per_side: int = 2,
) -> tuple[dict[str, str], dict[str, str]]:
    channels = candidate_channels or [f"can{i}" for i in range(0, 8)]
    used_channels: set[str] = set()

    master_left_ids = _ids_for_side(master_motor_configs, "left")
    master_right_ids = _ids_for_side(master_motor_configs, "right")
    slave_left_ids = _ids_for_side(slave_motor_configs, "left")
    slave_right_ids = _ids_for_side(slave_motor_configs, "right")

    master_left, _, _ = _pick_best_channel(channels, master_left_ids, used_channels, min_hits=min_hits_per_side)
    used_channels.add(master_left)
    master_right, _, _ = _pick_best_channel(channels, master_right_ids, used_channels, min_hits=min_hits_per_side)
    used_channels.add(master_right)
    slave_left, _, _ = _pick_best_channel(channels, slave_left_ids, used_channels, min_hits=min_hits_per_side)
    used_channels.add(slave_left)
    slave_right, _, _ = _pick_best_channel(channels, slave_right_ids, used_channels, min_hits=min_hits_per_side)

    master_bus_configs = {"left": master_left, "right": master_right}
    slave_bus_configs = {"left": slave_left, "right": slave_right}
    return master_bus_configs, slave_bus_configs


def flush_bus_rx(bus) -> None:
    try:
        while bus.channel_handler.recv(timeout=0) is not None:
            pass
    except Exception:
        pass


def _decode_status_frame(bus, motor_name: str, data: bytes) -> tuple[float, float, float, float]:
    model = bus.motors[motor_name].model
    position_u16, velocity_u16, torque_i16, temperature_u16 = struct.unpack(">HHHH", data)

    position = (float(position_u16) / 0x7FFF - 1.0) * MODEL_MIT_POSITION_TABLE[model]
    velocity = (float(velocity_u16) / 0x7FFF - 1.0) * MODEL_MIT_VELOCITY_TABLE[model]
    torque = (float(torque_i16) / 0x7FFF - 1.0) * MODEL_MIT_TORQUE_TABLE[model]
    temperature = float(temperature_u16) * 0.1

    if bus.calibration:
        calibration = bus.calibration[motor_name]
        position = (position - calibration["homing_offset"]) * calibration["direction"]
        velocity = velocity * calibration["direction"]
        torque = torque * calibration["direction"]

    return position, velocity, torque, temperature


def read_operation_frame_for_motor(bus, motor_name: str, *, timeout: float = 0.12) -> tuple[float, float, float, float]:
    expected_id = bus.motors[motor_name].id
    deadline = time.time() + timeout

    while time.time() < deadline:
        remaining = max(0.0, deadline - time.time())
        frame = bus.channel_handler.recv(timeout=remaining)
        if frame is None:
            continue
        if not frame.is_extended_id:
            continue

        communication_type = (frame.arbitration_id >> 24) & 0x1F
        extra_data = (frame.arbitration_id >> 8) & 0xFFFF
        device_id = extra_data & 0xFF

        if communication_type not in [CommunicationType.OPERATION_STATUS, CommunicationType.FAULT_REPORT]:
            continue
        if device_id != expected_id:
            continue

        if communication_type == CommunicationType.FAULT_REPORT:
            raise RuntimeError(f"Received fault frame from {motor_name}")

        return _decode_status_frame(bus, motor_name, frame.data)

    raise RuntimeError(f"No response from the motor {motor_name}")


def write_operation_and_read(
    bus,
    motor_name: str,
    position: float,
    kp: float,
    kd: float,
    *,
    velocity: float = 0.0,
    torque: float = 0.0,
    timeout: float = 0.12,
) -> tuple[float, float, float, float]:
    flush_bus_rx(bus)
    bus.write_operation_frame(motor_name, position, kp, kd, velocity=velocity, torque=torque)
    return read_operation_frame_for_motor(bus, motor_name, timeout=timeout)


def _pack_parameter_value(param_dtype, value):
    match param_dtype:
        case np.uint8:
            return struct.pack("<BBH", value, 0, 0)
        case np.int8:
            return struct.pack("<bBH", value, 0, 0)
        case np.uint16:
            return struct.pack("<HH", value, 0)
        case np.int16:
            return struct.pack("<hH", value, 0)
        case np.uint32:
            return struct.pack("<L", value)
        case np.int32:
            return struct.pack("<l", value)
        case np.float32:
            return struct.pack("<f", value)
        case _:
            raise ValueError(f"Unsupported parameter type: {param_dtype}")


def write_parameter_for_motor(bus, motor_name: str, parameter_type, value, *, timeout: float = 0.12) -> None:
    device_id = bus.motors[motor_name].id
    param_id, param_dtype, _ = parameter_type
    value_buffer = _pack_parameter_value(param_dtype, value)
    data = struct.pack("<HH", param_id, 0x00) + value_buffer
    flush_bus_rx(bus)
    bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, device_id, data)
    read_operation_frame_for_motor(bus, motor_name, timeout=timeout)


def enable_motor(bus, motor_name: str, *, timeout: float = 0.12) -> None:
    device_id = bus.motors[motor_name].id
    flush_bus_rx(bus)
    bus.transmit(CommunicationType.ENABLE, bus.host_id, device_id)
    read_operation_frame_for_motor(bus, motor_name, timeout=timeout)


def disable_motor(bus, motor_name: str, *, timeout: float = 0.12) -> None:
    device_id = bus.motors[motor_name].id
    flush_bus_rx(bus)
    bus.transmit(CommunicationType.DISABLE, bus.host_id, device_id)
    read_operation_frame_for_motor(bus, motor_name, timeout=timeout)


def set_torque_limits(
    arm: Arm,
    torque_limits: Optional[dict[str, float]] = None,
    *,
    strict: bool = False,
    on_error: ErrorHandler = _default_error_handler,
) -> None:
    for motor_name, entry in arm.motors.items():
        bus, motor = entry
        torque_limit = _resolve_torque_limit(motor.model, torque_limits)
        try:
            write_parameter_for_motor(bus, motor_name, ParameterType.TORQUE_LIMIT, torque_limit)
        except Exception as e:
            if strict:
                raise
            on_error(f"failed to set torque limit for {motor_name}", e)


def enable_motors(
    arm: Arm,
    *,
    strict: bool = False,
    on_error: ErrorHandler = _default_error_handler,
) -> None:
    for motor_name, entry in arm.motors.items():
        bus, _ = entry
        try:
            enable_motor(bus, motor_name)
        except Exception as e:
            if strict:
                raise
            on_error(f"failed to enable {motor_name}", e)


def prepare_arm(
    arm: Arm,
    torque_limits: Optional[dict[str, float]] = None,
    *,
    strict: bool = False,
    on_error: ErrorHandler = _default_error_handler,
) -> None:
    set_torque_limits(arm, torque_limits=torque_limits, strict=strict, on_error=on_error)
    enable_motors(arm, strict=strict, on_error=on_error)


def shutdown_arm(arm: Arm, *, damping: float = 0.5) -> None:
    for motor_name, entry in arm.motors.items():
        bus, _ = entry
        try:
            write_operation_and_read(bus, motor_name, position=0.0, kp=0.0, kd=damping, velocity=0.0)
            disable_motor(bus, motor_name)
        except Exception:
            pass

    for _, bus in arm.buses.items():
        try:
            bus.disconnect()
        except Exception:
            pass


def detect_active_motors(
    master_arm: Arm,
    slave_arm: Arm,
    *,
    slave_kp: float,
    slave_kd: float,
    on_error: ErrorHandler = _default_error_handler,
) -> list[str]:
    active = []
    for motor_name in master_arm.motors.keys():
        master_bus, _ = master_arm.motors[motor_name]
        slave_bus, _ = slave_arm.motors[motor_name]
        try:
            m_pos, m_vel, _, _ = write_operation_and_read(
                master_bus,
                motor_name,
                0.0,
                0.0,
                0.0,
                torque=0.0,
            )
            if m_pos is None or m_vel is None:
                raise RuntimeError("no master feedback")
            write_operation_and_read(
                slave_bus,
                motor_name,
                m_pos,
                slave_kp,
                slave_kd,
                velocity=m_vel,
            )
            active.append(motor_name)
        except Exception as e:
            on_error(f"Skipping {motor_name}", e)
    return active
