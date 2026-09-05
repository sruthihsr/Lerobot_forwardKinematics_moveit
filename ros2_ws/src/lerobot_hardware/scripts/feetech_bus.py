#!/usr/bin/env python3
"""
Minimal Feetech STS3215 serial driver for the SO101 arm.

Mirrors the register map and encoding used by LeRobot's FeetechMotorsBus
(https://github.com/huggingface/lerobot -- src/lerobot/motors/feetech/{feetech.py,tables.py},
src/lerobot/motors/motors_bus.py) closely enough to talk to the same servos,
without depending on the full lerobot package (its distribution isn't
importable from ROS's Python 3.12 interpreter -- lerobot is installed under
Python 3.13 environments, and rclpy's compiled extension is ABI-locked to
3.12).

Calibration (per-servo min/max raw position) is read live from each servo's
own EEPROM on connect rather than from a calibration JSON file on disk, so
this always reflects whatever is actually flashed onto the physical unit
that responds on the given port.
"""

import math
import os

import scservo_sdk as scs
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# Raw tick captured per servo with the physical arm posed at the URDF's
# zero/home pose -- see config/so101_calibration.yaml. Used as each joint's
# zero-reference instead of the servo's live EEPROM min/max midpoint, which
# does not generally line up with the URDF's own zero.
try:
    CALIBRATION_PATH = os.path.join(
        get_package_share_directory("lerobot_hardware"), "config", "so101_calibration.yaml"
    )
except PackageNotFoundError:
    # Not run through a sourced ROS install (e.g. a standalone script/test) --
    # fall back to the path relative to this file's own (possibly symlinked)
    # source location.
    CALIBRATION_PATH = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "..", "config", "so101_calibration.yaml"
    )

BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0  # STS/SMS series (matches lerobot's DEFAULT_PROTOCOL_VERSION for sts3215)

# (address, size_bytes) -- from lerobot's STS_SMS_SERIES_CONTROL_TABLE
ADDR_TORQUE_ENABLE = (40, 1)
ADDR_ACCELERATION = (41, 1)
ADDR_GOAL_POSITION = (42, 2)
ADDR_GOAL_VELOCITY = (46, 2)
ADDR_MIN_POSITION_LIMIT = (9, 2)
ADDR_MAX_POSITION_LIMIT = (11, 2)
ADDR_PRESENT_POSITION = (56, 2)
ADDR_PHASE = (18, 1)

# Goal_Position / Present_Position use sign-magnitude encoding at bit 15 on
# STS/SMS series (lerobot's STS_SMS_SERIES_ENCODINGS_TABLE). Position
# magnitudes here are always non-negative (0..4095) so this is a no-op in
# practice, but it's applied for correctness / to match tested behavior.
POSITION_SIGN_BIT = 15

RESOLUTION = 4096  # ticks per revolution, STS3215
MAX_RES = RESOLUTION - 1

# A commanded position that would need clamping by more than this many raw
# ticks (~0.02 rad) aborts instead of silently executing a different pose.
CLAMP_ABORT_TICKS = round(0.02 * MAX_RES / (2 * math.pi))


def _encode_sign_magnitude(value, sign_bit):
    max_magnitude = (1 << sign_bit) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError(f"magnitude {magnitude} exceeds {max_magnitude}")
    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit) | magnitude


def _decode_sign_magnitude(encoded, sign_bit):
    direction_bit = (encoded >> sign_bit) & 1
    magnitude = encoded & ((1 << sign_bit) - 1)
    return -magnitude if direction_bit else magnitude


class ClampExceeded(RuntimeError):
    """A commanded position needed clamping beyond CLAMP_ABORT_TICKS."""


class FeetechBus:
    """Talks to STS3215 servos on a single serial bus."""

    def __init__(self, port, servo_ids=(1, 2, 3, 4, 5, 6),
                 acceleration=20, goal_velocity=150, logger=None):
        self.port_name = port
        self.servo_ids = list(servo_ids)
        self.acceleration = acceleration
        self.goal_velocity = goal_velocity
        self.logger = logger

        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)

        self.limits = {}  # servo_id -> (min_raw, max_raw)
        self.zero_raw = self._load_calibration()

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def connect(self):
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {self.port_name}")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate {BAUDRATE} on {self.port_name}")

        for servo_id in self.servo_ids:
            model_number, comm_result, _error = self.packet_handler.ping(self.port_handler, servo_id)
            if comm_result != scs.COMM_SUCCESS:
                raise RuntimeError(
                    f"Servo id={servo_id} did not respond on {self.port_name} "
                    f"({self.packet_handler.getTxRxResult(comm_result)})"
                )
            self._log(f"servo {servo_id}: ping OK (model_number={model_number})")

        for servo_id in self.servo_ids:
            self._clear_phase_bit(servo_id)
            min_raw = self._read(servo_id, ADDR_MIN_POSITION_LIMIT)
            max_raw = self._read(servo_id, ADDR_MAX_POSITION_LIMIT)
            self.limits[servo_id] = (min_raw, max_raw)
            self._log(f"servo {servo_id}: live limits raw=[{min_raw}, {max_raw}]")

        for servo_id in self.servo_ids:
            self._write(servo_id, ADDR_ACCELERATION, self.acceleration)
            self._write(servo_id, ADDR_GOAL_VELOCITY, self.goal_velocity)
            self._write(servo_id, ADDR_TORQUE_ENABLE, 1)

    def connect_read_only(self):
        """Like connect(), but never writes anything to the servos: no
        Phase-bit clear, no Acceleration/Goal_Velocity/Torque_Enable. Used
        by the diagnostic probe to confirm the port/servos respond before
        anything is allowed to touch hardware registers."""
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port {self.port_name}")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate {BAUDRATE} on {self.port_name}")

        for servo_id in self.servo_ids:
            model_number, comm_result, _error = self.packet_handler.ping(self.port_handler, servo_id)
            if comm_result != scs.COMM_SUCCESS:
                raise RuntimeError(
                    f"Servo id={servo_id} did not respond on {self.port_name} "
                    f"({self.packet_handler.getTxRxResult(comm_result)})"
                )
            self._log(f"servo {servo_id}: ping OK (model_number={model_number})")

        for servo_id in self.servo_ids:
            min_raw = self._read(servo_id, ADDR_MIN_POSITION_LIMIT)
            max_raw = self._read(servo_id, ADDR_MAX_POSITION_LIMIT)
            self.limits[servo_id] = (min_raw, max_raw)

    def _clear_phase_bit(self, servo_id):
        # Forces Present_Position readings into [0, resolution-1] instead of
        # a signed/wrapped range. Only known to be necessary for the
        # STS3215 (matches lerobot's configure_motors()).
        phase = self._read(servo_id, ADDR_PHASE)
        if phase & 0x10:
            self._write(servo_id, ADDR_PHASE, phase & ~0x10)

    def _read(self, servo_id, addr_spec):
        address, size = addr_spec
        if size == 1:
            value, comm_result, _error = self.packet_handler.read1ByteTxRx(self.port_handler, servo_id, address)
        else:
            value, comm_result, _error = self.packet_handler.read2ByteTxRx(self.port_handler, servo_id, address)
        if comm_result != scs.COMM_SUCCESS:
            raise RuntimeError(
                f"Read failed servo={servo_id} addr={address}: "
                f"{self.packet_handler.getTxRxResult(comm_result)}"
            )
        return value

    def _write(self, servo_id, addr_spec, value):
        address, size = addr_spec
        if size == 1:
            comm_result, _error = self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, address, value)
        else:
            comm_result, _error = self.packet_handler.write2ByteTxRx(self.port_handler, servo_id, address, value)
        if comm_result != scs.COMM_SUCCESS:
            raise RuntimeError(
                f"Write failed servo={servo_id} addr={address} value={value}: "
                f"{self.packet_handler.getTxRxResult(comm_result)}"
            )

    def _load_calibration(self):
        if not os.path.exists(CALIBRATION_PATH):
            self._log(
                f"WARNING: no calibration file at {CALIBRATION_PATH} -- falling "
                f"back to each servo's live raw-range midpoint as zero, which "
                f"will not match the URDF's zero pose. Run the home-pose "
                f"calibration to fix this."
            )
            return {}
        with open(CALIBRATION_PATH) as f:
            data = yaml.safe_load(f)
        return {int(k): v for k, v in data.get("home_raw", {}).items()}

    def _zero(self, servo_id):
        if servo_id in self.zero_raw:
            return self.zero_raw[servo_id]
        min_raw, max_raw = self.limits[servo_id]
        return (min_raw + max_raw) / 2.0

    def raw_to_rad(self, servo_id, raw):
        return (raw - self._zero(servo_id)) * (2 * math.pi / MAX_RES)

    def rad_to_raw(self, servo_id, rad):
        return round(rad * MAX_RES / (2 * math.pi) + self._zero(servo_id))

    def read_positions_rad(self):
        """Returns {servo_id: radians} for all servos, via GroupSyncRead."""
        address, size = ADDR_PRESENT_POSITION
        sync_read = scs.GroupSyncRead(self.port_handler, self.packet_handler, address, size)
        for servo_id in self.servo_ids:
            sync_read.addParam(servo_id)

        comm_result = sync_read.txRxPacket()
        if comm_result != scs.COMM_SUCCESS:
            raise RuntimeError(f"sync_read failed: {self.packet_handler.getTxRxResult(comm_result)}")

        positions = {}
        for servo_id in self.servo_ids:
            raw_encoded = sync_read.getData(servo_id, address, size)
            raw = _decode_sign_magnitude(raw_encoded, POSITION_SIGN_BIT)
            positions[servo_id] = self.raw_to_rad(servo_id, raw)
        return positions

    def write_positions_rad(self, targets_rad, dry_run=False):
        """targets_rad: {servo_id: radians}.

        Every target is hard-clamped to that servo's live calibrated
        [min_raw, max_raw] range before writing. Raises ClampExceeded
        instead of silently executing a different pose if a target needed
        clamping by more than CLAMP_ABORT_TICKS.
        """
        address, size = ADDR_GOAL_POSITION
        raw_targets = {}
        for servo_id, rad in targets_rad.items():
            min_raw, max_raw = self.limits[servo_id]
            raw = self.rad_to_raw(servo_id, rad)
            clamped = max(min_raw, min(max_raw, raw))
            if abs(clamped - raw) > CLAMP_ABORT_TICKS:
                raise ClampExceeded(
                    f"servo {servo_id}: target {rad:.4f} rad (raw={raw}) is "
                    f"outside calibrated range [{min_raw}, {max_raw}] by "
                    f"{abs(clamped - raw)} ticks"
                )
            raw_targets[servo_id] = clamped

        if dry_run:
            self._log(f"[dry-run] would write Goal_Position: {raw_targets}")
            return

        sync_write = scs.GroupSyncWrite(self.port_handler, self.packet_handler, address, size)
        for servo_id, raw in raw_targets.items():
            encoded = _encode_sign_magnitude(raw, POSITION_SIGN_BIT)
            data = [scs.SCS_LOBYTE(encoded), scs.SCS_HIBYTE(encoded)]
            sync_write.addParam(servo_id, data)

        comm_result = sync_write.txPacket()
        if comm_result != scs.COMM_SUCCESS:
            raise RuntimeError(f"sync_write failed: {self.packet_handler.getTxRxResult(comm_result)}")

    def close(self):
        self.port_handler.closePort()
