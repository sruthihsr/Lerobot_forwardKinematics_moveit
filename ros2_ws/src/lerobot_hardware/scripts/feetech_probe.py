#!/usr/bin/env python3
"""
Read-only diagnostic for the SO101 Feetech servo bus.

Opens the serial port, pings servo IDs 1-6, and prints each one's live
Min/Max Position Limit and current Present Position. Performs NO writes to
any servo register -- run this first to confirm the port/servos respond as
expected before anything else (the hardware bridge, MoveIt execution) is
allowed to touch the arm.

Usage:
    python3 feetech_probe.py [--port /dev/ttyACM0]
"""

import argparse
import math

from feetech_bus import FeetechBus, MAX_RES, _decode_sign_magnitude, POSITION_SIGN_BIT, ADDR_PRESENT_POSITION

JOINT_NAMES = ["1", "2", "3", "4", "5", "6"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args()

    bus = FeetechBus(args.port)
    print(f"Connecting to {args.port} (read-only, no servo writes)...")
    bus.connect_read_only()

    print("\n" + "=" * 70)
    print(f"{'joint':<6} {'min_raw':>8} {'max_raw':>8} {'present_raw':>12} {'present_rad':>12}")
    print("=" * 70)
    for joint_name, servo_id in zip(JOINT_NAMES, bus.servo_ids):
        min_raw, max_raw = bus.limits[servo_id]
        raw_encoded = bus._read(servo_id, ADDR_PRESENT_POSITION)
        raw = _decode_sign_magnitude(raw_encoded, POSITION_SIGN_BIT)
        rad = bus.raw_to_rad(servo_id, raw)
        print(f"{joint_name:<6} {min_raw:>8} {max_raw:>8} {raw:>12} {rad:>12.4f}")

    bus.close()
    print("\nNo writes were sent. Compare these ranges against the URDF")
    print("joint limits in so101_base.xacro before proceeding.")


if __name__ == "__main__":
    main()
