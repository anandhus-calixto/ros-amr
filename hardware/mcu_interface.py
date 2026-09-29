#!/usr/bin/env python3
"""Low-level MCU serial bench-test tool.

Mirrors the wire protocol used by hardware/diffbot_system.cpp and
hardware/include/calixto-ros-bot/mcu_comms.hpp (the calixto_ros_bot/
DiffBotSystemHardware ros2_control plugin), so the MCU firmware can be
exercised directly over serial without bringing up the full ros2_control /
diff_drive_controller stack.

This is a true 2-wheel differential drive robot (one left wheel, one right
wheel — no front/rear pairs). PID is closed-loop on the motor driver itself;
the MCU only needs to forward target wheel velocities and encoder ticks.

Protocol (binary, standardized 2026-09-29 - was plain ASCII before that):

  Command Packet (fire-and-forget, no reply), 15 bytes:
      [0xA5][cmd_id][seq_u16][cmd_vel_left_f32][cmd_vel_right_f32][control_flags][crc8][0x00]
      Velocities in rad/s (HW_IF_VELOCITY) — see diffbot_system.cpp write().

  Telemetry Request (2 bytes) -> Telemetry Packet reply (24 bytes):
      Request: [0xE5][crc8]
      Reply:   [0x5A][seq_u16][ticks_left_i32][ticks_right_i32]
               [vel_left_f32][vel_right_f32][status_flags][fault_code][battery_soc][crc8][0x00]

  All multi-byte fields little-endian. CRC-8: polynomial 0x07, init 0x00, no
  reflection. See calixto-amr-imx-rt-app/calixto-amr-info.md's "MCU↔MPU
  Binary Packet Protocol" section for the full spec.

Serial parameters (from description/diffbot.ros2_control.xacro):
  device=/dev/ttyLP2  baud=115200  timeout_ms=30
(This is the port on the robot's onboard computer, not the RF remote's
/dev/ttyUSB* receiver — run this script on the robot, or point --device
at whatever serial device the MCU is actually attached to.)

Robot geometry (from calixto-ros-bot.md / config/diffbot_controllers.yaml):
  wheel_separation = 0.35 m   (350 mm)
  wheel_radius     = 0.085 m  (170 mm diameter)
"""
import argparse
import struct
import time

import serial

DEFAULT_DEVICE = "/dev/ttyLP2"
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 0.03  # 30 ms, matches xacro timeout_ms

WHEEL_SEPARATION = 0.35  # m
WHEEL_RADIUS = 0.085  # m

SYNC_TELEMETRY_REQUEST = 0xE5  # i.MX 95 -> i.MX RT
SYNC_COMMAND = 0xA5            # i.MX 95 -> i.MX RT
SYNC_TELEMETRY = 0x5A          # i.MX RT -> i.MX 95
CMD_ID_DRIVE_VELOCITY = 0x01


def _crc8(data):
    """CRC-8, polynomial 0x07, init 0x00, no reflection, no final XOR - must
    match src/mcu_frame/mcu_frame_protocol.c on the MCU exactly."""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class McuComms:
    """Python port of calixto-ros-bot/mcu_comms.hpp - binary protocol
    (standardized 2026-09-29), same wire format as
    calixto-amr-imx-rt-app/src/mcu_frame/mcu_frame_protocol.h."""

    def __init__(self, device=DEFAULT_DEVICE, baud=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT):
        self._ser = serial.Serial(device, baud, timeout=timeout)
        self._seq = 0

    def disconnect(self):
        self._ser.close()

    def connected(self):
        return self._ser.is_open

    def read_encoder_values(self):
        """Sends a Telemetry Request and blocks for the Telemetry Packet
        reply (matches the old "e\\r"-request shape, just binary now).
        Returns (0, 0) on timeout, a bad sync byte, or a CRC mismatch -
        distinguishable from a genuine 0,0 reading only by the printed
        warning, same as the old ASCII path's effective behaviour."""
        req_body = bytes([SYNC_TELEMETRY_REQUEST])
        self._ser.write(req_body + bytes([_crc8(req_body)]))

        reply = self._ser.read(24)
        if len(reply) != 24:
            print(f"Warning: telemetry read timed out (got {len(reply)}/24 bytes).")
            return 0, 0

        header, seq, ticks_l, ticks_r, vel_l, vel_r, status, fault, batt, crc, _pad = \
            struct.unpack("<BHiiffBBBBB", reply)

        if header != SYNC_TELEMETRY:
            print(f"Warning: telemetry packet bad sync byte: 0x{header:02X}")
            return 0, 0
        if _crc8(reply[:22]) != crc:
            print("Warning: telemetry packet CRC mismatch.")
            return 0, 0

        return ticks_l, ticks_r

    def set_motor_values(self, left, right):
        """Fire-and-forget - firmware sends no reply to a Command Packet."""
        body = struct.pack(
            "<BBHffB",
            SYNC_COMMAND, CMD_ID_DRIVE_VELOCITY,
            self._seq & 0xFFFF, left, right,
            0x00,  # control_flags - not yet acted on by the firmware
        )
        self._ser.write(body + bytes([_crc8(body), 0x00]))  # crc8 + padding
        self._seq += 1


def twist_to_wheel_speeds(linear_x, angular_z, wheel_separation=WHEEL_SEPARATION, wheel_radius=WHEEL_RADIUS):
    """Differential-drive kinematics -> per-side wheel angular velocity (rad/s)."""
    left = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
    right = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius
    return left, right


def clamp(value, limit):
    return max(-limit, min(limit, value))


def clamp_and_warn(value, limit, label):
    clamped = clamp(value, limit)
    if clamped != value:
        print(f"Warning: {label}={value} exceeds limit +/-{limit}, clamping to {clamped}")
    return clamped


def stream_motor_values(mcu, left, right, rate_hz):
    """Keep re-sending the same wheel command at rate_hz until Ctrl+C, then
    send a zero-velocity stop. This mirrors how the real ros2_control write()
    loop refreshes the command continuously (controller_manager update_rate
    is 20 Hz, and diffbot_controllers.yaml sets cmd_vel_timeout: 0.5s), so a
    single one-shot write is not representative of real operation.
    """
    period = 1.0 / rate_hz
    print(f"Streaming at {rate_hz:.1f} Hz. Press Ctrl+C to stop "
          f"(a zero-velocity stop will be sent first).")
    count = 0
    start = time.time()
    try:
        while True:
            mcu.set_motor_values(left, right)
            count += 1
            elapsed = time.time() - start
            print(f"\r[{elapsed:7.2f}s #{count:5d}] sent -> "
                  f"left={left:.4f} right={right:.4f}", end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping: sending zero velocity.")
        mcu.set_motor_values(0.0, 0.0)


def main():
    ap = argparse.ArgumentParser(description="Bench-test the low-level MCU serial link.")
    ap.add_argument("--device", default=DEFAULT_DEVICE)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    ap.add_argument("--max-linear", type=float, default=2.0,
                     help="Clamp |linear.x| to this, in m/s (default 2.0, matches "
                          "diffbot_controllers.yaml linear.x max_velocity)")
    ap.add_argument("--max-angular", type=float, default=4.0,
                     help="Clamp |angular.z| to this, in rad/s (default 4.0, matches "
                          "diffbot_controllers.yaml angular.z max_velocity)")
    sub = ap.add_subparsers(dest="mode", required=True)

    p_vel = sub.add_parser("vel", help="Send a linear/angular velocity command")
    p_vel.add_argument("linear_x", type=float)
    p_vel.add_argument("angular_z", type=float)
    p_vel.add_argument("--once", action="store_true",
                        help="Send once and exit instead of streaming until Ctrl+C")
    p_vel.add_argument("--rate", type=float, default=20.0,
                        help="Refresh rate in Hz when streaming (default 20, matches "
                             "controller_manager update_rate)")

    p_raw = sub.add_parser("raw", help="Send raw per-wheel velocities directly")
    p_raw.add_argument("left", type=float)
    p_raw.add_argument("right", type=float)
    p_raw.add_argument("--once", action="store_true",
                        help="Send once and exit instead of streaming until Ctrl+C")
    p_raw.add_argument("--rate", type=float, default=20.0,
                        help="Refresh rate in Hz when streaming (default 20)")

    sub.add_parser("stop", help="Send zero velocity to all wheels")

    p_enc = sub.add_parser("encoders", help="Poll and print encoder ticks")
    p_enc.add_argument("--rate", type=float, default=2.0, help="Polls per second")
    p_enc.add_argument("--count", type=int, default=10, help="Number of polls (0 = forever)")

    args = ap.parse_args()

    mcu = McuComms(args.device, args.baud, args.timeout)
    try:
        if args.mode == "vel":
            linear_x = clamp_and_warn(args.linear_x, args.max_linear, "linear.x")
            angular_z = clamp_and_warn(args.angular_z, args.max_angular, "angular.z")
            left, right = twist_to_wheel_speeds(linear_x, angular_z)
            print(f"linear.x={linear_x} angular.z={angular_z} "
                  f"-> left={left:.4f} right={right:.4f} rad/s")
            if args.once:
                mcu.set_motor_values(left, right)
            else:
                stream_motor_values(mcu, left, right, args.rate)
        elif args.mode == "raw":
            # Bound each wheel by the fastest speed reachable within the
            # linear/angular limits (worst case: max linear + max angular combined).
            max_wheel = max(abs(v) for v in twist_to_wheel_speeds(args.max_linear, args.max_angular))
            left = clamp_and_warn(args.left, max_wheel, "left")
            right = clamp_and_warn(args.right, max_wheel, "right")
            if args.once:
                mcu.set_motor_values(left, right)
            else:
                stream_motor_values(mcu, left, right, args.rate)
        elif args.mode == "stop":
            mcu.set_motor_values(0.0, 0.0)
        elif args.mode == "encoders":
            n = 0
            while args.count == 0 or n < args.count:
                left, right = mcu.read_encoder_values()
                print(f"left={left} right={right}")
                n += 1
                time.sleep(1.0 / args.rate)
    finally:
        mcu.disconnect()


if __name__ == "__main__":
    main()
