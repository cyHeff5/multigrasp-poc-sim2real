"""Arrow-key teleop for Sawyer joint positions."""

import argparse
import sys
from typing import Dict, Optional

from real.sawyer_arm import SawyerArm


def _read_key() -> Optional[str]:
    try:
        import msvcrt

        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):
            ch2 = msvcrt.getch()
            return {
                b"H": "UP",
                b"P": "DOWN",
                b"K": "LEFT",
                b"M": "RIGHT",
            }.get(ch2)
        if ch in (b"q", b"Q"):
            return "Q"
        return None
    except ImportError:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch in ("q", "Q"):
                return "Q"
            if ch != "\x1b":
                return None
            seq = sys.stdin.read(2)
            return {
                "[A": "UP",
                "[B": "DOWN",
                "[D": "LEFT",
                "[C": "RIGHT",
            }.get(seq)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _get_joint_angles(arm: SawyerArm) -> Dict[str, float]:
    return arm.limb.joint_angles()


def main() -> int:
    parser = argparse.ArgumentParser(description="Teleop Sawyer joints with arrow keys.")
    parser.add_argument("--limb", default="right", help="Sawyer limb name (usually 'right').")
    parser.add_argument("--step", type=float, default=0.1, help="Joint delta (radians) for up/down.")
    args = parser.parse_args()

    arm = SawyerArm(limb_name=args.limb)
    angles = _get_joint_angles(arm)
    joint_names = sorted(angles.keys())
    if not joint_names:
        print("No joints found for limb:", args.limb)
        return 1

    index = 0
    print("Arrow keys: left/right select joint, up/down change by step, q to quit.")
    print("Step:", args.step)
    print("Selected:", joint_names[index], "=", angles[joint_names[index]])

    while True:
        key = _read_key()
        if key is None:
            continue
        if key == "Q":
            print("Exiting teleop.")
            return 0
        if key == "LEFT":
            index = (index - 1) % len(joint_names)
        elif key == "RIGHT":
            index = (index + 1) % len(joint_names)
        elif key in ("UP", "DOWN"):
            angles = _get_joint_angles(arm)
            joint = joint_names[index]
            delta = args.step if key == "UP" else -args.step
            new_value = angles[joint] + delta
            arm.move_to_joint_positions({joint: new_value})
            angles[joint] = new_value
        else:
            continue

        current_joint = joint_names[index]
        current_value = _get_joint_angles(arm)[current_joint]
        print("Selected:", current_joint, "=", current_value)


if __name__ == "__main__":
    raise SystemExit(main())
