"""Run learned grasp on hardware and slightly lift the Sawyer arm."""

import argparse
import json
import time
from pathlib import Path
from typing import Optional

from shared.mapping import ensure_real_targets
from real.AR10_Extended import hand
from real.sawyer_arm import SawyerArm


def _lift_arm(
    limb_name: str,
    joint_name: str,
    delta: float,
    timeout: Optional[float],
):
    arm = SawyerArm(limb_name=limb_name)
    angles = arm.joint_angles()
    if joint_name not in angles:
        known = ", ".join(sorted(angles.keys()))
        raise KeyError(f"joint '{joint_name}' not found (known: {known})")
    angles[joint_name] += float(delta)
    arm.move_to_joint_positions(angles, timeout=timeout)


def main():
    parser = argparse.ArgumentParser(description="Run learned grasp on the real hand and lift Sawyer slightly.")
    parser.add_argument("--input", default="shared/cem_best.json", help="Path to cem_best.json")
    parser.add_argument("--sleep-hand", type=float, default=2.0, help="Seconds to wait after sending hand targets.")
    parser.add_argument("--limb", default="right", help="Sawyer limb name (usually 'right').")
    parser.add_argument(
        "--lift-joint",
        default="right_j1",
        help="Joint to offset (e.g. right_j1). Defaults to 'right_j1'.",
    )
    parser.add_argument("--lift-delta", type=float, default=0.1, help="Joint delta (radians) for the lift.")
    parser.add_argument("--lift-timeout", type=float, default=2.0, help="Timeout for Sawyer motion.")
    args = parser.parse_args()

    in_path = Path(args.input)
    record = json.loads(in_path.read_text(encoding="utf-8"))
    record = ensure_real_targets(record)
    targets = record.get("real_targets")
    print("pose:", record.get("pose_name"))
    print("real targets:", targets)

    ar10 = hand()
    ar10.motion_manager.multi_move(targets)
    if args.sleep_hand > 0:
        time.sleep(args.sleep_hand)

    _lift_arm(args.limb, args.lift_joint, args.lift_delta, args.lift_timeout)


if __name__ == "__main__":
    main()
