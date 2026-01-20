"""Execute learned 10D vector on the real AR10 hand."""

import argparse
import json
import time
from pathlib import Path

from shared.mapping import ensure_real_targets
from real.AR10_Extended import hand


def main():
    parser = argparse.ArgumentParser(description="Prepare real AR10 targets from shared grasp output.")
    parser.add_argument("--input", default="shared/cem_best.json", help="Path to shared grasp output.")
    parser.add_argument("--sleep", type=float, default=2.0, help="Seconds to wait after sending targets.")
    args = parser.parse_args()

    in_path = Path(args.input)
    record = json.loads(in_path.read_text(encoding="utf-8"))
    record = ensure_real_targets(record)
    targets = record.get("real_targets")
    print("pose:", record.get("pose_name"))
    print("real targets:", targets)

    ar10 = hand()
    ar10.motion_manager.multi_move(targets)
    if args.sleep > 0:
        time.sleep(args.sleep)


if __name__ == "__main__":
    main()
