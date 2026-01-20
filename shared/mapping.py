"""Shared sim-to-real mapping utilities."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional


DEFAULT_CALIBRATION_PATH = Path(__file__).with_name("ar10_calibration.json")


def _clip01(values: Iterable[float]) -> List[float]:
    return [min(1.0, max(0.0, float(v))) for v in values]


def load_calibration(path: Optional[Path] = None) -> Dict[str, List[float]]:
    calib_path = path or DEFAULT_CALIBRATION_PATH
    if calib_path.exists():
        data = json.loads(calib_path.read_text(encoding="utf-8"))
        return {
            "servo_targets_min": list(data["servo_targets_min"]),
            "servo_targets_max": list(data["servo_targets_max"]),
        }
    return {
        "servo_targets_min": [4200.0] * 10,
        "servo_targets_max": [7700.0] * 10,
    }


def map_to_targets(
    q01: Iterable[float],
    servo_targets_min: Iterable[float],
    servo_targets_max: Iterable[float],
) -> List[int]:
    q = _clip01(q01)
    mins = list(servo_targets_min)
    maxs = list(servo_targets_max)
    if len(q) != 10 or len(mins) != 10 or len(maxs) != 10:
        raise ValueError("Expected 10 values for q01/min/max.")
    targets = []
    for i in range(10):
        t = mins[i] + q[i] * (maxs[i] - mins[i])
        targets.append(int(round(t)))
    return targets


def build_grasp_record(
    q01: Iterable[float],
    pose_name: Optional[str],
    calibration_path: Optional[Path] = None,
) -> Dict[str, object]:
    calib = load_calibration(calibration_path)
    targets = map_to_targets(q01, calib["servo_targets_min"], calib["servo_targets_max"])
    return {
        "pose_name": pose_name,
        "best_q": _clip01(q01),
        "real_targets": targets,
        "calibration": {
            "servo_targets_min": calib["servo_targets_min"],
            "servo_targets_max": calib["servo_targets_max"],
        },
    }


def ensure_real_targets(
    record: Dict[str, object],
    calibration_path: Optional[Path] = None,
) -> Dict[str, object]:
    if "real_targets" in record:
        return record
    q01 = record.get("best_q")
    if not isinstance(q01, list) or len(q01) != 10:
        raise ValueError("record.best_q must be a list of 10 values.")
    extended = build_grasp_record(q01, record.get("pose_name"), calibration_path)
    for key, value in record.items():
        if key not in extended:
            extended[key] = value
    return extended
