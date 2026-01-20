"""Interactive tool to tune pre-grasp arm joints using IK."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import pybullet as p

from sim.world import World, WorldConfig


def load_session(path: Path, name: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    sessions = data.get("sessions", {})
    if isinstance(sessions, list):
        sessions = {item["name"]: item for item in sessions if isinstance(item, dict) and "name" in item}
    if name not in sessions:
        known = ", ".join(sorted(sessions.keys()))
        raise KeyError(f"session '{name}' not found (known: {known})")
    return sessions[name]


def main():
    parser = argparse.ArgumentParser(description="Tune pre-grasp arm joint angles.")
    parser.add_argument("--session", default=None, help="Session name from rl/train_sessions.json")
    parser.add_argument("--part-id", type=int, default=None, help="Benchmark part id (1..14)")
    args = parser.parse_args()

    session_cfg = None
    if args.session:
        session_path = Path(__file__).parents[1] / "rl" / "train_sessions.json"
        session_cfg = load_session(session_path, args.session)

    benchmark_part_id = args.part_id
    if benchmark_part_id is None and session_cfg:
        benchmark_part_id = session_cfg.get("benchmark_part_id")
    if benchmark_part_id is None:
        benchmark_part_id = 3

    world = World(WorldConfig(gui=True, robot_type="ur5", benchmark_part_id=benchmark_part_id))
    try:
        world.reset_world()

        target_xyz, target_rpy = world.arm.ee_pose()
        target_xyz = np.array(target_xyz, dtype=float)
        target_rpy = np.array(target_rpy, dtype=float)

        print("Controls:")
        print("  Move:  arrows (x/y), PgUp/PgDn (z)")
        print("  Hand:  o = open, c = close")
        print("  Pose:  k = print current arm joint angles")
        print("Press Ctrl+C to quit.")

        step = 0.002
        key_down = p.KEY_IS_DOWN
        key_trigger = p.KEY_WAS_TRIGGERED

        while True:
            keys = p.getKeyboardEvents()
            delta = np.zeros(3, dtype=float)

            if keys.get(p.B3G_UP_ARROW, 0) & key_down:
                delta[0] += step
            if keys.get(p.B3G_DOWN_ARROW, 0) & key_down:
                delta[0] -= step
            if keys.get(p.B3G_RIGHT_ARROW, 0) & key_down:
                delta[1] -= step
            if keys.get(p.B3G_LEFT_ARROW, 0) & key_down:
                delta[1] += step
            if keys.get(p.B3G_PAGE_UP, 0) & key_down:
                delta[2] += step
            if keys.get(p.B3G_PAGE_DOWN, 0) & key_down:
                delta[2] -= step

            if np.any(delta):
                target_xyz = target_xyz + delta
                q = world.arm._ik(target_xyz, target_rpy)
                p.setJointMotorControlArray(
                    world.robot_id,
                    world.arm.joints,
                    p.POSITION_CONTROL,
                    targetPositions=q.tolist(),
                    positionGains=[0.3] * len(world.arm.joints),
                    forces=[200.0] * len(world.arm.joints),
                )

            if keys.get(ord("k"), 0) & key_trigger:
                joint_positions = [p.getJointState(world.robot_id, j)[0] for j in world.arm.joints]
                print("arm joints:", joint_positions)
            if keys.get(ord("o"), 0) & key_trigger:
                world.hand.open_all_fingers()
            if keys.get(ord("c"), 0) & key_trigger:
                world.hand.close_all_fingers()

            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        world.close()


if __name__ == "__main__":
    main()
