"""Manual demo in simulation using the latest CEM result."""

import argparse
import json
import time
from pathlib import Path
from typing import Optional

import pybullet as p

from sim.poc_env import PocEnv
from sim.world import World, WorldConfig


def _load_session_config(path: Path, session_name: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    sessions = data.get("sessions", {})
    if isinstance(sessions, list):
        sessions = {item["name"]: item for item in sessions if isinstance(item, dict) and "name" in item}
    if session_name not in sessions:
        known = ", ".join(sorted(sessions.keys()))
        raise KeyError(f"session '{session_name}' not found (known: {known})")
    return sessions[session_name]


def _setup_world(gui: bool, session_cfg: Optional[dict]):
    cfg = WorldConfig(gui=gui, robot_type="ur5")
    if session_cfg:
        cfg.benchmark_part_id = session_cfg.get("benchmark_part_id", cfg.benchmark_part_id)
        cfg.benchmark_pos = tuple(session_cfg.get("benchmark_pos", cfg.benchmark_pos))
        cfg.benchmark_rpy = tuple(session_cfg.get("benchmark_rpy", cfg.benchmark_rpy))
        pre_grasp_joints = session_cfg.get("pre_grasp_joint_positions")
        if pre_grasp_joints is not None:
            cfg.home_joint_positions = tuple(pre_grasp_joints)
    world = World(cfg)
    env = PocEnv(world=world)
    env.reset()
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=120,
        cameraPitch=-39.80,
        cameraTargetPosition=[0.02, -0.26, -0.31],
    )
    return world, env


def main():
    parser = argparse.ArgumentParser(description="Run a sim demo with cem_best.json.")
    parser.add_argument("--best-path", default="shared/cem_best.json", help="Path to cem_best.json")
    parser.add_argument("--session", default=None, help="Session name from rl/train_sessions.json")
    parser.add_argument("--gui", default="True", choices=["True", "False"], help="Enable GUI (True/False).")
    args = parser.parse_args()

    best_path = Path(args.best_path)
    if not best_path.exists():
        raise FileNotFoundError(f"best file not found: {best_path}")
    best = json.loads(best_path.read_text(encoding="utf-8"))
    best_q = best.get("best_q")
    if not isinstance(best_q, list) or len(best_q) != 10:
        raise ValueError("best_q must be a list of 10 values in cem_best.json")

    session_name = args.session or best.get("session")
    session_cfg = None
    if session_name:
        session_path = Path(__file__).parents[1] / "rl" / "train_sessions.json"
        session_cfg = _load_session_config(session_path, session_name)

    world, env = _setup_world(gui=(args.gui == "True"), session_cfg=session_cfg)
    try:
        print("Controls: r = reset, b = eval best, k = print hand joints, q = quit")
        key_trigger = p.KEY_WAS_TRIGGERED

        while True:
            keys = p.getKeyboardEvents()
            if keys.get(ord("r"), 0) & key_trigger:
                env.reset()
                print("reset done")
            if keys.get(ord("b"), 0) & key_trigger:
                reward, info = env.evaluate(best_q)
                print("best reward:", reward)
                print("best info:", info)
            if keys.get(ord("k"), 0) & key_trigger:
                print("hand joints:", env.world.hand.get_current_joint_command_vector())
            if keys.get(ord("q"), 0) & key_trigger:
                break
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        world.close()


if __name__ == "__main__":
    main()
