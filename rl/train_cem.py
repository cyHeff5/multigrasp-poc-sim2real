"""CEM training for 10D AR10 grasp vector using PocEnv."""

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from sim.poc_env import PocEnv, PocEnvConfig
from sim.world import World, WorldConfig
from shared.mapping import build_grasp_record


@dataclass
class CemConfig:
    population: int = 64
    elites: int = 8
    iterations: int = 30
    seed: int = 42
    min_std: float = 0.05
    init_mean: float = 0.5
    init_std: float = 0.25
    output_dir: str = "outputs"
    sleep_s: float = 0.0
    gui: bool = False
    benchmark_part_id: int = 3
    benchmark_pos: tuple = (0.6, 0.0, 0.05)
    benchmark_rpy: tuple = (0.0, 0.0, 0.0)
    pre_grasp_joint_positions: Optional[tuple] = None


def clip01(x: np.ndarray) -> np.ndarray:
    return np.clip(x, 0.0, 1.0)


def expand_action(q_active: Sequence[float], active_joints: Sequence[int]) -> List[float]:
    q_full = [0.0] * 10
    for i, joint_index in enumerate(active_joints):
        q_full[joint_index] = float(q_active[i])
    return q_full


def evaluate_population(
    env: PocEnv,
    population: np.ndarray,
    it: int,
    active_joints: Sequence[int],
) -> List[float]:
    rewards = []
    for i, q in enumerate(population, start=1):
        env.reset()
        q_full = expand_action(q.tolist(), active_joints)
        reward, _ = env.evaluate(q_full)
        rewards.append(float(reward))
        print(f"iter {it:02d} trial {i:03d} reward {reward:.3f}")
    return rewards


def _load_session_config(path: Path, session_name: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    sessions = data.get("sessions", {})
    if isinstance(sessions, list):
        by_name = {}
        for item in sessions:
            if isinstance(item, dict) and "name" in item:
                by_name[item["name"]] = item
        sessions = by_name
    if session_name not in sessions:
        known = ", ".join(sorted(sessions.keys()))
        raise KeyError(f"session '{session_name}' not found (known: {known})")
    return sessions[session_name]


def _parse_active_joints(raw: Optional[Sequence[int]]) -> List[int]:
    if not raw:
        return list(range(10))
    active = [int(x) for x in raw]
    if len(active) != len(set(active)):
        raise ValueError("allowed_joints contains duplicates")
    if not all(0 <= x < 10 for x in active):
        raise ValueError("allowed_joints must be indices 0..9")
    return active


def main(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description="Train CEM grasp policy.")
    parser.add_argument("--session", default=None, help="Session name from rl/train_sessions.json")
    parser.add_argument(
        "--gui",
        default=None,
        choices=["True", "False"],
        help="Enable GUI (True/False).",
    )
    args = parser.parse_args(argv)

    cfg = CemConfig()
    rng = np.random.default_rng(cfg.seed)

    session_name = args.session
    session_cfg = None
    if args.gui is not None:
        cfg.gui = args.gui == "True"
    if session_name:
        session_path = Path(__file__).with_name("train_sessions.json")
        session_cfg = _load_session_config(session_path, session_name)
        cfg.benchmark_part_id = session_cfg.get("benchmark_part_id", cfg.benchmark_part_id)
        cfg.benchmark_pos = tuple(session_cfg.get("benchmark_pos", cfg.benchmark_pos))
        cfg.benchmark_rpy = tuple(session_cfg.get("benchmark_rpy", cfg.benchmark_rpy))
        pre_grasp_joints = session_cfg.get("pre_grasp_joint_positions")
        if pre_grasp_joints is not None:
            cfg.pre_grasp_joint_positions = tuple(pre_grasp_joints)

    world = World(
        WorldConfig(
            gui=cfg.gui,
            robot_type="ur5",
            benchmark_part_id=cfg.benchmark_part_id,
            benchmark_pos=cfg.benchmark_pos,
            benchmark_rpy=cfg.benchmark_rpy,
            home_joint_positions=cfg.pre_grasp_joint_positions,
        )
    )
    env = PocEnv(world=world, config=PocEnvConfig())

    active_joints = _parse_active_joints(
        session_cfg.get("allowed_joints") if session_cfg else None
    )
    action_dim = len(active_joints)
    mean = np.full(action_dim, cfg.init_mean, dtype=float)
    std = np.full(action_dim, cfg.init_std, dtype=float)

    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "cem_log.jsonl"

    best_reward = float("-inf")
    best_q = None

    try:
        for it in range(1, cfg.iterations + 1):
            samples = rng.normal(loc=mean, scale=std, size=(cfg.population, action_dim))
            samples = clip01(samples)

            rewards = evaluate_population(env, samples, it, active_joints)
            order = np.argsort(rewards)[::-1]
            elites = samples[order[: cfg.elites]]
            elite_rewards = [rewards[i] for i in order[: cfg.elites]]

            mean = np.mean(elites, axis=0)
            std = np.maximum(np.std(elites, axis=0), cfg.min_std)

            if elite_rewards[0] > best_reward:
                best_reward = elite_rewards[0]
                best_q = expand_action(elites[0].tolist(), active_joints)

            log_row = {
                "iter": it,
                "session": session_name,
                "active_joints": active_joints,
                "best_reward": best_reward,
                "elite_reward_mean": float(np.mean(elite_rewards)),
                "elite_reward_max": float(np.max(elite_rewards)),
                "mean": mean.tolist(),
                "std": std.tolist(),
                "best_q": best_q,
            }
            with log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(log_row) + "\n")

            print(
                f"iter {it:02d} | best {best_reward:.3f} | "
                f"elite mean {log_row['elite_reward_mean']:.3f} | "
                f"elite max {log_row['elite_reward_max']:.3f}"
            )

            if cfg.sleep_s > 0:
                time.sleep(cfg.sleep_s)
    finally:
        world.close()

    shared_dir = Path("shared")
    shared_dir.mkdir(parents=True, exist_ok=True)
    summary_path = shared_dir / "cem_best.json"
    summary = build_grasp_record(best_q, session_name)
    summary["best_reward"] = best_reward
    summary["session"] = session_name
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("saved:", summary_path)


if __name__ == "__main__":
    main()
