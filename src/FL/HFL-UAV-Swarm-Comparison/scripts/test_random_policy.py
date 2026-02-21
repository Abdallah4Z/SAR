#!/usr/bin/env python3
"""
Validation script for the UAV Swarm Environment.

Runs a random policy for multiple episodes and checks that metrics
fall within expected ranges from the V4 prompt specification.

Usage:
    python scripts/test_random_policy.py
"""

import sys
import time
sys.path.insert(0, '.')

from src.environments.uav_swarm_env import UAVSwarmEnv
from configs.env_configs import CONFIGS
import numpy as np


def run_episode(env, seed, num_steps=2000):
    """Run a single episode with random policy."""
    obs, info = env.reset(seed=seed)

    total_reward = 0
    for step in range(num_steps):
        actions = {
            i: np.random.uniform(-2, 2, 3).astype(np.float32)
            for i in range(env.num_uavs)
        }
        obs, rewards, terminated, truncated, info = env.step(actions)
        total_reward += sum(rewards.values())

        if step % (num_steps // 10) == 0:
            print(f"  Step {step}: Alive={info['alive_uavs']}, "
                  f"Pending={info['pending_tasks']}, "
                  f"Collisions={info['collisions']}")

        if terminated or truncated:
            break

    return info['metrics'], total_reward, step + 1


def main():
    print("=" * 60)
    print("UAV Swarm Environment — Validation Script")
    print("=" * 60)

    config = CONFIGS['small']
    env = UAVSwarmEnv(config)

    print(f"\nConfig: {config['num_uavs']} UAVs, {config['num_ground_devices']} GDs")
    print(f"Timestep: {env.dt}s, Episode: {config['episode_length']} steps")
    print(f"Observation shape: {7 + 20 + 4 * (config['num_uavs'] - 1)} dims")
    print()

    # Run multiple episodes
    num_episodes = 3
    all_metrics = []

    for ep in range(num_episodes):
        print(f"--- Episode {ep + 1}/{num_episodes} (seed={42 + ep}) ---")
        t0 = time.time()
        metrics, total_reward, steps = run_episode(env, seed=42 + ep, num_steps=2000)
        elapsed = time.time() - t0

        all_metrics.append(metrics)
        fps = steps / elapsed

        print(f"  Steps: {steps}, Time: {elapsed:.2f}s, FPS: {fps:.0f}")
        print(f"  Success Rate: {metrics['task_success_rate']:.1f}%")
        print(f"  Avg Latency:  {metrics['average_latency_ms']:.1f} ms")
        print(f"  Energy:       {metrics['total_energy_j']:.1f} J")
        print(f"  Fairness CV:  {metrics['fairness_cv']:.3f}")
        print(f"  Completed:    {metrics['tasks_completed']}, Failed: {metrics['tasks_failed']}")
        print(f"  Total Reward:  {total_reward:.1f}")
        print()

    # Summary
    print("=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)

    avg_success = np.mean([m['task_success_rate'] for m in all_metrics])
    avg_latency = np.mean([m['average_latency_ms'] for m in all_metrics])
    avg_fps = 2000 / elapsed  # Approximate from last episode

    checks = []

    # Check 1: Success rate
    if avg_success >= 10 and avg_success <= 70:
        print(f"✅ Success Rate: {avg_success:.1f}% (target: 10-70%)")
        checks.append(True)
    else:
        print(f"❌ Success Rate: {avg_success:.1f}% (target: 10-70%)")
        checks.append(False)

    # Check 2: Latency positive
    if avg_latency > 0:
        print(f"✅ Avg Latency:  {avg_latency:.1f} ms (positive)")
        checks.append(True)
    else:
        print(f"❌ Avg Latency:  {avg_latency:.1f} ms (must be positive)")
        checks.append(False)

    # Check 3: FPS
    if avg_fps > 500:
        print(f"✅ Performance:  {avg_fps:.0f} steps/sec (target: >500)")
        checks.append(True)
    else:
        print(f"❌ Performance:  {avg_fps:.0f} steps/sec (target: >500)")
        checks.append(False)

    # Check 4: Episodes complete without crash
    print(f"✅ Stability:    {num_episodes}/{num_episodes} episodes completed")
    checks.append(True)

    print()
    if all(checks):
        print("✅ ALL VALIDATION CHECKS PASSED")
    else:
        print("⚠️ SOME CHECKS NEED ATTENTION (see above)")

    return all(checks)


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
