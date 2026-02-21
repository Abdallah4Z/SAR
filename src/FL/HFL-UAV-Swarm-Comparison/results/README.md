# Results Directory

Managed by Ammar (FL-MARL Analysis Lead).

## Structure

```
results/
├── fl_marl_combinations/       # All FL x MARL combination results
│   ├── fedavg_mappo/           # FedAvg + MAPPO (Abdalla+Belal)
│   ├── fedavg_maddpg/
│   ├── fedavg_qmix/
│   ├── fedprox_mappo/          # FedProx + MAPPO (Jo+Belal)
│   ├── fedprox_maddpg/
│   ├── fedprox_qmix/
│   ├── fedadam_mappo/          # FedAdam + MAPPO (Ammar+Belal)
│   ├── fedadam_maddpg/
│   ├── fedadam_qmix/
│   ├── hierarchical_mappo/     # HFL + MAPPO (Belal)
│   ├── hierarchical_maddpg/
│   └── hierarchical_qmix/
├── literature_comparison/      # Comparison vs paper baselines
│   ├── baseline_results/       # Stored baseline evaluation results
│   ├── our_results/            # Best combo results for comparison
│   └── comparison_analysis/    # Statistical tests and tables
├── final_selection/            # Top 3-5 chosen combinations
│   ├── top_combinations/
│   └── recommendations.md
├── comparisons/
│   ├── scaling_analysis/
│   ├── algorithm_comparison/
│   └── statistical_tests/
├── plots/                       # All generated figures
└── README.md                    # This file
```

## File Format

Each experiment result is stored as a JSON file:

```
results/fl_marl_combinations/{fl}_{marl}/{N}drones_seed{S}.json
```

**Example:** `results/fl_marl_combinations/fedadam_mappo/10drones_seed2.json`

```json
{
  "fl": "fedadam",
  "marl": "mappo",
  "num_drones": 10,
  "seed": 2,
  "avg_episode_reward": 1.523,
  "std_episode_reward": 0.14,
  "avg_average_latency_ms": 42.3,
  "avg_task_success_rate": 0.87,
  "avg_energy_j": 312.5,
  "avg_fairness_cv": 0.12
}
```

## How to Store Results (for team members)

```python
from src.evaluation.combination_tester import CombinationTester

tester = CombinationTester(results_dir="results/fl_marl_combinations")

# After your training run completes:
tester.store_result(
    fl_name="fedadam",        # your FL algorithm
    marl_name="mappo",        # your MARL algorithm
    num_drones=10,              # swarm size you tested
    seed=0,                     # random seed
    metrics=evaluator_results   # dict from evaluator.evaluate()
)
```

## How to Load All Results for Analysis

```python
from src.evaluation.combination_tester import CombinationTester
from src.evaluation.comparator import Comparator
from src.evaluation.selection_framework import SelectionFramework

tester = CombinationTester(results_dir="results/fl_marl_combinations")
tester.load_all_results()   # Loads all JSON files from disk

comp = Comparator(tester)
top5 = comp.get_top_combinations(n=5)

sf = SelectionFramework(tester)
best = sf.select_top_combinations()
sf.print_selection_report(best)
```

## Experiment Matrix

| FL Algorithm | MARL Algorithm | Owner | Swarm Sizes | Seeds |
|---|---|---|---|---|
| FedAvg | MAPPO, MADDPG, QMIX | Abdalla | 5,10,20,50,100 | 0-4 |
| FedProx | MAPPO, MADDPG, QMIX | Jo | 5,10,20,50,100 | 0-4 |
| FedAdam | MAPPO, MADDPG, QMIX | Ammar | 5,10,20,50,100 | 0-4 |
| HFL | MAPPO, MADDPG, QMIX | Belal | 5,10,20,50,100 | 0-4 |

**Total:** 4 FL x 3 MARL x 5 sizes x 5 seeds = **300 runs**

## Metrics Tracked

| Metric | Description | Unit |
|---|---|---|
| avg_episode_reward | Mean reward per episode | (dimensionless) |
| avg_average_latency_ms | Mean task completion latency | milliseconds |
| avg_task_success_rate | Fraction of tasks completed before deadline | [0, 1] |
| avg_energy_j | Total energy consumption per episode | Joules |
| avg_fairness_cv | Coefficient of variation of task distribution | [0, +inf], lower is fairer |

## Analysis Notebooks / Scripts

- `scripts/experiments/run_fl_marl_combinations.py` — Run all 300 experiments
- `scripts/evaluate/evaluate_all.py` — Evaluate trained models
- `scripts/evaluate/compare_algorithms.py` — Generate comparison report

## Contact

**Ammar** — Analysis lead. Coordinate with team before modifying file naming conventions.
