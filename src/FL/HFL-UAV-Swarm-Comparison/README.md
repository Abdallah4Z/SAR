# HFL-UAV-Swarm-Comparison

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0+-red.svg)](https://pytorch.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac_Sim-GPU-green.svg)](https://developer.nvidia.com/isaac-sim)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.txt)

A comprehensive benchmark comparing **Federated Learning (FL)** algorithms combined with **Multi-Agent Reinforcement Learning (MARL)** for UAV swarm coordination and task offloading. This project evaluates 20 FL-MARL combinations across 5 swarm sizes (5-100 drones) using GPU-accelerated simulation with NVIDIA Isaac Sim/OmniDrones.

## 🎯 Overview

This project addresses the critical challenge of coordinating large-scale UAV swarms for mobile edge computing tasks. We systematically compare:

- **4 Federated Learning Algorithms**: FedAvg, FedProx, FedAdam, Hierarchical FL (HFL)
- **5 MARL Algorithms**: MAPPO, MADDPG, QMIX, PPO, SAC
- **5 Swarm Sizes**: 5, 10, 20, 50, 100 drones
- **Total Combinations**: 20 FL-MARL pairs × 5 scales = 100 experimental configurations

### Key Features

✅ **GPU-Accelerated Simulation**: Built on NVIDIA Isaac Sim and OmniDrones for efficient large-scale testing  
✅ **Realistic Communication Models**: Channel models with SINR, packet loss, and energy consumption  
✅ **Modular Architecture**: Easy to extend with new FL or MARL algorithms  
✅ **Comprehensive Metrics**: Task completion, energy efficiency, communication overhead, scalability  
✅ **Reproducible Results**: Seeded experiments with checkpointing and detailed logging  

---

## 📊 Research Questions

1. **Which FL-MARL combination achieves the best performance?**
2. **How do algorithms scale from 5 to 100 drones?**
3. **How do our results compare to state-of-the-art baselines from literature?**
4. **When should each combination be preferred (latency vs. energy vs. scalability)?**

---

## 🏗️ Project Structure

```
HFL-UAV-Swarm-Comparison/
├── configs/                      # Configuration files
│   ├── env_configs.py           # Environment parameters (5-100 drones)
│   ├── fl_configs.py            # FL algorithm configs
│   ├── marl_configs.py          # MARL algorithm configs
│   └── baseline_configs.py      # Literature baseline configs
│
├── isaac_env/                    # Isaac Sim GPU environment
│   ├── uav_task_offloading.py   # Main Isaac Sim environment
│   ├── task_manager_gpu.py      # Task generation (Poisson process)
│   ├── channel_model_gpu.py     # Wireless communication model
│   └── energy_model_gpu.py      # Energy consumption model
│
├── src/
│   ├── algorithms/              # MARL implementations
│   │   ├── base/                # Base agent classes
│   │   ├── mappo/               # Multi-Agent PPO
│   │   ├── maddpg/              # Multi-Agent DDPG
│   │   ├── qmix/                # QMIX (value factorization)
│   │   ├── ppo/                 # PPO (independent learning)
│   │   └── sac/                 # Soft Actor-Critic
│   │
│   ├── federated/               # FL implementations
│   │   ├── base_aggregator.py   # Base FL class
│   │   ├── fedavg.py            # Federated Averaging
│   │   ├── fedprox.py           # FedProx (proximal term)
│   │   ├── fedadam.py           # FedAdam (adaptive opt)
│   │   └── hierarchical_aggregator.py  # Hierarchical FL
│   │
│   ├── environments/            # CPU fallback environments
│   ├── evaluation/              # Evaluation metrics
│   ├── utils/                   # Helper utilities
│   └── visualization/           # Plotting tools
│
├── scripts/
│   ├── train/                   # Training scripts
│   │   ├── train_maddpg.py     # Train MADDPG with any FL
│   │   ├── train_mappo.py
│   │   ├── train_qmix.py
│   │   ├── train_ppo.py
│   │   └── train_sac.py
│   │
│   ├── evaluate/                # Evaluation scripts
│   ├── experiments/             # Batch experiment runners
│   └── setup/                   # Installation scripts
│
├── results/                     # Experimental results
│   ├── maddpg/                 # Per-algorithm results
│   ├── mappo/
│   ├── qmix/
│   ├── ppo/
│   ├── sac/
│   ├── comparisons/            # Cross-algorithm analysis
│   ├── final_selection/        # Top 3-5 combinations
│   └── literature_comparison/  # Baseline comparisons
│
├── models/                     # Saved model checkpoints
├── logs/                       # Training logs
├── tests/                      # Unit tests
└── docs/                       # Documentation
```

---

## 🚀 Quick Start

### Prerequisites

- NVIDIA GPU (RTX 3080 or better recommended)
- NVIDIA Isaac Sim 2023.1+ or OmniDrones
- Python 3.8+
- CUDA 11.8+
- 32GB+ RAM (for 100-drone simulations)

### Installation

1. **Clone the repository:**
   ```bash
   cd /home/skyvision/SAR/src/FL
   git clone <repository-url> HFL-UAV-Swarm-Comparison
   cd HFL-UAV-Swarm-Comparison
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Set up Isaac Sim environment** (if not already configured):
   ```bash
   # Follow OmniDrones installation guide
   # https://github.com/btx0424/OmniDrones
   ```

### Training Examples

**Train MADDPG with FedAvg (10 drones):**
```bash
python scripts/train/train_maddpg.py \
    --num_drones 10 \
    --fl_algorithm fedavg \
    --num_rounds 100 \
    --episodes_per_round 5 \
    --seed 42
```

**Train MAPPO with Hierarchical FL (20 drones):**
```bash
python scripts/train/train_mappo.py \
    --num_drones 20 \
    --fl_algorithm hierarchical \
    --num_rounds 100 \
    --seed 42
```

**Run all 12 MADDPG combinations:**
```bash
bash scripts/run_maddpg_12.sh
```

**Run all 36 combinations (4 FL × 3 MARL algorithms):**
```bash
bash scripts/run_all_36.sh
```

---

## 🧪 Algorithms

### Federated Learning

| Algorithm | Description | Key Parameter |
|-----------|-------------|---------------|
| **FedAvg** | Standard federated averaging | `client_fraction` |
| **FedProx** | Proximal term for heterogeneity | `mu=0.01` |
| **FedAdam** | Adaptive server-side optimization | `server_lr=0.01` |
| **Hierarchical FL** | Multi-level aggregation with clustering | `num_clusters` |

### Multi-Agent RL

| Algorithm | Type | Specialization |
|-----------|------|----------------|
| **MAPPO** | Actor-Critic | Centralized critic, decentralized actors |
| **MADDPG** | Actor-Critic | Multi-agent DDPG with experience replay |
| **QMIX** | Value-based | Value function factorization |
| **PPO** | Actor-Critic | Independent learners (baseline) |
| **SAC** | Actor-Critic | Maximum entropy RL |

---

## 📈 Results

Results are stored in `results/` with the following structure:

```
results/{algorithm}/{N}drones_{fl}algorithm_seed{S}/
├── training_metrics.json        # Episode rewards, loss curves
├── evaluation_results.json      # Final performance metrics
├── checkpoint_roundX.pth        # Model checkpoints
└── plots/                       # Reward curves, heatmaps
```

### Key Metrics

- **Task Completion Rate**: % of tasks successfully processed
- **Average Task Latency**: End-to-end processing time
- **Energy Efficiency**: Tasks/Joule per UAV
- **Communication Overhead**: Bytes transmitted per round
- **Scalability**: Performance degradation with swarm size

### Preliminary Findings

*(Based on results directory inspection)*

- ✅ MADDPG experiments completed for 4, 5, 10, 20, 50, 100 drones
- ✅ All 4 FL algorithms tested with MADDPG
- ✅ MAPPO, QMIX, PPO, SAC experiments in progress
- 📊 Statistical analysis and comparison pending

---

## 🧩 Environment Details

### Observation Space (Dynamic)

**Dimension**: `8 + 20 + 4×(N-1)` where N = number of drones

- **Self State (8)**: `[x, y, z, vx, vy, vz, battery, cpu_freq]`
- **Local Tasks (20)**: 5 tasks × `[data_size, cycles, deadline, priority]`
- **Neighbors (4×(N-1))**: Each neighbor's `[x, y, z, battery]`

**Examples:**
- 4 drones → 40 dimensions
- 10 drones → 64 dimensions
- 100 drones → 424 dimensions

### Action Space (Continuous)

**Dimension**: 5 continuous values in `[-1, 1]`

- `[target_x, target_y, target_z]`: Movement command
- `offload`: Offload decision (continuous, discretized in env)
- `cpu_freq`: CPU frequency scaling

### Reward Function

```python
reward = α × task_performance - β × energy_cost - γ × communication_cost
```

- **Task Performance**: Completed tasks, latency penalties
- **Energy Cost**: Battery depletion, hovering cost
- **Communication Cost**: Transmission overhead

---

## 🛠️ Extending the Framework

### Adding a New FL Algorithm

1. Create `src/federated/my_algorithm.py`:
   ```python
   from src.federated.base_aggregator import BaseAggregator
   
   class MyFLAlgorithm(BaseAggregator):
       def aggregate(self, local_models):
           # Your aggregation logic
           pass
   ```

2. Add config in `configs/fl_configs.py`:
   ```python
   FL_CONFIGS['my_fl'] = {
       'num_rounds': 100,
       'my_param': 0.5,
   }
   ```

3. Import in training scripts.

### Adding a New MARL Algorithm

1. Create `src/algorithms/my_marl/`:
   - `my_agent.py` (inherits from `BaseAgent`)
   - `my_network.py` (neural network architecture)
   - `my_trainer.py` (training loop)

2. Add config in `configs/marl_configs.py`

3. Create `scripts/train/train_my_marl.py`

---

## 🔬 Testing

Run unit tests:
```bash
pytest tests/
```

Specific test suites:
```bash
pytest tests/test_maddpg.py          # Test MADDPG implementation
pytest tests/test_federated.py       # Test FL aggregators
pytest tests/test_gpu_env.py         # Test Isaac Sim environment
pytest tests/test_communication.py   # Test channel models
```

---

## 📚 Citation

If you use this code in your research, please cite:

```bibtex
@software{hfl_uav_swarm_2026,
  title={HFL-UAV-Swarm-Comparison: Federated Learning for UAV Swarm Coordination},
  author={[Your Team]},
  year={2026},
  url={https://github.com/[your-repo]}
}
```

---

## 📄 License

This project is licensed under the MIT License - see [LICENSE.txt](LICENSE.txt) for details.

---

## 🙏 Acknowledgments

- **NVIDIA Isaac Sim / OmniDrones**: GPU-accelerated simulation framework
- **CleanRL**: Reference implementations for RL algorithms
- **Federated Learning Community**: FL algorithm designs and best practices

---

## 🐛 Troubleshooting

**Issue**: CUDA out of memory with 100 drones  
**Solution**: Reduce `num_envs` in config or use gradient accumulation

**Issue**: Isaac Sim crashes on startup  
**Solution**: Use `--headless` flag or verify GPU drivers

**Issue**: Training is too slow  
**Solution**: Enable `--cpu_env` for debugging, use GPU env for full experiments

**Issue**: Results not reproducible  
**Solution**: Ensure same seed, PyTorch deterministic mode, fixed environment config

---

## 📞 Contact

For questions or collaboration:
- Open an issue on GitHub
- Email: [your-email]

---

**Status**: ✅ Core experiments completed | 📊 Analysis in progress | 📝 Paper writing phase