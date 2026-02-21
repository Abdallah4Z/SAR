# HFL Multi-Algorithm Comparison Project
## Team Structure & Task Assignment for 5 Members

---

## PROJECT OVERVIEW

This project implements a comprehensive comparison of Federated Learning (FL) algorithms combined with Multi-Agent Reinforcement Learning (MARL) for UAV swarm coordination. The team will test multiple FL algorithms (Hierarchical FL, FedAvg, FedProx, FedAdam) with five different MARL algorithms (MAPPO, MADDPG, QMIX, PPO, SAC) across 5 swarm sizes (5, 10, 20, 50, 100 drones) to determine the optimal FL-MARL combination.

**Extended Research Questions:**
1. Which combination of FL algorithm and MARL algorithm achieves the best performance for UAV swarm task offloading?
2. How does algorithm performance scale across different swarm sizes (5 to 100 drones)?
3. How do our best-performing FL-MARL combinations compare against state-of-the-art approaches from literature?

**Research Approach:**
- **Phase 1:** Implement and test different FL algorithms (HFL, FedAvg, FedProx, FedAdam)
- **Phase 2:** Combine each FL algorithm with each MARL algorithm (5 MARL × 4 FL = 20 combinations)
- **Phase 3:** Test all combinations across 5 swarm sizes
- **Phase 4:** Select the top 3-5 best-performing combinations
- **Phase 5:** Compare best combinations against baseline methods from literature review
- **Phase 6:** Provide recommendations on when to use each FL-MARL combination

---

## TEAM MEMBER ASSIGNMENTS

### **Member 1: Environment & Infrastructure Lead**
**Primary Responsibility:** Build and maintain the UAV swarm simulation environment and shared infrastructure

**Your Folders:**
- `src/environments/` - All environment-related code
- `src/communication/` - Communication simulation
- `configs/` - Configuration files for all experiments
- `scripts/setup/` - Installation and setup scripts

**Your Tasks:**
- Create the UAV swarm Gymnasium environment with PyBullet physics
- Implement communication channel models (SINR, packet loss, energy)
- Build the task generation and management system
- **Create Python-based configuration dictionaries for different swarm sizes (5, 10, 20, 50, 100)**
- Set up the project infrastructure (requirements.txt, Docker if needed)
- Create data logging and metrics collection framework
- Build visualization tools for environment state

**Files You Own:**
```
src/environments/uav_swarm_env.py          # Main environment
src/environments/drone_dynamics.py         # Physics and movement
src/environments/task_manager.py           # Task generation and tracking
src/communication/channel_model.py         # Wireless channel simulation
src/communication/energy_model.py          # Energy consumption
configs/env_configs.py                     # All environment configs (Python dict)
scripts/setup/install_dependencies.sh      # Setup script
```

**Integration Points:**
- Provide environment API to Members 2-5 for their RL algorithms
- Work with Member 2 to define observation/action spaces
- Coordinate with all members on reward function design

---

### **Member 2: MAPPO Implementation & FL Framework Architect**

**Primary Responsibility:** Implement MAPPO algorithm and establish the modular federated learning architecture

**Your Folders:**

* `src/algorithms/mappo/` - MAPPO implementation
* `src/federated/` - Shared FL infrastructure (Base classes + HFL)
* `src/algorithms/base/` - Base classes for all algorithms

**Your Tasks:**

* Implement MAPPO actor-critic networks (worker and coordinator)
* Build the Proximal Policy Optimization training loop with GAE
* Create the **base RL agent class** that all other members must inherit
* Build the **modular FL framework** (base aggregator) so any MARL can use any FL algorithm
* **Implement Hierarchical FL (HFL):** Multi-level aggregation with dynamic clustering
* Create dynamic clustering mechanism with RL-based decisions
* Implement coordinator trajectory optimization
* Establish the training pipeline and standard communication protocols for FL updates
* Manage shared configuration files for all members

**Files You Own:**

```
src/algorithms/base/base_agent.py          # Base class for all RL agents
src/algorithms/base/actor_critic.py        # Base actor-critic architecture
src/algorithms/mappo/mappo_agent.py        # MAPPO agent implementation
src/algorithms/mappo/mappo_network.py      # MAPPO neural networks
src/algorithms/mappo/mappo_trainer.py      # MAPPO training loop
src/federated/base_aggregator.py           # Base FL aggregator class
src/federated/hierarchical_aggregator.py   # Hierarchical FL (HFL) implementation
src/federated/clustering.py                # Dynamic clustering logic
src/federated/coordinator_policy.py        # Coordinator RL policy
configs/fl_configs.py                      # FL algorithm configs (Python dict)
configs/marl_configs.py                    # MARL configs (Python dict)

```

**Integration Points:**

* Define the `BaseAggregator` interface that Members 3, 4, and 5 will implement
* Provide the HFL implementation as the primary communication backbone
* Work with Member 1 on observation/action space design
* Coordinate with Member 5 on evaluation standards

---

### **Member 3: MADDPG & FedAvg Implementation Lead**

**Primary Responsibility:** Implement MADDPG algorithm and the standard FedAvg framework

**Your Folders:**

* `src/algorithms/maddpg/` - MADDPG implementation
* `src/federated/fedavg.py` - FedAvg implementation

**Your Tasks:**

* Implement MADDPG actor-critic networks with centralized critic
* Build the off-policy training loop with experience replay
* **Implement FedAvg:** Standard federated averaging algorithm within the Member 2 framework
* Adapt MADDPG to work interchangeably with FedAvg and HFL
* Implement target networks and soft updates
* Create replay buffers for experience storage
* Tune hyperparameters specific to MADDPG and FedAvg
* Run experiments across all 5 swarm sizes for this combination
* Compare MADDPG performance against MAPPO baseline

**Files You Own:**

```
src/algorithms/maddpg/maddpg_agent.py      # MADDPG agent (extends base_agent)
src/algorithms/maddpg/maddpg_network.py    # Actor and centralized critic
src/algorithms/maddpg/maddpg_trainer.py    # MADDPG training loop
src/algorithms/maddpg/replay_buffer.py     # Experience replay
src/algorithms/maddpg/target_network.py    # Target network management
src/federated/fedavg.py                    # Standard FedAvg implementation

```

**Note:** MADDPG hyperparameters are defined in `configs/marl_configs.py` (managed by Member 2)

**Integration Points:**

* Inherit from base_agent.py created by Member 2
* Register `FedAvg` into the modular framework established by Member 2
* Share environment interface with Member 1
* Coordinate with Member 5 on metrics and evaluation

---

### **Member 4: QMIX & FedProx Implementation Lead**

**Primary Responsibility:** Implement QMIX algorithm and the FedProx algorithm

**Your Folders:**

* `src/algorithms/qmix/` - QMIX implementation
* `src/federated/fedprox.py` - FedProx implementation

**Your Tasks:**

* Implement QMIX with value decomposition and mixing network
* Handle discrete action space conversion for QMIX
* **Implement FedProx:** FedAvg with a proximal term to handle system heterogeneity
* Adapt QMIX to work with FedProx and HFL
* Build Q-networks and mixing network for QMIX
* Run experiments for QMIX across all swarm sizes
* Evaluate QMIX performance under FedProx constraints

**Files You Own:**

```
src/algorithms/qmix/qmix_agent.py          # QMIX agent
src/algorithms/qmix/qmix_network.py        # Q-networks and mixer
src/algorithms/qmix/qmix_trainer.py        # QMIX training loop
src/algorithms/qmix/action_discretizer.py  # Convert continuous to discrete
src/federated/fedprox.py                   # FedProx implementation

```

**Note:** QMIX hyperparameters are defined in `configs/marl_configs.py` (managed by Member 2)

**Integration Points:**

* Use base classes and aggregator interface from Member 2
* Integrate with federated learning framework
* Work with Member 1 on action space discretization for QMIX
* Share results with Member 5 for analysis

---

### **Member 5: FedAdam Implementation & Comprehensive Analysis Lead**

**Primary Responsibility:** Implement FedAdam algorithm, lead FL-MARL combination analysis, and benchmark against literature

**Your Folders:**

* `src/federated/fedadam.py` - FedAdam implementation
* `src/evaluation/` - Evaluation and comparison framework
* `src/visualization/` - Plotting and visualization
* `src/baselines/` - Literature baseline implementations
* `results/` - Experimental results storage

**Your Tasks:**

* **Implement FedAdam:** Adaptive federated optimization with momentum
* Ensure FedAdam is compatible with all MARL algorithms (MAPPO, MADDPG, QMIX)
* **Create FL-MARL combination testing framework** (evaluating all 4 FL algorithms against the 3 MARL variants)
* **Implement or adapt baseline methods from literature review:**
* Select 3-5 key papers for comparison
* Implement their methods or adapt results
* Ensure fair comparison settings


* Create comprehensive evaluation framework for all combinations
* Implement all metrics trackers (latency, energy, throughput, fairness, convergence)
* **Build selection framework to identify top-performing FL-MARL combinations**
* **Create literature comparison analysis:**
* Statistical significance tests vs baselines
* Performance gap analysis
* Computational cost comparison


* Build comparison dashboard and visualization tools
* Run comparative experiments across all algorithms and scales
* Generate plots, tables, and statistical analysis
* Write the results analysis and comparison report

**Files You Own:**

```
src/federated/fedadam.py                   # FedAdam implementation
src/baselines/literature_baselines.py      # Implementations from papers
src/baselines/baseline_adapters.py         # Adapt literature methods
src/evaluation/metrics_tracker.py          # All metrics implementation
src/evaluation/evaluator.py                # Evaluation framework
src/evaluation/combination_tester.py       # FL-MARL combination testing
src/evaluation/comparator.py               # Algorithm comparison
src/evaluation/literature_comparison.py    # Compare against papers
src/evaluation/selection_framework.py      # Select best combinations
src/visualization/plot_training.py         # Training curves
src/visualization/plot_scaling.py          # Scaling analysis
src/visualization/plot_comparison.py       # Algorithm comparison plots
src/visualization/plot_fl_marl_matrix.py   # FL-MARL combination heatmaps
src/visualization/plot_literature.py       # Literature comparison plots
src/visualization/dashboard.py             # Real-time dashboard
configs/baseline_configs.py                # Literature baseline configs (Python dict)
results/README.md                          # Results documentation
results/literature_comparison/             # Comparison with papers

```

**Note:** Configuration management is shared with Member 2, but analysis logic is yours.

**Integration Points:**

* Integrate the `FedAdam` aggregator into Member 2’s framework
* Collect results/logs from Members 2, 3, and 4 for final analysis
* Work with Member 1 on metrics logging integration
* Coordinate final experiments and paper writing with all members


## COMPLETE PROJECT FOLDER STRUCTURE

```
HFL-UAV-Swarm-Comparison/
│
├── README.md                              # Project overview (ALL - collaborative)
├── requirements.txt                       # Python dependencies (Member 1)
├── setup.py                               # Package installation (Member 1)
├── .gitignore                            # Git ignore file (Member 1)
├── Dockerfile                            # Docker container (Member 1 - optional)
├── docker-compose.yml                    # Multi-container setup (Member 1 - optional)
│
├── configs/                              # Configuration files (Python-based)
│   ├── __init__.py                      # Make configs a Python package
│   ├── env_configs.py                   # All environment configs (Member 1)
│   ├── fl_configs.py                    # FL algorithm configs (Member 2)
│   ├── marl_configs.py                  # MARL algorithm configs (Member 2)
│   └── baseline_configs.py              # Literature baseline configs (Member 5)
│
├── src/                                  # Source code
│   │
│   ├── environments/                     # Environment implementation (Member 1)
│   │   ├── __init__.py
│   │   ├── uav_swarm_env.py             # Main Gymnasium environment
│   │   ├── drone_dynamics.py            # UAV physics and movement
│   │   ├── task_manager.py              # Task generation and scheduling
│   │   ├── reward_calculator.py         # Reward function
│   │   └── state_observer.py            # State observation logic
│   │
│   ├── communication/                    # Communication models (Member 1)
│   │   ├── __init__.py
│   │   ├── channel_model.py             # Wireless channel (SINR, path loss)
│   │   ├── packet_manager.py            # Packet transmission/loss
│   │   └── energy_model.py              # Energy consumption tracking
│   │
│   ├── algorithms/                       # RL algorithms
│   │   │
│   │   ├── base/                        # Base classes (Member 2)
│   │   │   ├── __init__.py
│   │   │   ├── base_agent.py           # Base RL agent interface
│   │   │   ├── actor_critic.py         # Base actor-critic architecture
│   │   │   └── buffer.py               # Base experience buffer
│   │   │
│   │   ├── mappo/                       # MAPPO implementation (Member 2)
│   │   │   ├── __init__.py
│   │   │   ├── mappo_agent.py          # MAPPO agent
│   │   │   ├── mappo_network.py        # Actor-critic networks
│   │   │   ├── mappo_trainer.py        # Training loop with GAE
│   │   │   └── gae.py                  # Generalized Advantage Estimation
│   │   │
│   │   ├── maddpg/                      # MADDPG implementation (Member 3)
│   │   │   ├── __init__.py
│   │   │   ├── maddpg_agent.py         # MADDPG agent
│   │   │   ├── maddpg_network.py       # Actor + centralized critic
│   │   │   ├── maddpg_trainer.py       # Off-policy training
│   │   │   ├── replay_buffer.py        # Experience replay
│   │   │   └── target_network.py       # Target network soft updates
│   │   │
│   │   ├── qmix/                        # QMIX implementation (Member 4)
│   │   │   ├── __init__.py
│   │   │   ├── qmix_agent.py           # QMIX agent
│   │   │   ├── qmix_network.py         # Q-networks and mixing network
│   │   │   ├── qmix_trainer.py         # Value decomposition training
│   │   │   └── action_discretizer.py   # Continuous to discrete actions
│   │   │
│   │   ├── ppo/                         # Independent PPO (Member 4)
│   │   │   ├── __init__.py
│   │   │   ├── ppo_agent.py            # Independent PPO agent
│   │   │   ├── ppo_network.py          # Actor-critic for single agent
│   │   │   └── ppo_trainer.py          # PPO training without coordination
│   │   │
│   │   └── sac/                         # SAC implementation (Member 5)
│   │       ├── __init__.py
│   │       ├── sac_agent.py            # SAC agent
│   │       ├── sac_network.py          # Actor, dual critics, alpha
│   │       ├── sac_trainer.py          # SAC training with entropy
│   │       └── replay_buffer.py        # Off-policy replay
│   │
│   ├── federated/                        # Federated Learning (Member 2)
│   │   ├── __init__.py
│   │   ├── base_aggregator.py           # Base FL aggregator class
│   │   ├── fedavg.py                    # Standard FedAvg algorithm
│   │   ├── fedprox.py                   # FedProx with proximal term
│   │   ├── fedadam.py                   # FedAdam adaptive optimization
│   │   ├── hierarchical_aggregator.py   # Hierarchical FL (HFL)
│   │   ├── clustering.py                # Dynamic clustering strategies
│   │   ├── coordinator_policy.py        # Coordinator RL for clustering
│   │   └── model_broadcaster.py         # Global model distribution
│   │
│   ├── baselines/                        # Literature baselines (Member 5)
│   │   ├── __init__.py
│   │   ├── literature_baselines.py      # Implementations from papers
│   │   └── baseline_adapters.py         # Adapters for fair comparison
│   │
│   ├── evaluation/                       # Evaluation framework (Member 5)
│   │   ├── __init__.py
│   │   ├── metrics_tracker.py           # Latency, energy, throughput, etc.
│   │   ├── evaluator.py                 # Run evaluation episodes
│   │   ├── combination_tester.py        # Test FL-MARL combinations
│   │   ├── comparator.py                # Compare algorithms
│   │   ├── literature_comparison.py     # Compare with papers
│   │   ├── selection_framework.py       # Select best combinations
│   │   └── statistical_tests.py         # T-tests, confidence intervals
│   │
│   ├── visualization/                    # Visualization (Member 5)
│   │   ├── __init__.py
│   │   ├── plot_training.py             # Training curves (reward, loss)
│   │   ├── plot_scaling.py              # Scaling analysis (5→100 drones)
│   │   ├── plot_comparison.py           # Algorithm comparison
│   │   ├── plot_fl_marl_matrix.py       # FL-MARL combination heatmaps
│   │   ├── plot_literature.py           # Literature comparison plots
│   │   ├── plot_metrics.py              # Metrics visualization
│   │   └── dashboard.py                 # Real-time monitoring dashboard
│   │
│   └── utils/                            # Shared utilities (ALL)
│       ├── __init__.py
│       ├── logger.py                    # Logging utilities
│       ├── checkpoint.py                # Model saving/loading
│       ├── seed.py                      # Random seed management
│       └── timer.py                     # Timing utilities
│
├── scripts/                              # Executable scripts
│   │
│   ├── setup/                           # Setup scripts (Member 1)
│   │   ├── install_dependencies.sh      # Install all requirements
│   │   ├── setup_wandb.sh              # Configure Weights & Biases
│   │   └── download_datasets.sh        # Download any needed data
│   │
│   ├── train/                           # Training scripts (Each member for their algorithm)
│   │   ├── train_mappo.py              # Train MAPPO (Member 2)
│   │   ├── train_maddpg.py             # Train MADDPG (Member 3)
│   │   ├── train_qmix.py               # Train QMIX (Member 4)
│   │   ├── train_ppo.py                # Train Independent PPO (Member 4)
│   │   └── train_sac.py                # Train SAC (Member 5)
│   │
│   ├── evaluate/                        # Evaluation scripts (Member 5)
│   │   ├── evaluate_single.py          # Evaluate one algorithm
│   │   ├── evaluate_all.py             # Evaluate all algorithms
│   │   └── compare_algorithms.py       # Generate comparison results
│   │
│   └── experiments/                     # Experiment runners (Member 5)
│       ├── run_scaling_experiment.py   # Run all swarm sizes
│       ├── run_fl_marl_combinations.py # Run all 20 FL-MARL combinations
│       ├── run_best_combinations.py    # Run top performers
│       ├── run_literature_comparison.py # Compare with baselines
│       ├── run_full_comparison.py      # Run all algorithms
│       └── sweep_hyperparameters.py    # Hyperparameter search
│
├── tests/                                # Unit tests (Each member for their code)
│   ├── test_environment.py              # Test environment (Member 1)
│   ├── test_communication.py            # Test communication (Member 1)
│   ├── test_mappo.py                    # Test MAPPO (Member 2)
│   ├── test_federated.py                # Test FL framework (Member 2)
│   ├── test_maddpg.py                   # Test MADDPG (Member 3)
│   ├── test_qmix.py                     # Test QMIX (Member 4)
│   ├── test_ppo.py                      # Test PPO (Member 4)
│   ├── test_sac.py                      # Test SAC (Member 5)
│   └── test_metrics.py                  # Test metrics (Member 5)
│
├── results/                              # Experimental results (Member 5 manages)
│   ├── README.md                        # Results documentation
│   ├── fl_marl_combinations/            # Results for all 20 FL-MARL combos
│   │   ├── combination_matrix.csv       # Performance matrix
│   │   ├── fedavg_mappo/                # FedAvg + MAPPO results
│   │   ├── fedavg_maddpg/               # FedAvg + MADDPG results
│   │   ├── fedprox_mappo/               # FedProx + MAPPO results
│   │   └── ...                          # All 20 combinations
│   ├── literature_comparison/           # Comparison with papers
│   │   ├── baseline_results/            # Results from literature
│   │   ├── our_results/                 # Our results on same settings
│   │   └── comparison_analysis/         # Statistical comparison
│   ├── final_selection/                 # Best combinations analysis
│   │   ├── top_combinations/            # Top 3-5 performers
│   │   ├── performance_profiles/        # When each combo excels
│   │   └── recommendations.md           # Usage recommendations
│   ├── mappo/                           # MAPPO results (legacy structure)
│   │   ├── 5_drones/
│   │   ├── 10_drones/
│   │   ├── 20_drones/
│   │   ├── 50_drones/
│   │   └── 100_drones/
│   ├── maddpg/                          # MADDPG results
│   │   └── [same structure]
│   ├── qmix/                            # QMIX results
│   │   └── [same structure]
│   ├── ppo/                             # PPO results
│   │   └── [same structure]
│   ├── sac/                             # SAC results
│   │   └── [same structure]
│   ├── comparisons/                     # Comparison plots and tables
│   │   ├── scaling_analysis/
│   │   ├── algorithm_comparison/
│   │   ├── fl_algorithm_comparison/     # Compare FL algorithms
│   │   └── statistical_tests/
│   └── final_report/                    # Final analysis report
│       ├── figures/
│       ├── tables/
│       └── report.pdf
│
├── models/                               # Saved model checkpoints
│   ├── mappo/                           # MAPPO checkpoints (Member 2)
│   ├── maddpg/                          # MADDPG checkpoints (Member 3)
│   ├── qmix/                            # QMIX checkpoints (Member 4)
│   ├── ppo/                             # PPO checkpoints (Member 4)
│   └── sac/                             # SAC checkpoints (Member 5)
│
├── logs/                                 # Training logs (AUTO-GENERATED)
│   ├── tensorboard/                     # TensorBoard logs
│   ├── wandb/                           # Weights & Biases logs
│   └── txt/                             # Text logs
│
├── docs/                                 # Documentation (ALL contribute)
│   ├── project_description.md           # Overall project description
│   ├── environment_api.md               # Environment documentation (Member 1)
│   ├── algorithm_comparison.md          # Algorithm comparison guide
│   ├── metrics_guide.md                 # Metrics documentation (Member 5)
│   ├── meeting_notes/                   # Team meeting notes
│   └── paper_draft/                     # Research paper draft
│       ├── sections/
│       ├── figures/
│       └── main.tex
│
└── .github/                              # GitHub specific (Member 1)
    └── workflows/
        ├── tests.yml                    # Automated testing
        └── linting.yml                  # Code quality checks
```

---

## PYTHON-BASED CONFIGURATION APPROACH

**Why Python configs instead of YAML?**
- ✅ No YAML parsing libraries needed
- ✅ Direct Python imports - no file loading overhead
- ✅ Better IDE support (autocomplete, type checking)
- ✅ Can compute values programmatically
- ✅ Easier debugging and validation
- ✅ Version control friendly

### **Configuration File Structure:**

#### **configs/env_configs.py** (Member 1 owns)
```python
"""Environment configurations for different swarm sizes"""

ENV_CONFIGS = {
    5: {
        'num_drones': 5,
        'area_size': (100, 100, 50),  # (x, y, z) in meters
        'num_tasks': 20,
        'communication_range': 50,
        'max_speed': 10,
        'battery_capacity': 1000,
        # ... other environment params
    },
    10: {'num_drones': 10, 'area_size': (150, 150, 50), ...},
    20: {'num_drones': 20, 'area_size': (200, 200, 50), ...},
    50: {'num_drones': 50, 'area_size': (300, 300, 50), ...},
    100: {'num_drones': 100, 'area_size': (400, 400, 50), ...}
}

def get_env_config(num_drones):
    """Get environment config for specific swarm size"""
    return ENV_CONFIGS[num_drones]
```

#### **configs/fl_configs.py** (Member 2 owns)
```python
"""Federated Learning algorithm configurations"""

FL_CONFIGS = {
    'fedavg': {
        'num_rounds': 100,
        'local_epochs': 5,
        'batch_size': 32,
        'learning_rate': 0.001,
    },
    'fedprox': {
        'num_rounds': 100,
        'local_epochs': 5,
        'mu': 0.01,  # Proximal term
    },
    'fedadam': {
        'num_rounds': 100,
        'server_learning_rate': 0.01,
        'beta1': 0.9,
        'beta2': 0.99,
    },
    'hierarchical': {
        'num_rounds': 100,
        'num_clusters': 'auto',
        'intra_cluster_rounds': 3,
    }
}

def get_fl_config(fl_algorithm):
    return FL_CONFIGS[fl_algorithm]
```

#### **configs/marl_configs.py** (Member 2 owns)
```python
"""MARL algorithm configurations"""

MARL_CONFIGS = {
    'mappo': {
        'hidden_dim': 256,
        'clip_param': 0.2,
        'gamma': 0.99,
        # ... MAPPO params
    },
    'maddpg': {
        'actor_lr': 0.001,
        'buffer_size': 100000,
        'tau': 0.005,
        # ... MADDPG params
    },
    'qmix': {...},
    'ppo': {...},
    'sac': {...}
}

def get_marl_config(marl_algorithm):
    return MARL_CONFIGS[marl_algorithm]
```

#### **configs/baseline_configs.py** (Member 5 owns)
```python
"""Literature baseline configurations"""

BASELINE_CONFIGS = {
    'baseline_paper1': {...},
    'baseline_paper2': {...},
}

def get_baseline_config(baseline_name):
    return BASELINE_CONFIGS[baseline_name]
```

### **How to Use Configs:**

```python
# In any training script
from configs.env_configs import get_env_config
from configs.fl_configs import get_fl_config
from configs.marl_configs import get_marl_config

# Get configurations
env_config = get_env_config(num_drones=10)
fl_config = get_fl_config('fedavg')
marl_config = get_marl_config('mappo')

# Use them
env = UAVSwarmEnv(**env_config)
agent = MAPPOAgent(**marl_config)
aggregator = FedAvg(**fl_config)
```

### **Running Experiments:**

```python
# In scripts/experiments/run_fl_marl_combinations.py
from configs.env_configs import ENV_CONFIGS
from configs.fl_configs import FL_CONFIGS
from configs.marl_configs import MARL_CONFIGS

# Test all 20 FL-MARL combinations
for fl_name in FL_CONFIGS.keys():
    for marl_name in MARL_CONFIGS.keys():
        for num_drones in ENV_CONFIGS.keys():
            run_experiment(fl_name, marl_name, num_drones)
```

---

## DETAILED TASK BREAKDOWN BY PHASE

### **PHASE 1: Foundation (Weeks 1-2)**

**Member 1 Tasks:**
- Set up project repository and folder structure
- Create requirements.txt with all dependencies
- Implement basic UAV swarm environment with Gymnasium API
- **Create Python configuration dictionaries for all 5 swarm sizes in configs/env_configs.py**
- Build communication channel simulation (SINR, packet loss)
- Implement energy consumption model
- Create task generation system
- **Deliverable:** Working environment that other members can import

**Member 2 Tasks:**
- Create base agent class that all algorithms will inherit
- Implement base actor-critic architecture
- Build MAPPO networks (worker and coordinator)
- Implement PPO training loop with GAE
- Create federated averaging algorithm
- Build hierarchical aggregation system
- Implement basic clustering mechanism
- **Create Python configs for FL algorithms (configs/fl_configs.py) and MARL algorithms (configs/marl_configs.py)**
- **Deliverable:** Working MAPPO + HFL baseline that others can adapt, plus complete config system

**Members 3, 4, 5:**
- Study the environment API from Member 1
- Review base classes from Member 2
- Research your assigned algorithms (papers, existing implementations)
- Prepare algorithm design documents
- Set up development environments
- **Deliverable:** Algorithm design specifications

---

### **PHASE 2: Algorithm Implementation (Weeks 3-6)**

**Member 1 Tasks:**
- Enhance environment with more realistic physics
- Add visualization tools for debugging
- Implement metrics logging framework
- Support Members 2-5 with environment-related issues
- Create automated testing for environment
- **Deliverable:** Robust, well-tested environment

**Member 2 Tasks:**
- Complete MAPPO implementation with all features
- Implement coordinator trajectory optimization
- Add dynamic clustering with RL-driven decisions
- Run initial experiments on 5 and 10 drones
- Debug and optimize MAPPO performance
- **Deliverable:** Fully working MAPPO baseline with results

**Member 3 Tasks:**
- Implement MADDPG actor and centralized critic networks
- Build off-policy training loop
- Create experience replay buffer
- Implement target networks with soft updates
- Adapt MADDPG to work with HFL framework from Member 2
- Run initial tests on 5 drones
- **Deliverable:** Working MADDPG integrated with HFL

**Member 4 Tasks:**
- Implement QMIX Q-networks and mixing network
- Create action discretization for QMIX
- Build QMIX training loop with value decomposition
- Implement Independent PPO (simpler than MAPPO)
- Test both algorithms on 5 drones
- **Deliverable:** Working QMIX and Independent PPO

**Member 5 Tasks:**
- Implement SAC with automatic entropy tuning
- Build dual Q-networks and actor
- Create SAC training loop
- Implement comprehensive metrics tracker
- Build evaluation framework
- Create initial visualization tools
- **Deliverable:** Working SAC and evaluation framework

---

### **PHASE 3: FL-MARL Combination & Scaling Experiments (Weeks 7-10)**

**ALL MEMBERS:**
- Test your assigned MARL algorithm(s) with ALL 4 FL algorithms
- Run experiments on ALL swarm sizes: 5, 10, 20, 50, 100
- Each MARL × 4 FL algorithms × 5 swarm sizes × 5 random seeds
  - Member 2 (MAPPO): 1 MARL × 4 FL × 5 sizes × 5 seeds = 100 runs
  - Member 3 (MADDPG): 1 MARL × 4 FL × 5 sizes × 5 seeds = 100 runs
  - Member 4 (QMIX + PPO): 2 MARL × 4 FL × 5 sizes × 5 seeds = 200 runs
  - Member 5 (SAC): 1 MARL × 4 FL × 5 sizes × 5 seeds = 100 runs
  - **Total: 500 experimental runs across the team**
- Log all metrics during training
- Save model checkpoints for best combinations
- Document any issues or failures

**Member 1:**
- Monitor all experiments for environment bugs
- Fix any scaling issues that emerge
- Ensure consistent logging across all experiments
- Provide computational resource support

**Member 2:**
- Run MAPPO with all 4 FL algorithms across all scales
- Ensure all FL algorithms work correctly
- Tune FL-specific hyperparameters if needed
- Share baseline FL algorithm performance

**Member 3:**
- Run MADDPG with all 4 FL algorithms
- Compare FL algorithm effectiveness for off-policy learning
- Document FL-MADDPG integration challenges

**Member 4:**
- Run QMIX and PPO with all 4 FL algorithms
- Compare how FL algorithms work with value decomposition
- Analyze differences in FL performance for on-policy vs value-based
- Identify when each FL algorithm works better

**Member 5:**
- Run SAC with all 4 FL algorithms
- Coordinate all experimental runs across the team
- Monitor resource usage (GPU, memory)
- Begin initial combination analysis as results come in
- Create preliminary FL-MARL performance matrix

---

### **PHASE 4: Combination Selection & Literature Comparison (Weeks 11-13)**

**Member 5 (LEAD):**
- **FL-MARL Combination Analysis:**
  - Collect all 500+ experimental results from team members
  - Create comprehensive FL-MARL performance matrix (4 FL × 5 MARL = 20 combinations)
  - Generate heatmaps showing best combinations for each metric
  - Run statistical significance tests across combinations
  - Identify top 3-5 best-performing FL-MARL combinations
  - Analyze when each combination excels (swarm size, objectives, constraints)

- **Literature Comparison:**
  - Implement or adapt 3-5 baseline methods from literature review
  - Run baselines on same experimental settings (fair comparison)
  - Compare top combinations against state-of-the-art methods
  - Statistical significance testing vs literature baselines
  - Performance gap analysis (how much improvement?)
  - Computational cost comparison
  - Create literature comparison tables and plots

- **Final Analysis:**
  - Build comprehensive comparison dashboard
  - Create scaling analysis charts (5→100 drones)
  - Generate all comparison plots and tables
  - Write results analysis section
  - Develop usage recommendations for each combination

**ALL MEMBERS:**
- Contribute results and observations
- Review FL-MARL combination analysis from Member 5
- Provide insights on algorithm-specific behaviors
- Help interpret unexpected results
- Review literature comparison methodology

**Member 1:**
- Create environment behavior visualizations
- Document any environment-related findings
- Support reproducibility documentation

**Members 2, 3, 4:**
- Write algorithm-specific analysis sections
- Explain why certain FL algorithms work better with your MARL
- Compare your MARL with different FL algorithms
- Identify strengths and weaknesses of each combination

---

### **PHASE 5: Paper Writing & Final Deliverables (Weeks 14-15)**

**ALL MEMBERS:**
- Collaborate on research paper

**Writing Assignments:**
- **Member 1:** 
  - Introduction
  - Environment/System Model section
  - Experimental Setup details
- **Member 2:** 
  - Related Work (both FL and MARL literature)
  - Federated Learning Algorithms section (FedAvg, FedProx, FedAdam, HFL)
  - MAPPO methodology
- **Member 3:** 
  - MADDPG methodology section
  - Off-policy MARL discussion
- **Member 4:** 
  - QMIX and PPO methodology sections
  - Value decomposition vs independent learning analysis
- **Member 5:** 
  - **Experimental Results** (main results section):
    - FL-MARL combination analysis
    - Top combination identification
    - Scaling analysis
  - **Literature Comparison** section:
    - Baseline method descriptions
    - Comparative results vs state-of-the-art
    - Statistical significance analysis
    - Performance improvement discussion
  - **Discussion and Analysis**:
    - When to use each FL-MARL combination
    - Trade-offs and recommendations
  - Conclusion
  - Abstract (written last)

**Final Deliverables:**
- Complete codebase with comprehensive documentation
- Trained models for all FL-MARL combinations and scales
- Results for all 20 FL-MARL combinations across 5 scales
- Literature baseline implementations and results
- Comprehensive comparison analysis
- Research paper draft including literature comparison
- Presentation slides
- Supplementary materials:
  - FL-MARL performance matrix
  - Statistical test results
  - Usage recommendation guide

---

## COORDINATION AND COMMUNICATION

### **Weekly Team Meetings**
- **Monday:** Planning meeting - assign tasks for the week
- **Wednesday:** Progress check - discuss blockers and help each other
- **Friday:** Review meeting - demonstrate progress, integration testing

### **Communication Channels**
- **Slack/Discord:** Daily communication, quick questions
- **GitHub Issues:** Track bugs, feature requests, task assignments
- **GitHub Pull Requests:** Code reviews (everyone reviews everyone's code)
- **Shared Google Doc:** Meeting notes, decisions, design discussions

### **Code Integration Rules**
1. **NO direct commits to main branch**
2. Create feature branches: `member1/environment-setup`, `member2/mappo-implementation`, etc.
3. Open Pull Request when ready
4. At least ONE other team member must review and approve
5. Run all tests before merging
6. Merge only when tests pass

### **Naming Conventions**
- **Branches:** `memberX/feature-name` (e.g., `member3/maddpg-replay-buffer`)
- **Commits:** Clear descriptions (e.g., "Add SINR calculation to channel model")
- **Files:** snake_case (e.g., `uav_swarm_env.py`)
- **Classes:** PascalCase (e.g., `MAPPOAgent`)
- **Functions:** snake_case (e.g., `compute_advantage`)
- **Variables:** snake_case (e.g., `battery_level`)

---

## DEPENDENCY MANAGEMENT

**Member 1 creates `requirements.txt` with:**

```
# Core ML/RL
torch>=2.0.0
numpy>=1.24.0
gymnasium>=0.29.0

# Multi-Agent RL
ray[rllib]>=2.9.0
stable-baselines3>=2.0.0

# Physics Simulation
pybullet>=3.2.5
pyflyt

# Federated Learning
flwr>=1.0.0

# Visualization
matplotlib>=3.7.0
seaborn>=0.12.0
tensorboard>=2.13.0
wandb

# Utilities
pyyaml>=6.0
pandas>=2.0.0
scipy>=1.10.0
tqdm>=4.65.0

# Testing
pytest>=7.3.0
pytest-cov>=4.1.0
```

**All members use the same dependencies - no adding new libraries without team discussion!**

---

## SUCCESS CRITERIA

### **Individual Success (Each Member):**
- Your assigned MARL algorithm(s) work correctly with ALL FL algorithms
- Code is well-documented and tested
- Experiments complete successfully on all 5 swarm sizes with all 4 FL algorithms
- Your sections of the paper are written

### **Team Success:**
- All 5 MARL algorithms (MAPPO, MADDPG, QMIX, PPO, SAC) implemented
- All 4 FL algorithms (HFL, FedAvg, FedProx, FedAdam) implemented
- All 20 FL-MARL combinations tested on all 5 swarm sizes
- Top 3-5 best-performing combinations identified
- Literature baselines implemented and compared
- Comprehensive comparison showing:
  - Which FL-MARL combination is best overall
  - When each combination excels
  - How our best methods compare to state-of-the-art
- Clear conclusions about trade-offs between combinations
- Complete research paper with FL-MARL analysis and literature comparison

### **Research Success:**
- Discover which FL-MARL combination works best for UAV swarms
- Understand how different FL algorithms affect MARL performance
- Understand how combination performance scales from 5 to 100 drones
- Identify when to use each FL-MARL combination (based on swarm size, objectives, constraints)
- Demonstrate improvements over existing state-of-the-art methods
- Provide actionable recommendations for practitioners
- Contribute new knowledge to federated learning and multi-agent RL communities

### **Novel Contributions:**
1. **Comprehensive FL-MARL comparison** - First study comparing multiple FL algorithms with multiple MARL algorithms for UAV swarms
2. **Scaling analysis** - Understanding how FL-MARL combinations scale from small (5) to large (100) swarms
3. **Best practice identification** - Clear guidelines on when to use which combination
4. **Performance validation** - Demonstrating improvements over literature baselines

---

## TIMELINE SUMMARY

| Week | Member 1 | Member 2 | Member 3 | Member 4 | Member 5 |
|------|----------|----------|----------|----------|----------|
| 1-2 | Environment + Infra | Research FL Algorithms | Research MADDPG | Research QMIX/PPO | Research SAC & Literature |
| 3-4 | Environment Polish | MAPPO + All FL Algos | Implement MADDPG | Implement QMIX | Implement SAC |
| 5-6 | Testing Support | Complete 4 FL Algos | MADDPG + FL Integration | PPO/QMIX + FL | SAC + FL + Metrics |
| 7-8 | Monitor Experiments | MAPPO × 4 FL Tests | MADDPG × 4 FL Tests | QMIX/PPO × 4 FL Tests | SAC × 4 FL Tests |
| 9-10 | Support Scaling | All Scale Tests | All Scale Tests | All Scale Tests | All Scale Tests |
| 11-12 | Environment Section | Review Combinations | Review Combinations | Review Combinations | FL-MARL Analysis |
| 13 | Fix Issues | Finalize Results | Finalize Results | Finalize Results | Literature Comparison |
| 14-15 | Intro & Setup Section | Related Work + Methods | MADDPG Section | QMIX/PPO Sections | Results + Analysis + Conclusion |

**Total Timeline: 15 weeks (extended from original 12 weeks due to comprehensive FL-MARL testing)**

---

## FINAL NOTES

**REMEMBER:**
- **Enhanced Research Scope:** We're not just comparing MARL algorithms, we're comparing FL-MARL COMBINATIONS (20 total)
- **Literature Comparison:** Top combinations will be benchmarked against state-of-the-art from literature
- **Python-Only Configs:** All configurations are Python dictionaries - NO YAML files, NO notebooks
  - Configs in `configs/env_configs.py`, `configs/fl_configs.py`, `configs/marl_configs.py`
  - Import directly: `from configs.env_configs import get_env_config`
  - Easier to use, better IDE support, no parsing overhead
- **NO CODE in this document** - just folder structure and task assignments
- Each member owns specific folders and files
- Work independently but coordinate frequently
- Help each other when stuck
- All code must be reviewed before merging
- Document everything as you go
- **Primary Goals:**
  1. Determine which FL-MARL combination is BEST for UAV swarms
  2. Understand when to use each combination
  3. Demonstrate improvements over existing methods
  4. Provide actionable recommendations

**KEY RESEARCH QUESTIONS TO ANSWER:**
1. Which FL algorithm works best with which MARL algorithm?
2. How do combinations scale from 5 to 100 drones?
3. What are the trade-offs between different FL-MARL combinations?
4. How do our best combinations compare to state-of-the-art from literature?
5. When should practitioners use each combination?

**GOOD LUCK TEAM! Let's build something groundbreaking! 🚁🤖📊**