# Checkpoint System for Training Resumption

## Overview

Added comprehensive checkpoint functionality to save and resume training progress for all 3 MARL algorithms (MAPPO, MADDPG, QMIX). This allows you to:
- **Resume from interruptions** - Continue training after crashes or Ctrl+C
- **Save compute resources** - Don't restart from scratch when experiments fail
- **Experiment incrementally** - Train for N rounds, evaluate, then continue
- **Preserve FL state** - FedAdam momentum/velocity and FedProx global weights are saved

## What Gets Saved

Each checkpoint includes:
1. **Agent models** - All network weights (actor, critic, target networks)
2. **Optimizer states** - Adam momentum, learning rate schedules
3. **Trainer state** - Current round, total steps, training history
4. **FL aggregator state** - FedAdam m/v tensors, FedProx global weights, round number
5. **Metrics history** - All logged metrics from previous rounds

## Usage

### Basic Usage - Auto-save Checkpoints

Checkpoints are automatically saved during training at intervals:

```bash
# MAPPO - saves every 10 rounds (configurable with --save_interval)
python scripts/train/train_mappo.py \
    --num_drones 10 \
    --fl_algorithm fedadam \
    --num_rounds 100 \
    --save_interval 10

# Checkpoints saved at: checkpoints/mappo/round_10, round_20, round_30, ...
```

### Resume Training

Use `--resume` flag to continue from a checkpoint:

```bash
# Auto-detect latest checkpoint for this config
python scripts/train/train_mappo.py \
    --num_drones 10 \
    --fl_algorithm fedadam \
    --num_rounds 100 \
    --resume

# Or specify exact checkpoint name
python scripts/train/train_mappo.py \
    --num_drones 10 \
    --fl_algorithm fedadam \
    --num_rounds 100 \
    --resume \
    --checkpoint_name "round_50"
```

### Interrupt Recovery

If training is interrupted (Ctrl+C), checkpoint is automatically saved:

```bash
# Start training
python scripts/train/train_mappo.py --num_drones 10 --num_rounds 100

# Press Ctrl+C during training
^C
Training interrupted by user!
Saving checkpoint: 10drones_fedavg_interrupted_20260221_003045

# Resume from interrupted checkpoint
python scripts/train/train_mappo.py --num_drones 10 --num_rounds 100 \
    --resume --checkpoint_name "10drones_fedavg_interrupted_20260221_003045"
```

## Checkpoint Locations

```
checkpoints/
├── mappo/
│   ├── round_10/
│   │   ├── agent_0.pt
│   │   ├── agent_1.pt
│   │   ├── ...
│   │   └── trainer_state.pt
│   ├── round_20/
│   └── 10drones_fedadam_interrupted_20260221_003045/
├── maddpg/
│   └── 3drones_fedavg/
└── qmix/
    └── 10drones_fedprox/
```

## Examples for Each Algorithm

### MAPPO

```bash
# Train with auto-checkpointing
python scripts/train/train_mappo.py \
    --num_drones 20 \
    --fl_algorithm hierarchical \
    --num_rounds 200 \
    --save_interval 20 \
    --num_parallel_envs 8

# Resume after interruption
python scripts/train/train_mappo.py \
    --num_drones 20 \
    --fl_algorithm hierarchical \
    --num_rounds 200 \
    --resume \
    --checkpoint_name "round_100"
```

### MADDPG

```bash
# Train with auto-checkpointing (checkpoint saved after each FL round)
python scripts/train/train_maddpg.py \
    --num_drones 10 \
    --fl_algorithm fedadam \
    --num_rounds 100 \
    --episodes_per_round 5

# Resume
python scripts/train/train_maddpg.py \
    --num_drones 10 \
    --fl_algorithm fedadam \
    --num_rounds 100 \
    --resume \
    --checkpoint_name "10drones_fedadam"
```

### QMIX

```bash
# Train
python scripts/train/train_qmix.py \
    --num_drones 10 \
    --fl fedprox \
    --num_rounds 100

# Resume
python scripts/train/train_qmix.py \
    --num_drones 10 \
    --fl fedprox \
    --num_rounds 100 \
    --resume \
    --checkpoint_name "10drones_fedprox"
```

## Configuration

### Auto-save Intervals

Control how often checkpoints are saved:

```bash
# MAPPO - save every N rounds
--save_interval 10  # Save at rounds 10, 20, 30, ...

# Default intervals:
# - MAPPO: 10 rounds
# - MADDPG: After each round (training_history saved)
# - QMIX: Manual or on-demand
```

### Checkpoint Naming

Auto-generated names follow pattern:
- `{num_drones}drones_{fl_algorithm}` - Default checkpoint name
- `round_{N}` - Periodic checkpoint at round N
- `{config}_interrupted_{timestamp}` - Saved on Ctrl+C

Custom names:
```bash
--checkpoint_name "my_experiment_v2"
```

## Testing

Run checkpoint test suite:

```bash
cd /home/skyvision/HFL-UAV-Swarm-Comparison
bash test_checkpoint.sh
```

This will:
1. Train MAPPO for 3 rounds
2. Save checkpoint at round 2
3. Resume training from round 2 to round 5
4. Verify all algorithms support checkpointing

## Implementation Details

### What's Preserved

1. **MAPPO Trainer State:**
   - `current_round`: Resume point
   - `total_steps`: Cumulative environment steps
   - `training_history`: All previous metrics
   - `config`: Training configuration

2. **FedAdam Aggregator:**
   - `m`: First moment estimates (momentum)
   - `v`: Second moment estimates (adaptive LR)
   - `round_number`: FL round counter

3. **FedProx Aggregator:**
   - `global_weights`: Reference model for proximal term
   - `round_number`: FL round counter

4. **Agent Models:**
   - Actor/Critic networks
   - Target networks (MADDPG/QMIX)
   - Optimizer states (Adam momentum)

### Resume Behavior

When resuming:
- Training continues from `current_round + 1`
- FL aggregator state restored (important for FedAdam convergence)
- Agent exploration parameters (epsilon, noise) restored
- Training history preserved and extended

## Troubleshooting

### Checkpoint Not Found

```
Warning: Checkpoint not found: checkpoints/mappo/round_50
Starting from scratch (checkpoint not found)
```

**Solution:** Verify checkpoint name and path:
```bash
ls checkpoints/mappo/
```

### FedAdam Convergence Issues After Resume

If FedAdam behaves differently after resume, check:
1. Momentum/velocity loaded correctly (should see in logs)
2. `round_number` matches (aggregator should continue from correct round)

### Large Checkpoint Files

Each checkpoint ~50-200MB depending on:
- Number of agents (10-100 drones)
- Network size (hidden_dim)
- FL algorithm (FedAdam has additional m/v tensors)

To reduce size:
- Increase `--save_interval` (keep fewer checkpoints)
- Delete old checkpoints: `rm -rf checkpoints/mappo/round_*`

## Benefits for Your 36 Experiments

With checkpointing, you can now:

1. **Handle GPU crashes** - Resume all 36 experiments without restarting
2. **Incremental training** - Train 50 rounds, evaluate, then extend to 100
3. **Resource optimization** - Pause low-priority experiments, resume later
4. **Experiment comparison** - Checkpoint at round N, try different configs

Example workflow:
```bash
# Terminal 1: MAPPO experiments
for drones in 3 10 20; do
    python scripts/train/train_mappo.py --num_drones $drones --num_rounds 100 --resume
done

# Terminal 2: MADDPG experiments  
for fl in fedavg fedadam fedprox hierarchical; do
    python scripts/train/train_maddpg.py --fl_algorithm $fl --num_rounds 100 --resume
done
```

If any experiment crashes, simply re-run with `--resume` and it continues!
