#!/bin/bash
# ============================================================================
# Launch all 36 experiments: 4 FL x 3 MARL x 3 drone counts
# 8 GPUs available (RTX A6000), ~4-5 experiments per GPU
# ============================================================================
# Usage: bash scripts/run_all_36.sh
# ============================================================================

set -e

PROJ_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJ_DIR"

RESULTS_BASE="results"
LOG_DIR="logs/experiments_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

SEED=42

# Experiment matrix
FL_ALGOS=("fedavg" "fedprox" "fedadam" "hierarchical")
DRONE_COUNTS=(10 20 50)

# Parallel environments for better GPU utilization (8-16 recommended)
NUM_PARALLEL=8

# Checkpoint settings (auto-save during training)
CHECKPOINT_MAPPO=100   # Save every 100 rounds (500 total)
CHECKPOINT_QMIX=50     # Save every 50 rounds (200 total)
CHECKPOINT_MADDPG=25   # Save every 25 rounds (100 total)
# RESUME_FLAG="--resume"  # Uncomment to auto-resume from checkpoints
RESUME_FLAG=""          # Default: start fresh

echo "============================================================"
echo "  Launching 36 Experiments (4 FL x 3 MARL x 3 drone counts)"
echo "============================================================"
echo "  FL algorithms: ${FL_ALGOS[*]}"
echo "  Drone counts:  ${DRONE_COUNTS[*]}"
echo "  MARL:          mappo, qmix, maddpg"
echo "  Environment:   GPU-tensorized (default)"
echo "  Parallel envs: $NUM_PARALLEL (for 30-50% GPU utilization)"
echo "  Checkpoints:   MAPPO every ${CHECKPOINT_MAPPO}, QMIX every ${CHECKPOINT_QMIX}, MADDPG every ${CHECKPOINT_MADDPG}"
echo "  Resume mode:   ${RESUME_FLAG:-disabled}"
echo "  Seed:          $SEED"
echo "  Log dir:       $LOG_DIR"
echo "============================================================"
echo ""

GPU=0
EXP_NUM=0

# ---- MAPPO experiments: 500 FL rounds, 1 full episode each ----
echo ">>> MAPPO experiments (12 total, 500 rounds × full episode)"
for fl in "${FL_ALGOS[@]}"; do
    for drones in "${DRONE_COUNTS[@]}"; do
        EXP_NUM=$((EXP_NUM + 1))
        GPU_ID=$((( EXP_NUM - 1 ) % 8))
        SAVE_DIR="${RESULTS_BASE}/mappo/${drones}drones_${fl}_seed${SEED}"
        LOG_FILE="${LOG_DIR}/mappo_${fl}_${drones}drones.log"

        echo "  [${EXP_NUM}/36] MAPPO + ${fl} + ${drones} drones → GPU ${GPU_ID}"

        CUDA_VISIBLE_DEVICES=$GPU_ID \
        python -u scripts/train/train_mappo.py \
            --num_drones "$drones" \
            --fl_algorithm "$fl" \
            --num_rounds 500 \
            --seed "$SEED" \
            --save_dir "$SAVE_DIR" \
            --eval_interval 50 \
            --save_interval "$CHECKPOINT_MAPPO" \
            --device cuda \
            --num_parallel_envs "$NUM_PARALLEL" \
            $RESUME_FLAG \
            > "$LOG_FILE" 2>&1 &
    done
done

# ---- QMIX experiments: 200 FL rounds, 1 full episode each ----
echo ""
echo ">>> QMIX experiments (12 total, 200 rounds × full episode)"
for fl in "${FL_ALGOS[@]}"; do
    for drones in "${DRONE_COUNTS[@]}"; do
        EXP_NUM=$((EXP_NUM + 1))
        GPU_ID=$((( EXP_NUM - 1 ) % 8))
        LOG_FILE="${LOG_DIR}/qmix_${fl}_${drones}drones.log"

        echo "  [${EXP_NUM}/36] QMIX + ${fl} + ${drones} drones → GPU ${GPU_ID}"

        CUDA_VISIBLE_DEVICES=$GPU_ID \
        python -u scripts/train/train_qmix.py \
            --num_drones "$drones" \
            --fl "$fl" \
            --num_rounds 200 \
            --seed "$SEED" \
            --device cuda \
            $RESUME_FLAG \
            > "$LOG_FILE" 2>&1 &
    done
done

# ---- MADDPG experiments: 100 FL rounds, 5 full episodes per round ----
echo ""
echo ">>> MADDPG experiments (12 total, 100 rounds × 5 full episodes)"
for fl in "${FL_ALGOS[@]}"; do
    for drones in "${DRONE_COUNTS[@]}"; do
        EXP_NUM=$((EXP_NUM + 1))
        GPU_ID=$((( EXP_NUM - 1 ) % 8))
        LOG_FILE="${LOG_DIR}/maddpg_${fl}_${drones}drones.log"

        echo "  [${EXP_NUM}/36] MADDPG + ${fl} + ${drones} drones → GPU ${GPU_ID}"

        CUDA_VISIBLE_DEVICES=$GPU_ID \
        python -u scripts/train/train_maddpg.py \
            --num_drones "$drones" \
            --fl "$fl" \
            --num_rounds 100 \
            --episodes_per_round 5 \
            --seed "$SEED" \
            --device cuda \
            $RESUME_FLAG \
            > "$LOG_FILE" 2>&1 &
    done
done

echo ""
echo "============================================================"
echo "  All 36 experiments launched!"
echo "============================================================"
echo "  Monitor with: tail -f ${LOG_DIR}/*.log"
echo "  Check status: ps aux | grep train_ | grep -v grep | wc -l"
echo "  GPU usage:    nvidia-smi"
echo "============================================================"
