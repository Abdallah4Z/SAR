#!/bin/bash
# ============================================================================
# Launch 12 QMIX experiments: 4 FL x 3 drone counts
# 8 GPUs available (RTX A6000), 1-2 experiments per GPU
# ============================================================================
# Usage: bash scripts/run_qmix_12.sh
# ============================================================================

set -e

PROJ_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJ_DIR"

RESULTS_BASE="results"
LOG_DIR="logs/qmix_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

SEED=42

# Experiment matrix
FL_ALGOS=("fedavg" "fedprox" "fedadam" "hierarchical")
DRONE_COUNTS=(10 20 50)

# Parallel environments for better GPU utilization
NUM_PARALLEL=8

# Checkpoint settings (auto-save during training)
CHECKPOINT_QMIX=50     # Save every 50 rounds (200 total)
# RESUME_FLAG="--resume"  # Uncomment to auto-resume from checkpoints
RESUME_FLAG=""          # Default: start fresh

echo "============================================================"
echo "  Launching 12 QMIX Experiments (4 FL x 3 drone counts)"
echo "============================================================"
echo "  FL algorithms: ${FL_ALGOS[*]}"
echo "  Drone counts:  ${DRONE_COUNTS[*]}"
echo "  MARL:          QMIX only"
echo "  Environment:   GPU-tensorized (default)"
echo "  Parallel envs: $NUM_PARALLEL (for 30-50% GPU utilization)"
echo "  Checkpoints:   Every ${CHECKPOINT_QMIX} rounds"
echo "  Resume mode:   ${RESUME_FLAG:-disabled}"
echo "  Seed:          $SEED"
echo "  Log dir:       $LOG_DIR"
echo "============================================================"
echo ""

EXP_NUM=0

# ---- QMIX experiments: 200 FL rounds, 1 full episode each ----
echo ">>> QMIX experiments (12 total, 200 rounds × full episode)"
for fl in "${FL_ALGOS[@]}"; do
    for drones in "${DRONE_COUNTS[@]}"; do
        EXP_NUM=$((EXP_NUM + 1))
        GPU_ID=$((( EXP_NUM - 1 ) % 8))
        LOG_FILE="${LOG_DIR}/qmix_${fl}_${drones}drones.log"

        echo "  [${EXP_NUM}/12] QMIX + ${fl} + ${drones} drones → GPU ${GPU_ID}"

        CUDA_VISIBLE_DEVICES=$GPU_ID \
        python -u scripts/train/train_qmix.py \
            --num_drones "$drones" \
            --fl "$fl" \
            --num_rounds 200 \
            --seed "$SEED" \
            --device cuda \
            --save_interval "$CHECKPOINT_QMIX" \
            $RESUME_FLAG \
            > "$LOG_FILE" 2>&1 &
    done
done

echo ""
echo "============================================================"
echo "  All 12 QMIX experiments launched!"
echo "============================================================"
echo "  Monitor with: tail -f ${LOG_DIR}/*.log"
echo "  Check status: ps aux | grep train_qmix | grep -v grep | wc -l"
echo "  GPU usage:    nvidia-smi"
echo "============================================================"
