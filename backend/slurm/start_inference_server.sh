#!/bin/bash
#SBATCH --job-name=ergon-openpi
#SBATCH --partition=gpu
#SBATCH --gres=gpu:1
#SBATCH --mem=32G
#SBATCH --cpus-per-task=4
#SBATCH --time=08:00:00
#SBATCH --output=openpi_server_%j.log
#
# Start the OpenPI (pi0) inference server on a GPU node.
# The server listens on PORT and accepts observation -> action requests.
#
# Usage:
#   sbatch start_inference_server.sh
#   sbatch start_inference_server.sh 8001  # custom port
#
# After the job starts, set up an SSH tunnel:
#   ssh -N -L 8001:<compute_node>:8001 lreddy3@oddjobs.bmi.emory.edu
#
# Then configure Ergon: CONTROL_MODEL_URL=http://localhost:8001

PORT="${1:-8001}"

echo "============================================"
echo "OpenPI Inference Server"
echo "Node: $(hostname)"
echo "GPU:  $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'unknown')"
echo "Port: $PORT"
echo "Time: $(date)"
echo "============================================"

# Load modules (adjust for your cluster)
module load cuda 2>/dev/null || true
module load python 2>/dev/null || true

# Activate environment
export PATH="$HOME/.cargo/bin:$PATH"
cd "$HOME/openpi"

# Start the inference server
# pi0_base is the general-purpose model; swap for fine-tuned checkpoints as needed
echo "Starting server on port $PORT..."
uv run scripts/serve_policy.py \
    --env.policy_cls=pi0:Pi0Policy \
    --env.config=pi0_base \
    --port="$PORT"
