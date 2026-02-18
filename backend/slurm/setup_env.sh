#!/bin/bash
# One-time setup: install OpenPI on the SLURM cluster.
# Run this on the login node (or in an interactive session).
#
# Usage: ssh lreddy3@oddjobs.bmi.emory.edu 'bash -s' < setup_env.sh

set -e

echo "=== Setting up OpenPI environment ==="

# Install uv if not present
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.cargo/bin:$PATH"
fi

# Clone OpenPI
if [ ! -d "$HOME/openpi" ]; then
    echo "Cloning OpenPI..."
    git clone https://github.com/Physical-Intelligence/openpi.git "$HOME/openpi"
fi

cd "$HOME/openpi"

# Create virtual environment and install dependencies
echo "Installing OpenPI dependencies (this may take a while)..."
uv sync --all-extras

# Download the pi0 base model weights
echo "Downloading pi0 base model weights..."
uv run scripts/download_weights.py --policy pi0_base

echo ""
echo "=== Setup complete ==="
echo "To start the inference server, submit a SLURM job:"
echo "  sbatch ~/openpi/slurm/start_inference_server.sh"
echo ""
echo "Or copy the scripts from the Ergon repo:"
echo "  scp backend/slurm/start_inference_server.sh lreddy3@oddjobs.bmi.emory.edu:~/"
