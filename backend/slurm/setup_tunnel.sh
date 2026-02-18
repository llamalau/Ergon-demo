#!/bin/bash
# Set up an SSH tunnel from the local machine to the OpenPI server on SLURM.
#
# Usage:
#   ./setup_tunnel.sh                           # auto-detect compute node
#   ./setup_tunnel.sh gpu-node-01               # specify compute node
#   ./setup_tunnel.sh gpu-node-01 8001 8001     # specify ports too
#
# Prerequisites:
#   - SSH key authentication to lreddy3@oddjobs.bmi.emory.edu
#   - A running SLURM job from start_inference_server.sh

SLURM_HOST="lreddy3@oddjobs.bmi.emory.edu"
REMOTE_PORT="${2:-8001}"
LOCAL_PORT="${3:-8001}"

# Auto-detect compute node if not specified
if [ -z "$1" ]; then
    echo "Detecting compute node from SLURM queue..."
    COMPUTE_NODE=$(ssh "$SLURM_HOST" "squeue -u \$USER -n ergon-openpi -o '%N' -h" 2>/dev/null | head -1)
    if [ -z "$COMPUTE_NODE" ]; then
        echo "ERROR: No running ergon-openpi job found. Submit one first:"
        echo "  ssh $SLURM_HOST sbatch ~/start_inference_server.sh"
        exit 1
    fi
    echo "Found inference server on node: $COMPUTE_NODE"
else
    COMPUTE_NODE="$1"
fi

echo "Setting up SSH tunnel: localhost:$LOCAL_PORT -> $COMPUTE_NODE:$REMOTE_PORT via $SLURM_HOST"

# Create the tunnel (runs in background)
ssh -N -f -L "$LOCAL_PORT:$COMPUTE_NODE:$REMOTE_PORT" "$SLURM_HOST"
TUNNEL_PID=$!

echo "Tunnel established (PID: $TUNNEL_PID)"
echo "Inference server available at: http://localhost:$LOCAL_PORT"
echo ""
echo "To test: curl http://localhost:$LOCAL_PORT/health"
echo "To stop: kill $TUNNEL_PID"

# Save PID for cleanup
echo "$TUNNEL_PID" > /tmp/ergon_tunnel.pid
