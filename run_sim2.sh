#!/bin/bash
# Resolve repo root from this script's location so paths work regardless of CWD.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
export ACADOS_SOURCE_DIR="$SCRIPT_DIR/acados-main"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
cd "$SCRIPT_DIR"
SIMFILE="$SCRIPT_DIR/6DOF_v4_pure/rocket_6dof_sim.py"
exec python3 "$SIMFILE" "$@"
