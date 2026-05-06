#!/bin/bash
# ============================================================================
# M130 SITL Validation Test Runner
# اختبار التحقق SITL للصاروخ M130
# ============================================================================
#
# Usage:
#   ./run_sitl_test.sh                    # Full test (baseline + SITL + compare)
#   ./run_sitl_test.sh --baseline-only    # Python baseline only
#   ./run_sitl_test.sh --compare-only     # Compare existing CSVs
#
# Prerequisites:
#   1. Python venv with numpy, pyyaml
#   2. acados libraries compiled
#   3. For SITL: PX4 binary compiled for x86_64 with rocket_mpc module
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"
M13_DIR="$(dirname "$SIM_DIR")"

# acados environment
export LD_LIBRARY_PATH="${M13_DIR}/acados-main/lib:${LD_LIBRARY_PATH:-}"
export ACADOS_SOURCE_DIR="${M13_DIR}/acados-main"

# Activate venv if it exists
if [ -f "${M13_DIR}/.venv/bin/activate" ]; then
    source "${M13_DIR}/.venv/bin/activate"
fi

# Create results directory
mkdir -p "${SCRIPT_DIR}/results"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║       M130 SITL VALIDATION TEST                     ║"
echo "║       Environment Setup Complete                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  ACADOS_SOURCE_DIR: ${ACADOS_SOURCE_DIR}"
echo "  LD_LIBRARY_PATH:   ${LD_LIBRARY_PATH}"
echo "  Python:            $(which python3)"
echo ""

cd "${SCRIPT_DIR}"
exec python3 run_sitl_test.py "$@"
