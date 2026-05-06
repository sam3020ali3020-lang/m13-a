#!/usr/bin/env bash
# ==============================================================================
# Race-detection test runner
# يبني ويُشغّل stress_sensor_race تحت 3 أوضاع:
#   1. TSan          — يكشف data races
#   2. ASan + UBSan  — يكشف memory errors + undefined behavior
#   3. Release       — sanity run بدون sanitizers (baseline perf)
#
# Usage:
#   ./run_race_tests.sh              # all three
#   ./run_race_tests.sh tsan         # TSan only
#   ./run_race_tests.sh asan         # ASan only
#   ./run_race_tests.sh release      # release only
#   ./run_race_tests.sh provoke      # sanity: verify TSan catches deliberate race
# ==============================================================================

set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE"

MODE="${1:-all}"
OVERALL_RC=0

# ASLR workaround for TSan on newer Ubuntu kernels
# See: https://github.com/google/sanitizers/issues/1716
RUNNER=(setarch "$(uname -m)" -R)

run_mode() {
    local name="$1"
    local sanitizer="$2"
    local extra_cmake="${3:-}"
    local env_opts="${4:-}"

    echo ""
    echo "================================================================"
    echo "  ${name} — SANITIZER=${sanitizer}"
    echo "================================================================"

    local build_dir="build_${name}"
    rm -rf "$build_dir"
    mkdir -p "$build_dir"
    # shellcheck disable=SC2086
    cmake -S . -B "$build_dir" -DSANITIZER="$sanitizer" $extra_cmake > "$build_dir/cmake.log" 2>&1 || {
        echo "  ❌ cmake configure failed — see $build_dir/cmake.log"
        OVERALL_RC=1
        return
    }
    cmake --build "$build_dir" -j4 > "$build_dir/build.log" 2>&1 || {
        echo "  ❌ build failed — see $build_dir/build.log"
        OVERALL_RC=1
        return
    }

    echo "  ✅ built"
    echo "  running..."

    local rc=0
    # shellcheck disable=SC2086
    env $env_opts "${RUNNER[@]}" "./$build_dir/stress_sensor_race" || rc=$?

    if [[ $rc -eq 0 ]]; then
        echo "  ✅ $name: PASS (rc=$rc)"
    else
        echo "  ❌ $name: FAIL (rc=$rc)"
        OVERALL_RC=$rc
    fi
}

case "$MODE" in
    tsan)
        run_mode "tsan" "thread" "" "TSAN_OPTIONS=halt_on_error=0:second_deadlock_stack=1:history_size=7"
        ;;
    asan)
        run_mode "asan" "address" "" "ASAN_OPTIONS=detect_leaks=1:abort_on_error=0 UBSAN_OPTIONS=print_stacktrace=1"
        ;;
    release)
        run_mode "release" "none" "" ""
        ;;
    provoke)
        echo "=== Sanity: build with PROVOKE_RACE=ON to verify TSan catches deliberate race ==="
        run_mode "provoke" "thread" "-DPROVOKE_RACE=ON" "TSAN_OPTIONS=halt_on_error=0"
        # نتوقّع فشلاً — نُبدّل الـ logic
        if [[ $OVERALL_RC -ne 0 ]]; then
            echo "  ✅ sanity: TSan correctly detected the deliberate race"
            OVERALL_RC=0
        else
            echo "  ❌ sanity: TSan did NOT detect the deliberate race — sanitizer broken!"
            OVERALL_RC=1
        fi
        ;;
    all)
        run_mode "tsan" "thread" "" "TSAN_OPTIONS=halt_on_error=0:second_deadlock_stack=1:history_size=7"
        run_mode "asan" "address" "" "ASAN_OPTIONS=detect_leaks=1:abort_on_error=0 UBSAN_OPTIONS=print_stacktrace=1"
        run_mode "release" "none" "" ""
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Usage: $0 [tsan|asan|release|provoke|all]"
        exit 1
        ;;
esac

echo ""
echo "================================================================"
if [[ $OVERALL_RC -eq 0 ]]; then
    echo "  ✅ ALL RACE TESTS PASSED"
else
    echo "  ❌ RACE TESTS FAILED (rc=$OVERALL_RC)"
fi
echo "================================================================"
exit $OVERALL_RC
