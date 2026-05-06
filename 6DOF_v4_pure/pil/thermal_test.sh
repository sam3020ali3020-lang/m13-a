#!/bin/bash
# ════════════════════════════════════════════════════════════════════════════
#  Thermal Stress Test for PIL
# ════════════════════════════════════════════════════════════════════════════
#
#  الغرض: اختبار أداء MPC تحت درجات حرارة مختلفة للـ CPU الهاتف
#  لمحاكاة سيناريو منصة الإطلاق (شمس + انتظار طويل)
#
#  الاستخدام:
#    ./thermal_test.sh                         # قياس 1 طيران بدون stress
#    ./thermal_test.sh --stress SECONDS        # تسخين ثم طيران
#    ./thermal_test.sh --sweep                 # 3 طيرانات: بارد/دافئ/ساخن
#
#  المتطلبات:
#    - adb متصل بالهاتف (4c2b17f0)
#    - التطبيق مُثبَّت
#    - adb reverse tcp:4560 tcp:5760
#
#  المخرجات:
#    thermal_results_YYYYMMDD_HHMMSS.csv  (temp → score mapping)
# ════════════════════════════════════════════════════════════════════════════

set -e

cd "$(dirname "$0")"

ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-/home/yoga/m13/m13/acados-main}"
export ACADOS_SOURCE_DIR
export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ACADOS_SOURCE_DIR}/interfaces/acados_template:$(realpath ..):$(realpath ../mpc):${PYTHONPATH:-}"

RESULTS_DIR="results"
mkdir -p "${RESULTS_DIR}"
RESULTS_CSV="${RESULTS_DIR}/thermal_results_$(date +%Y%m%d_%H%M%S).csv"

# ─── Helpers ────────────────────────────────────────────────────────────────

get_max_cpu_temp() {
    # يُعيد أعلى درجة حرارة لـ cpu زون بالـ °C (integer)
    adb shell 'for t in /sys/class/thermal/thermal_zone*/temp; do
        awk "{if (\$1 > 30000) print \$1/1000}" "$t" 2>/dev/null
    done' 2>/dev/null | sort -rn | head -1
}

start_stress() {
    local workers=${1:-8}
    echo "  [stress] starting ${workers} yes workers on phone..."
    adb shell "cat > /data/local/tmp/stress.sh << 'EOF'
#!/system/bin/sh
for i in \$(seq 1 ${workers}); do
    yes > /dev/null &
done
EOF
chmod 755 /data/local/tmp/stress.sh
nohup /data/local/tmp/stress.sh > /dev/null 2>&1 &" > /dev/null 2>&1
    sleep 1
}

stop_stress() {
    echo "  [stress] killing workers..."
    adb shell 'ps -A 2>/dev/null | awk "/ yes\$/ {print \$2}" | xargs -r kill -9 2>/dev/null; rm -f /data/local/tmp/stress.sh' > /dev/null 2>&1 || true
    sleep 1
}

setup_tunnels() {
    adb reverse tcp:4560 tcp:4560 > /dev/null
    adb reverse tcp:5760 tcp:5760 > /dev/null
}

wait_for_cooldown() {
    local target=${1:-50}
    echo "  [cooldown] waiting until CPU < ${target}°C..."
    while :; do
        T=$(get_max_cpu_temp)
        T_int=${T%.*}
        if [ "${T_int:-99}" -lt "${target}" ]; then
            echo "  [cooldown] reached ${T}°C"
            break
        fi
        printf "  [cooldown] %s°C ...\r" "${T}"
        sleep 5
    done
}

wait_for_heat() {
    local target=${1:-65}
    local max_wait=${2:-120}
    echo "  [heatup] waiting until CPU >= ${target}°C (max ${max_wait}s)..."
    local elapsed=0
    while [ "${elapsed}" -lt "${max_wait}" ]; do
        T=$(get_max_cpu_temp)
        T_int=${T%.*}
        if [ "${T_int:-0}" -ge "${target}" ]; then
            echo "  [heatup] reached ${T}°C in ${elapsed}s"
            return 0
        fi
        printf "  [heatup] %s°C (%ds elapsed)...\r" "${T}" "${elapsed}"
        sleep 5
        elapsed=$((elapsed + 5))
    done
    echo "  [heatup] timeout: stayed at ${T}°C"
    return 1
}

run_pil_and_extract() {
    # يُشغّل PIL ويستخرج (score, range_err, max_alpha) من المخرجات
    local label=$1
    echo
    echo "  ────────────────────────────────────────────"
    echo "  ▶ RUN: ${label}"
    echo "  ────────────────────────────────────────────"
    setup_tunnels

    local temp_before
    temp_before=$(get_max_cpu_temp)
    echo "  [T] CPU before: ${temp_before}°C"
    echo "  [⏳] Press START in the Android app..."

    local out_file="${RESULTS_DIR}/_pil_stdout_$$.log"
    python3 pil_runner.py > "${out_file}" 2>&1

    local temp_after
    temp_after=$(get_max_cpu_temp)
    echo "  [T] CPU after:  ${temp_after}°C"

    # استخراج النتائج
    local score range_err max_alpha max_fin time_s
    score=$(grep -oP 'PASS|WARN|FAIL \(\K[0-9.]+' "${out_file}" | head -1)
    [ -z "${score}" ] && score=$(grep -oP '\(\K[0-9.]+(?=/100\))' "${out_file}" | head -1)
    range_err=$(grep -oP 'err \K[+-]?[0-9.]+(?=%)' "${out_file}" | head -1)
    max_alpha=$(grep -oP 'Max \|α\|\(flight\): \K[0-9.]+' "${out_file}" | head -1)
    max_fin=$(grep -oP 'Max fin: \K[0-9.]+' "${out_file}" | head -1)
    time_s=$(grep -oP 'Time:\s+\K[0-9.]+' "${out_file}" | head -1)

    # أضف صف لـ CSV
    echo "${label},${temp_before},${temp_after},${score:-?},${range_err:-?},${max_alpha:-?},${max_fin:-?},${time_s:-?}" >> "${RESULTS_CSV}"

    echo "  ✓ Score=${score}  range_err=${range_err}%  |α|=${max_alpha}°  fin=${max_fin}°  time=${time_s}s"
    rm -f "${out_file}"
}

# ─── Main ────────────────────────────────────────────────────────────────────

MODE="${1:-single}"

echo "╔══════════════════════════════════════════════════════╗"
echo "║          PIL Thermal Stress Test                      ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Results → ${RESULTS_CSV}"
echo "╚══════════════════════════════════════════════════════╝"

# header CSV
echo "label,cpu_before_C,cpu_after_C,score,range_err_pct,max_alpha_deg,max_fin_deg,time_s" > "${RESULTS_CSV}"

trap stop_stress EXIT

case "${MODE}" in
    --stress)
        DURATION=${2:-60}
        echo
        echo "▶ Mode: STRESS (${DURATION}s heatup then run PIL)"
        wait_for_cooldown 55
        start_stress 8
        sleep "${DURATION}"
        run_pil_and_extract "stressed_${DURATION}s"
        stop_stress
        ;;

    --sweep)
        echo
        echo "▶ Mode: SWEEP (cold → warm → hot)"

        # 1) COLD
        wait_for_cooldown 50
        run_pil_and_extract "cold"

        # 2) WARM (30s stress)
        start_stress 8
        wait_for_heat 62 60
        run_pil_and_extract "warm_62C"
        stop_stress

        # 3) HOT (60s stress to reach 75°C+)
        wait_for_cooldown 55  # allow some cooldown
        start_stress 8
        wait_for_heat 74 90
        run_pil_and_extract "hot_75C"
        stop_stress

        wait_for_cooldown 50
        ;;

    *)
        # single run
        echo
        echo "▶ Mode: SINGLE run (current phone temperature)"
        run_pil_and_extract "single"
        ;;
esac

echo
echo "═══════════════════════════════════════════════════════"
echo "  Thermal Test Results"
echo "═══════════════════════════════════════════════════════"
cat "${RESULTS_CSV}" | column -t -s,
echo
echo "  CSV saved: ${RESULTS_CSV}"
echo "═══════════════════════════════════════════════════════"
