#!/usr/bin/env bash
# Minimal thermal sidecar for HIL runs.  Polls phone via adb every 500 ms and
# writes a CSV that hil_analysis.py picks up automatically when it sits next
# to the flight CSV as <flight_stem>_thermal.csv.
#
# Per the v5.1 thermal-review:
#   • Restricted to CPU-related zones only (review note #3) — battery/modem/
#     GPU are excluded so the reported max actually reflects CPU thermal
#     throttling rather than unrelated SoC heat.
#   • Captures cpu0/cpu4/cpu7 scaling_cur_freq alongside temperature (review
#     note #2) so a frequency drop — the SYMPTOM of throttling — is visible
#     in the report, not just temperature.
# Usage: ./_thermal_quick.sh <out_csv_path>   (run as background process)
set -u
OUT="${1:-/tmp/hil_thermal.csv}"
echo "wall_time,cpu_temp_c,cpu0_freq_mhz,cpu4_freq_mhz,cpu7_freq_mhz" > "$OUT"

while true; do
    NOW=$(date +%s.%3N)
    # Single adb call returns 4 lines: max_cpu_temp_mC, cpu0_freq, cpu4_freq, cpu7_freq
    # CPU-only zone filter: only zones whose `type` contains "cpu" (case-insensitive).
    OUT_RAW=$(adb shell 'M=0; for z in /sys/class/thermal/thermal_zone*; do typ=$(cat "$z/type" 2>/dev/null); case "$typ" in *cpu*|*CPU*) v=$(cat "$z/temp" 2>/dev/null || echo 0); if [ "$v" -gt "$M" ] 2>/dev/null; then M=$v; fi ;; esac; done; echo $M; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo 0; cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq 2>/dev/null || echo 0; cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq 2>/dev/null || echo 0' 2>/dev/null | tr -d '\r')
    if [ -n "$OUT_RAW" ]; then
        T_mC=$(echo "$OUT_RAW" | sed -n '1p')
        F0=$(echo  "$OUT_RAW" | sed -n '2p')
        F4=$(echo  "$OUT_RAW" | sed -n '3p')
        F7=$(echo  "$OUT_RAW" | sed -n '4p')
        TC=$(awk -v t="${T_mC:-0}" 'BEGIN{printf "%.2f", t/1000}')
        F0M=$(( ${F0:-0} / 1000 ))
        F4M=$(( ${F4:-0} / 1000 ))
        F7M=$(( ${F7:-0} / 1000 ))
        echo "${NOW},${TC},${F0M},${F4M},${F7M}" >> "$OUT"
    fi
    sleep 0.5
done
