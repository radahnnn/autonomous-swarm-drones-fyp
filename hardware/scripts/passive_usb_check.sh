#!/usr/bin/env bash
# ==============================================================================
# passive_usb_check.sh - READ-ONLY check that the ESP32 (and Matek) USB links are present and stable.
#
#   usage: hardware/scripts/passive_usb_check.sh [minutes=5] [log_dir=./bench_logs]
#
# What it does: every 5 s it lists the USB serial devices and any CAN network interfaces, counts how many
# times each device appeared/disappeared, and writes a log. At the end it prints a PASS / FAIL summary.
#
# What it does NOT do: it never opens a serial port (opening a CP2102 can toggle DTR/RTS and reset an ESP32),
# sends no MAVLink, no CAN frame, no arm or motor command. Safe to run with props on ONLY IF the flight battery
# is disconnected (motors have no power from USB). See hardware/reports/BENCH_SESSION_CHECK_2026-11-23.md.
# ==============================================================================
set -u
MINUTES="${1:-5}"
LOG_DIR="${2:-./bench_logs}"
mkdir -p "${LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="${LOG_DIR}/passive_usb_check_${STAMP}.log"

say() { echo "$@" | tee -a "${LOG}"; }

say "== passive USB check, ${MINUTES} min, started $(date -Is) =="
say "-- lsusb"
lsusb 2>&1 | tee -a "${LOG}"
say "-- kernel messages about USB serial (last 25)"
( dmesg 2>/dev/null || journalctl -k --no-pager 2>/dev/null ) | grep -iE "usb|tty|cp210|ch34|cdc_acm|can" | tail -25 | tee -a "${LOG}"

declare -A SEEN=() CHANGES=()
PREV=""
END=$(( $(date +%s) + MINUTES * 60 ))
SAMPLES=0
while [ "$(date +%s)" -lt "${END}" ]; do
    NOW="$(ls /dev/serial/by-id 2>/dev/null | sort | tr '\n' ' ')|$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')|$(ip -br link 2>/dev/null | grep -iE 'can' | awk '{print $1":"$2}' | tr '\n' ' ')"
    SAMPLES=$((SAMPLES + 1))
    if [ "${NOW}" != "${PREV}" ]; then
        say "$(date +%T) change -> ${NOW}"
        [ -n "${PREV}" ] && CHANGES[all]=$(( ${CHANGES[all]:-0} + 1 ))
        PREV="${NOW}"
    fi
    sleep 5
done

say "-- summary"
say "samples: ${SAMPLES}   presence changes after first sample: ${CHANGES[all]:-0}"
say "last state: ${PREV}"
BYID="$(ls /dev/serial/by-id 2>/dev/null | wc -l)"
CANIF="$(ip -br link 2>/dev/null | grep -ciE 'can')"
VERDICT="PASS"
[ "${BYID}" -eq 0 ] && { say "FAIL: no /dev/serial/by-id device present"; VERDICT="FAIL"; }
[ "${CHANGES[all]:-0}" -gt 0 ] && { say "FAIL: the device set changed during the run (disconnect/reconnect)"; VERDICT="FAIL"; }
[ "${CANIF}" -eq 0 ] && say "NOTE: no SocketCAN interface on this computer (expected: CAN is between ESP32 and Matek, not on the laptop)"
say "USB presence/stability: ${VERDICT}   (log: ${LOG})"
say "This proves enumeration only. It does not prove ESP32 firmware, CAN traffic, or a command path."
[ "${VERDICT}" = "PASS" ]
