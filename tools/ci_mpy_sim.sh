#!/usr/bin/env bash
# CI helper: run tests/mpy_sim_check.py <mode> under MicroPython and fail on
# non-zero exit, timeout, or any Python traceback in the output.
# Needs $MICROPYTHON (path to the unix-port binary).
set -u -o pipefail
mode="$1"
log="sim-${mode}.log"
MICROPYPATH=tests/mock:tests:src timeout 1200 "$MICROPYTHON" tests/mpy_sim_check.py "$mode" > "$log" 2>&1
rc=$?
tail -n 40 "$log"
if [ $rc -eq 124 ]; then echo "::error::sim $mode timed out after 20 min"; exit 1; fi
if [ $rc -ne 0 ]; then echo "::error::sim $mode exited with $rc"; exit 1; fi
if grep -nE "Traceback|^[A-Za-z]*Error:" "$log"; then echo "::error::traceback/error in sim $mode output"; exit 1; fi
grep -q "^sim t =" "$log" || { echo "::error::sim $mode did not reach its final summary line"; exit 1; }
echo "sim $mode OK"
