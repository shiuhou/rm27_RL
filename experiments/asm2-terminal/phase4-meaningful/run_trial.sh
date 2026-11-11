#!/usr/bin/env bash
# Launch the single continuation and serve its independent development requests.
set -euo pipefail
run=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(cd -- "$run/../../../.." && pwd)
cd "$root"
relative=${run#"$root/"}
python=.agents/runs/rm27-stage1/native-env/bin/python
export PYTHONPATH="$root/src:$root" OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
test ! -e "$run/learning"
"$python" "$run/continue_braking.py" --check
set -o noclobber
./run_asm2_terminal_pilot.sh braking-continuation > "$run/learning-launch.log" 2>&1 &
pilot_pid=$!
while kill -0 "$pilot_pid" 2>/dev/null; do
    for request in "$run"/learning/request-*.json; do
        [[ -f "$request" ]] || continue
        frame=${request##*/request-}
        frame=${frame%.json}
        output="$run/learning/eval-$frame.json"
        [[ ! -e "$output" ]] || continue
        pending="$run/learning/.eval-$frame.pending"
        if timeout 150s "$python" -m rm27_asm2.terminal_native_eval \
            --suite recovery-development --experiment configs/rm27_asm2_terminal_recovery.json \
            --actor "$relative/learning/checkpoint-$frame.npz" \
            --checkpoint "$relative/learning/checkpoint-$frame.pt" \
            --workers 12 --output "$pending"; then
            mv -- "$pending" "$output"
        else
            echo "Independent development evaluation failed at $frame. Learner is waiting for its result." >&2
            break 2
        fi
    done
    sleep 2
done
wait "$pilot_pid"
