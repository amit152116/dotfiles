#!/bin/bash
# Reads ~/.task_focus and ~/.task_park_stack — no taskwarrior process if no focus.

FOCUS_FILE="$HOME/.task_focus"
PARK_STACK="$HOME/.task_park_stack"

focused_id=$(tr -d '[:space:]' <"$FOCUS_FILE" 2>/dev/null)
[[ "$focused_id" =~ ^[0-9]+$ ]] || focused_id=""

park_depth=0
if [[ -f "$PARK_STACK" ]] && [[ -s "$PARK_STACK" ]]; then
    park_depth=$(wc -l <"$PARK_STACK")
fi

project=$(tmux display-message -p '#S' 2>/dev/null | tr -cd '[:alnum:]_-')
pending=0
if [[ -n "$project" ]]; then
    pending=$(task project:"$project" status:pending count 2>/dev/null)
fi
pending=${pending:-0}

park_prefix=""
[[ $park_depth -gt 0 ]] && park_prefix="⏸${park_depth} "

if [[ -n "$focused_id" ]]; then
    output=$(task "$focused_id" export 2>/dev/null | python3 -c '
import json, sys, datetime

data = json.load(sys.stdin)
if not data:
    print("")
    sys.exit()
t = data[0]

raw_desc = t.get("description", "")
desc = (raw_desc[:35] + "…") if len(raw_desc) > 35 else raw_desc

pri = t.get("priority", "")
pri_str = f" [{pri}]" if pri else ""

elapsed_str = ""
if t.get("start"):
    dt = datetime.datetime.strptime(t["start"], "%Y%m%dT%H%M%SZ").replace(tzinfo=datetime.timezone.utc)
    mins = int((datetime.datetime.now(datetime.timezone.utc) - dt).total_seconds() / 60)
    elapsed_str = f" {mins}m" if mins < 60 else f" {mins//60}h{mins%60:02d}m"

print(f"{desc}{pri_str}{elapsed_str}")
' 2>/dev/null)

    if [[ -n "$output" ]]; then
        echo "${park_prefix}◉ ${output} | ${pending}p"
    else
        echo "${park_prefix}· ${pending}p"
    fi
else
    echo "${park_prefix}· ${pending}p"
fi
