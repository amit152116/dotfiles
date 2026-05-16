# Taskwarrior helper to get current project
_tmux_project() {
    if [[ -n "$TMUX" ]]; then
        tmux display-message -p '#S' | tr -cd '[:alnum:]_-'
    fi
}

# Taskwarrior wrapper to automatically scope to the current tmux session
function t() {
    local project=$(_tmux_project)
    if [[ -n "$project" ]]; then
        task project:"$project" $@
    else
        task $@
    fi

}

# Interactive task add with prompts
tadd() {
    local project=$(_tmux_project)
    local verb_tags extra_tags desc priority due all_tags cmd

    read "desc?Description: "
    [[ -z "$desc" ]] && echo "Aborted: no description" && return 1

    verb_tags=$(printf "implement\nfix\nrefactor\ntest\ndesign\ndebug\ndocs\nreview\ndeploy\nresearch\nconfig\nintegrate" |
        fzf --multi --prompt="Verb tags (TAB=multi, ENTER=done): " | tr '\n' ' ')

    read "extra_tags?Extra tags (space-separated, blank=none): "

    priority=$(printf "H - High\nM - Medium\nL - Low\nnone" |
        fzf --prompt="Priority: " | cut -d' ' -f1)
    [[ "$priority" == "none" ]] && priority=""

    read "due?Due date (today/fri/eom/blank=none): "

    all_tags=$(
        {
            echo "$verb_tags"
            echo "$extra_tags"
        } |
            tr ' ' '\n' | grep -v '^$' | sed 's/^+*/+/' | sort -u | tr '\n' ' '
    )

    local -a cmd_args=(add "$desc")
    [[ -n "$all_tags" ]] && cmd_args+=($=all_tags)
    [[ -n "$project" ]] && cmd_args+=(project:"$project")
    [[ -n "$priority" ]] && cmd_args+=(priority:"$priority")
    [[ -n "$due" ]] && cmd_args+=(due:"$due")

    echo "→ task ${cmd_args[*]}"
    task "${cmd_args[@]}"
}

_task_fzf_select() {
    local prompt="$1"
    shift
    local filter=("$@")
    local tmp
    tmp=$(mktemp)

    task "${filter[@]}" status:pending export 2>/dev/null |
        python3 -c '
import json, sys
tasks = json.load(sys.stdin)
for t in sorted(tasks, key=lambda x: -x.get("urgency", 0)):
    tid  = t["id"]
    pri  = t.get("priority", " ")
    desc = t["description"][:55]
    tags = " ".join("+" + g for g in t.get("tags", []))
    due  = t["due"][:10] if t.get("due") else ""
    print(f"{tid:4d}  [{pri}]  {desc:<55}  {tags:<30}  {due}")
' >"$tmp"

    local ids
    ids=$(fzf --multi --prompt="$prompt" \
        --layout=reverse --border --no-input \
        --bind="j:down,k:up,ctrl-d:half-page-down,ctrl-u:half-page-up" \
        --color="hl:yellow,hl+:yellow" <"$tmp" |
        awk "{print \$1}" | tr "\n" " ")

    rm -f "$tmp"
    echo "$ids"
}

tdone() {
    local project=$(_tmux_project)
    local filter=()
    [[ -n "$project" ]] && filter=("project:$project")

    local ids
    ids=$(_task_fzf_select "Complete (TAB=multi, ENTER=done): " "${filter[@]}")
    [[ -z "$ids" ]] && echo "Aborted" && return 1

    echo "→ task $ids done"
    task $ids done
}

tdel() {
    local project=$(_tmux_project)
    local filter=()
    [[ -n "$project" ]] && filter=("project:$project")

    local ids
    ids=$(_task_fzf_select "Delete (TAB=multi, ENTER=done): " "${filter[@]}")
    [[ -z "$ids" ]] && echo "Aborted" && return 1

    echo "→ task $ids delete"
    task $ids delete
}

TASK_FOCUS_FILE="$HOME/.task_focus"

_tfocus_get() {
    [[ -f "$TASK_FOCUS_FILE" ]] && cat "$TASK_FOCUS_FILE" || echo ""
}

_tfocus_set() { echo "$1" >"$TASK_FOCUS_FILE"; }
_tfocus_clear() { rm -f "$TASK_FOCUS_FILE"; }

# ── tstart ─────────────────────────────────────────────────────────────────
tstart() {
    local project=$(_tmux_project)
    local filter=()
    [[ -n "$project" ]] && filter=("project:$project")

    local tmp
    tmp=$(mktemp)
    task "${filter[@]}" status:pending export 2>/dev/null |
        python3 -c '
import json, sys
tasks = json.load(sys.stdin)
for t in sorted(tasks, key=lambda x: -x.get("urgency", 0)):
    tid  = t["id"]
    pri  = t.get("priority", " ")
    desc = t["description"][:55]
    tags = " ".join("+" + g for g in t.get("tags", []))
    print(f"{tid:4d}  [{pri}]  {desc:<55}  {tags}")
' >"$tmp"

    local id
    id=$(fzf --prompt="Start task: " \
        --layout=reverse --border \
        --bind="j:down,k:up" --no-input \
        <"$tmp" | awk '{print $1}')
    rm -f "$tmp"

    [[ -z "$id" ]] && echo "Aborted" && return 1

    task "$id" start && _tfocus_set "$id"
    echo "▶ focused: $id"
}

# ── tstop ──────────────────────────────────────────────────────────────────
tstop() {
    local tmp
    tmp=$(mktemp)
    task +ACTIVE export 2>/dev/null |
        python3 -c '
import json, sys
tasks = json.load(sys.stdin)
for t in tasks:
    tid  = t["id"]
    desc = t["description"][:55]
    tags = " ".join("+" + g for g in t.get("tags", []))
    print(f"{tid:4d}  [A]  {desc:<55}  {tags}")
' >"$tmp"

    [[ ! -s "$tmp" ]] && echo "No active tasks" && rm -f "$tmp" && return 1

    local ids
    ids=$(fzf --multi --prompt="Stop task (TAB=multi): " \
        --layout=reverse --border \
        --bind="j:down,k:up" --no-input \
        <"$tmp" | awk '{print $1}' | tr '\n' ' ')
    rm -f "$tmp"

    [[ -z "$ids" ]] && echo "Aborted" && return 1

    local focused
    focused=$(_tfocus_get)
    for id in $ids; do
        task "$id" stop
        [[ "$id" == "$focused" ]] && _tfocus_clear
    done
}

# ── tfocus ─────────────────────────────────────────────────────────────────
tfocus() {
    local focused
    focused=$(_tfocus_get)

    if [[ -z "$1" ]]; then
        # show current focus
        if [[ -n "$focused" ]]; then
            task "$focused" info 2>/dev/null | head -6
        else
            echo "No focused task. Run 'tstart' or 'tfocus set'."
        fi
        return
    fi

    if [[ "$1" == "set" ]]; then
        local project=$(_tmux_project)
        local filter=()
        [[ -n "$project" ]] && filter=("project:$project")

        local tmp
        tmp=$(mktemp)
        task "${filter[@]}" status:pending export 2>/dev/null |
            python3 -c '
import json, sys
tasks = json.load(sys.stdin)
for t in sorted(tasks, key=lambda x: -x.get("urgency", 0)):
    tid  = t["id"]
    pri  = t.get("priority", " ")
    desc = t["description"][:55]
    tags = " ".join("+" + g for g in t.get("tags", []))
    print(f"{tid:4d}  [{pri}]  {desc:<55}  {tags}")
' >"$tmp"

        local id
        id=$(fzf --prompt="Set focus: " \
            --layout=reverse --border \
            --bind="j:down,k:up" --no-input \
            <"$tmp" | awk '{print $1}')
        rm -f "$tmp"

        [[ -z "$id" ]] && echo "Aborted" && return 1
        _tfocus_set "$id"
        echo "◉ focused: $id"
        return
    fi

    if [[ "$1" == "clear" ]]; then
        _tfocus_clear && echo "Focus cleared"
        return
    fi

    echo "Usage: tfocus [set|clear]"
}

# ── tlog ───────────────────────────────────────────────────────────────────
tlog() {
    local project=$(_tmux_project)
    local filter=()
    [[ -n "$project" ]] && filter=("project:$project")

    local range="${1:-1w}" # default: last 1 week, pass "1m" for month etc.

    echo "── Completed (last $range) ──────────────────────"
    task "${filter[@]}" status:completed end.after:"$range ago" \
        rc.report.all.columns=id,end,tags,description \
        rc.report.all.labels=ID,Done,Tags,Description \
        rc.report.all.sort=end- \
        all 2>/dev/null
}

# ── tq — fast capture ──────────────────────────────────────────────────────
tq() {
    if [[ -z "$1" ]]; then
        tadd
        return
    fi

    local project=$(_tmux_project)
    local desc="$1"
    shift

    local -a args=(add "$desc")
    [[ -n "$project" ]] && args+=(project:"$project")
    (( $# > 0 )) && args+=("$@")

    task "${args[@]}"
}

TASK_PARK_STACK="$HOME/.task_park_stack"

# ── tpark — push focus to park stack ───────────────────────────────────────
tpark() {
    local note="${1:-}"
    local focused
    focused=$(_tfocus_get)

    if [[ -z "$focused" ]]; then
        echo "tpark: no active focus to park" >&2
        return 1
    fi

    local ts
    ts=$(date +%s)
    echo "${focused}:${ts}:${note}" >> "$TASK_PARK_STACK"
    _tfocus_clear

    local depth
    depth=$(wc -l < "$TASK_PARK_STACK" | tr -d ' ')
    echo "⏸ parked #${focused}. Stack depth: ${depth}"
}

# ── treturn — pop focus from park stack ────────────────────────────────────
treturn() {
    if [[ ! -f "$TASK_PARK_STACK" ]] || [[ ! -s "$TASK_PARK_STACK" ]]; then
        echo "treturn: park stack is empty" >&2
        return 1
    fi

    local entry
    entry=$(tail -1 "$TASK_PARK_STACK")

    local tmp
    tmp=$(mktemp)
    head -n -1 "$TASK_PARK_STACK" > "$tmp" && mv "$tmp" "$TASK_PARK_STACK" || { rm -f "$tmp"; return 1; }
    [[ ! -s "$TASK_PARK_STACK" ]] && rm -f "$TASK_PARK_STACK"

    local id ts note
    id=$(echo "$entry" | cut -d: -f1)
    ts=$(echo "$entry" | cut -d: -f2)
    note=$(echo "$entry" | cut -d: -f3-)

    _tfocus_set "$id" || { echo "treturn: failed to restore focus for #${id}" >&2; return 1; }

    local now elapsed
    now=$(date +%s)
    elapsed=$(( (now - ts) / 60 ))
    local age
    if [[ $elapsed -lt 60 ]]; then
        age="${elapsed}min ago"
    else
        age="$((elapsed / 60))h ago"
    fi

    local msg="▶ returned to #${id}"
    local desc
    desc=$(task "$id" _get description 2>/dev/null)
    [[ -n "$desc" ]] && msg+=" — ${desc}"
    msg+=" (parked ${age})"
    [[ -n "$note" ]] && msg+=" | \"${note}\""
    echo "$msg"
}

# ── twhere — show focus + park stack, read-only ────────────────────────────
twhere() {
    local project=$(_tmux_project)
    local focused
    focused=$(_tfocus_get)

    echo ""

    if [[ -n "$focused" ]]; then
        local desc pri
        desc=$(task "$focused" _get description 2>/dev/null)
        pri=$(task "$focused" _get priority 2>/dev/null)
        printf "◉ FOCUS:  #%-4s %-50s [%s]\n" "$focused" "$desc" "${pri:- }"
    else
        printf "◉ FOCUS:  (none)\n"
    fi

    if [[ -f "$TASK_PARK_STACK" ]] && [[ -s "$TASK_PARK_STACK" ]]; then
        local now
        now=$(date +%s)
        tac "$TASK_PARK_STACK" | while IFS= read -r entry; do
            local pid pts pdesc page pelapsed
            pid=$(echo "$entry" | cut -d: -f1)
            pts=$(echo "$entry" | cut -d: -f2)
            pdesc=$(task "$pid" _get description 2>/dev/null)
            pelapsed=$(( (now - pts) / 60 ))
            if [[ $pelapsed -lt 60 ]]; then
                page="${pelapsed}min ago"
            else
                page="$((pelapsed / 60))h ago"
            fi
            printf "⏸ PARKED: #%-4s %-50s (%s)\n" "$pid" "$pdesc" "$page"
        done
    fi

    echo ""

    local filter=()
    [[ -n "$project" ]] && filter=("project:$project")
    local recent
    recent=$(task "${filter[@]}" status:completed limit:3 \
        rc.report.all.columns=id,end,description \
        rc.report.all.labels=ID,Done,Description \
        rc.report.all.sort=end- \
        all 2>/dev/null | grep -E '^\s*[0-9]')

    if [[ -n "$recent" ]]; then
        echo "recent:"
        echo "$recent" | while IFS= read -r line; do
            printf "  ✓ %s\n" "$line"
        done
    fi
    echo ""
}

# ── task-dashboard — called by tmux popup (Alt+t) ──────────────────────────
task-dashboard() {
    set +xv
    # emulate -L zsh          # reset all inherited shell options (xtrace, verbose, etc.)
    # trap - DEBUG 2>/dev/null  # clear any DEBUG trap that echoes variable assignments
    local RST='\033[0m'   BOLD='\033[1m'    DIM='\033[2m'
    local RED='\033[1;31m' YLW='\033[1;33m' GRN='\033[32m'
    local CYN='\033[36m'  BCYN='\033[1;96m' WHT='\033[1;37m'
    local BLU='\033[34m'  MAG='\033[35m'    GRY='\033[2;37m'
    local DIV="${DIM}─────────────────────────────────────────────────────────────${RST}"

    while true; do
        clear
        local project=$(_tmux_project)
        local focused now_str
        focused=$(_tfocus_get)
        now_str=$(date '+%a %d %b  %H:%M')

        local park_depth=0
        [[ -f "$TASK_PARK_STACK" ]] && [[ -s "$TASK_PARK_STACK" ]] && \
            park_depth=$(wc -l < "$TASK_PARK_STACK" | tr -d ' ')

        # ── Header ──────────────────────────────────────────────────────────
        printf "\n"
        printf " ${WHT}◉  T A S K S${RST}   ${DIM}│${RST}   ${BCYN}${project:-all projects}${RST}   ${DIM}│   ${now_str}${RST}\n"
        printf " %b\n" "$DIV"

        # ── Focus ───────────────────────────────────────────────────────────
        printf "\n ${BCYN}◎  FOCUS${RST}  %b\n" "$DIV"
        if [[ -n "$focused" ]]; then
            local fdesc fpri ftags pri_badge
            fdesc=$(task "$focused" _get description 2>/dev/null)
            fpri=$(task "$focused" _get priority 2>/dev/null)
            ftags=$(task "$focused" export 2>/dev/null | python3 -c '
import json,sys
d=json.load(sys.stdin)
print(" ".join("+"+t for t in (d[0].get("tags") or [])) if d else "")
' 2>/dev/null)
            case "$fpri" in
                H) pri_badge="${RED}▲ H${RST}" ;;
                M) pri_badge="${YLW}▶ M${RST}" ;;
                L) pri_badge="${GRN}▼ L${RST}" ;;
                *) pri_badge="${DIM}  ·${RST}" ;;
            esac
            printf "   ${GRN}●${RST}  ${GRY}#%-4s${RST}  %b  ${WHT}%s${RST}\n" \
                "$focused" "$pri_badge" "$fdesc"
            [[ -n "$ftags" ]] && printf "          ${CYN}%s${RST}\n" "$ftags"
        else
            printf "   ${DIM}no active focus —${RST} press ${YLW}s${RST}${DIM} to start one${RST}\n"
        fi

        # ── Parked ──────────────────────────────────────────────────────────
        if [[ $park_depth -gt 0 ]]; then
            printf "\n ${BCYN}⏸  PARKED${RST}  %b\n" "$DIV"
            local now_ts
            now_ts=$(date +%s)
            tac "$TASK_PARK_STACK" | while IFS= read -r entry; do
                local pid pts pdesc pelapsed page
                pid=$(echo "$entry" | cut -d: -f1)
                pts=$(echo "$entry" | cut -d: -f2)
                pdesc=$(task "$pid" _get description 2>/dev/null)
                pelapsed=$(( (now_ts - pts) / 60 ))
                [[ $pelapsed -lt 60 ]] && page="${pelapsed}m" || page="$((pelapsed/60))h"
                printf "   ${MAG}⏸${RST}  ${GRY}#%-4s${RST}  ${WHT}%-50s${RST}  ${DIM}%s ago${RST}\n" \
                    "$pid" "$pdesc" "$page"
            done
        fi

        # ── Pending ─────────────────────────────────────────────────────────
        printf "\n ${BCYN}▤  PENDING${RST}  ${DIM}── ${RST}${CYN}${project:-all}${RST}  %b\n" "$DIV"
        local filter=()
        [[ -n "$project" ]] && filter=("project:$project")
        task "${filter[@]}" status:pending export 2>/dev/null | python3 -c '
import json, sys

RST  = "\033[0m";  DIM  = "\033[2m";   WHT  = "\033[1;37m"
RED  = "\033[1;31m"; YLW = "\033[1;33m"; GRN = "\033[32m"
CYN  = "\033[36m";   GRY = "\033[2;37m"

tasks = json.load(sys.stdin)
for t in sorted(tasks, key=lambda x: -x.get("urgency", 0))[:8]:
    tid  = t["id"]
    pri  = t.get("priority", "")
    desc = t["description"][:50]
    tags = " ".join("+" + g for g in t.get("tags", []))
    badge = (f"{RED}▲ H{RST}" if pri == "H" else
             f"{YLW}▶ M{RST}" if pri == "M" else
             f"{GRN}▼ L{RST}" if pri == "L" else
             f"{DIM}  ·{RST}")
    print(f"   {DIM}○{RST}  {GRY}#{tid:<4}{RST}  {badge}  {WHT}{desc:<50}{RST}  {CYN}{tags}{RST}")
' 2>/dev/null

        # ── Actions ─────────────────────────────────────────────────────────
        printf "\n %b\n" "$DIV"
        printf "  ${YLW}[a]${RST}${DIM}dd  ${RST}${YLW}[d]${RST}${DIM}one  ${RST}${YLW}[s]${RST}${DIM}tart  ${RST}${YLW}[S]${RST}${DIM}top  ${RST}${YLW}[p]${RST}${DIM}ark  ${RST}${YLW}[r]${RST}${DIM}eturn  ${RST}${YLW}[w]${RST}${DIM}here  ${RST}${YLW}[q]${RST}${DIM}uit${RST}\n\n"

        local key
        read -sk1 key
        case "$key" in
            a) tadd;    echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            d) tdone;   echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            s) tstart;  echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            S) tstop;   echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            p) tpark;   echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            r) treturn; echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            w) twhere;  echo; printf "${DIM}↵ continue${RST}"; read -sk1 ;;
            q|$'\e') break ;;
        esac
    done
    clear
}
