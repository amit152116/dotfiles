#!/usr/bin/env zsh

# persist only successful cmds to $HISTFILE, newest occurrence wins on repeat.
# in-memory list stays undeduped (zinit HIST_IGNORE_DUPS is consecutive-only) so up-arrow keeps full recall.
# zshaddhistory returns 2 -> in-memory yes, file no; precmd/zshexit append manually once exit code is known.
# file is append-only per-session (dupes included); startup compaction collapses to newest-occurrence-only,
# so a rerun sorts to file bottom instead of staying stale at first-seen position.
# exit 130 (SIGINT) kept too: deliberate abort, still worth recalling.
# needs INC_APPEND_HISTORY not SHARE_HISTORY: SHARE re-reads file every prompt, would re-import our append.

zmodload zsh/datetime # $EPOCHSECONDS for the EXTENDED_HISTORY stamp
autoload -Uz add-zsh-hook

typeset -g _HIST_PENDING=""

# WARN(history): concurrent startups (many tmux panes) can race the compact -> a dropped line, no corruption.
_hist_compact() {
    [[ -s "$HISTFILE" ]] || return
    local tmp
    tmp=$(mktemp "${HISTFILE}.XXXXXX") || return
    awk '
        /^: [0-9]+:[0-9]+;/ {
            if (key != "") { n++; rec[n] = block; k[n] = key }
            block = $0
            key = $0
            sub(/^: [0-9]+:[0-9]+;/, "", key)
            sub(/\\$/, "", key)
            next
        }
        {
            block = block "\n" $0
            line = $0
            sub(/\\$/, "", line)
            key = key "\n" line
        }
        END {
            if (key != "") { n++; rec[n] = block; k[n] = key }
            for (i = 1; i <= n; i++) last[k[i]] = i
            for (i = 1; i <= n; i++) if (last[k[i]] == i) print rec[i]
        }
    ' "$HISTFILE" >"$tmp" && command mv "$tmp" "$HISTFILE"
}
_hist_compact

# keep in memory incl. failures; defer file write to precmd once exit code known
zshaddhistory() {
    _HIST_PENDING=""
    [[ "$1" == ' '* ]] && return 1 # honor HIST_IGNORE_SPACE
    local line="${1%%$'\n'}"
    [[ -z "${line// /}" ]] && return 1 # skip empty/whitespace-only
    _HIST_PENDING="$line"
    return 2 # in-memory yes, file no
}

# append to $HISTFILE on success; next startup's _hist_compact collapses dupes
_hist_persist() {
    local ret=$?
    if [[ ($ret -eq 0 || $ret -eq 130) && -n "$_HIST_PENDING" ]]; then
        # backslash-continuation for embedded newlines: multi-line cmd reloads as one event, not several
        local nl=$'\n'
        local cmd=${_HIST_PENDING//$nl/\\$nl}
        print -r -- ": ${EPOCHSECONDS}:0;${cmd}" >>"$HISTFILE"
    fi
    _HIST_PENDING=""
}
add-zsh-hook precmd _hist_persist
add-zsh-hook zshexit _hist_persist # persist the last command (no precmd follows exit)

# run first so $? is the user command's exit before other precmd hooks mutate it
precmd_functions=(_hist_persist ${precmd_functions:#_hist_persist})
