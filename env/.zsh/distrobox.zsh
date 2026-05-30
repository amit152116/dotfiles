# Distrobox integration: shells always open on the HOST. Switch into a repo's
# container on demand.
#
# Mark a project by dropping a `.distrobox` file at its repo root containing the
# container name, e.g.:   echo "ros-dev" > .distrobox     (or run `dbox-mark`)
#
# Switching, all in the same pane:
#   - host -> container : run `box`     (reads the marker, or `box <name>`)
#   - container -> host : type `exit`   (host shell resumes underneath)
#   - one-off native cmd from inside the container: `onhost <cmd>`

# --- Detect whether this shell is already inside a container ----------------
# Markers: docker (/.dockerenv), podman/distrobox (/run/.containerenv, $container),
# toolbox (/run/.toolboxenv), distrobox ($CONTAINER_ID). Matches omz.zsh.
if [[ -f /.dockerenv || -f /run/.containerenv || -f /run/.toolboxenv ||
    -n "$container" || -n "$CONTAINER_ID" ]]; then
    export IN_CONTAINER=1
else
    export IN_CONTAINER=0
fi

# Read the container name from the nearest .distrobox marker, walking up.
_distrobox_marker_name() {
    local dir="$PWD"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/.distrobox" ]]; then
            grep -m1 -vE '^[[:space:]]*(#|$)' "$dir/.distrobox" 2>/dev/null | tr -d '[:space:]'
            return 0
        fi
        dir="${dir:h}"
    done
    return 1
}

# --- Enter the project's container (no exec: `exit` returns to host) ---------
# Usage: box [name]   (name defaults to the repo's .distrobox marker)
box() {
    ((IN_CONTAINER)) && {
        print -P "%F{yellow}already inside container%f"
        return
    }
    command -v distrobox >/dev/null 2>&1 || {
        print -P "%F{red}distrobox not installed%f"
        return 1
    }
    local name="${1:-$(_distrobox_marker_name)}"
    [[ -n "$name" ]] || {
        print -P "%F{red}no .distrobox marker here; usage: box <name>%f"
        return 1
    }
    if ! distrobox list 2>/dev/null | grep -qw "$name"; then
        print -P "%F{red}container '$name' not found — create it with 'distrobox create'%f"
        return 1
    fi
    distrobox enter "$name"
}

# --- Inside the container: run things on the host ---------------------------
# `onhost <cmd>` runs a one-off command on the host and returns to the box.
# `onhost` with no args drops back to the host shell *underneath* instead of
# stacking a new one: `box` entered without exec, so exiting the container
# returns to the original host shell (keeps the shell stack at depth 1).
if ((IN_CONTAINER)) && command -v distrobox-host-exec >/dev/null 2>&1; then
    onhost() {
        if (($#)); then
            distrobox-host-exec "$@"
        else
            exit
        fi
    }
fi

# --- Mark a repo to auto-detect a (already-created) container ----------------
# Usage: dbox-mark <name> [repo-path]   (create the box yourself first)
dbox-mark() {
    local name="$1" repo="$2"
    if [[ -z "$name" ]]; then
        print -P "%F{red}usage: dbox-mark <name> [repo-path]%f"
        return 1
    fi
    [[ -n "$repo" ]] || repo="$(git rev-parse --show-toplevel 2>/dev/null || print -r -- "$PWD")"
    if command -v distrobox >/dev/null 2>&1 &&
        ! distrobox list 2>/dev/null | grep -qw "$name"; then
        print -P "%F{yellow}note: container '$name' not found — create it with 'distrobox create'%f"
    fi
    print -r -- "$name" >"$repo/.distrobox" &&
        print -P "%F{green}marked %B$repo%b → container '$name'%f"
}
