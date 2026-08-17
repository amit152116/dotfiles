# distrobox integration: shells always open on HOST, switch into a repo's container on demand.
# mark a project: `echo "ros-dev" > .distrobox` at repo root (or `dbox-mark`).
# host->container: `box` (reads marker, or `box <name>`); container->host: `exit`; one-off host cmd from inside: `onhost <cmd>`

# markers: docker (/.dockerenv), podman/distrobox (/run/.containerenv, $container), toolbox (/run/.toolboxenv), distrobox ($CONTAINER_ID) — matches zinit.zsh
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

# no exec: `exit` returns to host. box [name] defaults to the repo's .distrobox marker
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

# onhost <cmd>: one-off host cmd, returns to box. onhost (no args): exit container -
# box entered without exec, so this drops to the original host shell, not a stacked one
if ((IN_CONTAINER)) && command -v distrobox-host-exec >/dev/null 2>&1; then
    onhost() {
        if (($#)); then
            distrobox-host-exec "$@"
        else
            exit
        fi
    }
fi

# dbox-mark <name> [repo-path]: marks repo for auto-detect; create the box yourself first
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
