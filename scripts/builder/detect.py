"""Workspace backend detection: ros (colcon) vs cmake."""

from __future__ import annotations

from pathlib import Path

from . import logger as log

# colcon discovers packages recursively from its base path, so a workspace is any dir
# with package.xml children: src/ layout, flat layout (pkgs at top level), or grouped.
# Ordered passes, not one glob set: src/ layout must win, else `cd src/pkg` resolves
# the root to src/ itself (which */package.xml also matches).
_SRC_GLOBS = ("src/*/package.xml", "src/*/*/package.xml")
_FLAT_GLOBS = ("*/package.xml",)
_DEEP_GLOBS = ("*/*/package.xml",)


def _ancestors(start: Path) -> list[Path]:
    """cwd and parents, stopping below $HOME — $HOME is never a workspace root."""
    cur = start.resolve()
    home = Path.home().resolve()
    out: list[Path] = []
    for d in (cur, *cur.parents):
        if d == home:
            break
        out.append(d)
    return out


def _first_match(dirs: list[Path], globs: tuple[str, ...]) -> Path | None:
    for d in dirs:
        if any(next(d.glob(g), None) for g in globs):
            return d
    return None


def find_ros_workspace(start: Path | None = None) -> Path | None:
    """Nearest ancestor (self included) holding ROS packages.

    Workspace-level markers win over the cwd's own package.xml, so building from
    inside a package still targets the whole workspace.
    """
    cur = (start or Path.cwd()).resolve()
    if cur == Path.home().resolve():
        return None

    dirs = _ancestors(cur)

    hit = _first_match(dirs, _SRC_GLOBS)
    if hit:
        return hit

    # flat layout (packages at repo top level): only a repo root or cwd itself may
    # claim it, else a parent dir full of unrelated repos looks like a workspace
    hit = _first_match(dirs, _FLAT_GLOBS)
    if hit and (hit == cur or (hit / ".git").exists()):
        return hit

    # grouped layouts on cwd only: a depth-2 glob walking up finds bogus roots
    if _first_match([cur], _DEEP_GLOBS) or (cur / "package.xml").is_file():
        return cur
    return None


def find_cmake_root(start: Path | None = None) -> Path | None:
    cur = (start or Path.cwd()).resolve()
    for d in (cur, *cur.parents):
        if (d / "CMakeLists.txt").is_file():
            return d
    return None


def detect_backend(override: str = "") -> tuple[str, Path]:
    """Return (backend, project root). Root is chdir'd into before any action.

    A ROS package dir holds both package.xml and CMakeLists.txt, so the ROS
    workspace check must win and must walk up to the colcon root.
    """
    if override == "ros":
        ws = find_ros_workspace()
        if not ws:
            log.error("--backend ros but no package.xml found here or in any parent")
            raise SystemExit(1)
        return "ros", ws

    if override == "cmake":
        root = find_cmake_root()
        if not root:
            log.error("--backend cmake but no CMakeLists.txt found")
            raise SystemExit(1)
        return "cmake", root

    ws = find_ros_workspace()
    if ws:
        return "ros", ws

    root = find_cmake_root()
    if root:
        return "cmake", root

    log.error("No buildable project here: need a package.xml (ROS) or CMakeLists.txt")
    raise SystemExit(1)
