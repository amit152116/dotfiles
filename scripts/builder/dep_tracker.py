"""Detect dependency packages that need rebuilding."""

from __future__ import annotations

import subprocess
from pathlib import Path


def get_stale_deps(packages: list[str], force: bool = False) -> list[str]:
    """Return dep packages missing from install/ or with changed source files.

    force=True: return all deps unconditionally (bypass mtime check).
    """
    if not packages:
        return []

    try:
        result = subprocess.run(
            ["colcon", "list", "--packages-up-to", *packages, "--names-only"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []

    all_deps = [ln.strip() for ln in result.stdout.splitlines() if ln.strip()]
    target_set = set(packages)
    stale: list[str] = []

    for dep in all_deps:
        if dep in target_set:
            continue

        install_dir = Path("install") / dep

        if not install_dir.exists():
            stale.append(dep)
            continue

        if force:
            stale.append(dep)
            continue

        src_path = _get_src_path(dep)
        if not src_path:
            continue

        # Any source file newer than install dir → stale
        found = subprocess.run(
            ["find", src_path, "-newer", str(install_dir), "-type", "f"],
            capture_output=True,
            text=True,
        )
        if found.returncode == 0 and found.stdout.strip():
            stale.append(dep)

    return stale


def _get_src_path(pkg: str) -> str:
    try:
        result = subprocess.run(
            ["colcon", "list", "--packages-select", pkg, "--paths-only"],
            capture_output=True,
            text=True,
            check=True,
        )
        path = result.stdout.strip()
        return path if path and Path(path).is_dir() else ""
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""
