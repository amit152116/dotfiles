"""python venv setup for colcon packages.

TODO(builder): colcon-coupled (uses `colcon list --paths-only`), so the cmake
backend skips venv setup entirely; generalize via a plain path scan if needed.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from . import logger as log


def setup_python_env(packages: list[str], venv_dir: str) -> None:
    # stamp mtime check skips reinstall when requirements.txt unchanged, avoids churn every build
    if not packages:
        return

    if shutil.which("uv") is None:
        log.error(
            "uv not found. Install it: https://docs.astral.sh/uv/getting-started/installation/"
        )
        return

    try:
        result = subprocess.run(
            ["colcon", "list", "--packages-select", *packages, "--paths-only"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        log.warning("Could not resolve package paths for venv setup")
        return

    src_paths = [
        Path(p)
        for p in result.stdout.splitlines()
        if p.strip() and Path(p.strip()).is_dir()
    ]
    if not src_paths:
        return

    venv = Path(venv_dir)
    venv_python = venv / "bin" / "python"
    stamps_dir = venv / ".stamps"

    venv_created = _ensure_venv(venv)

    stamps_dir.mkdir(exist_ok=True)

    pending: list[tuple[Path, Path]] = []  # (lock, stamp) pairs needing install

    for src in src_paths:
        _make_scripts_executable(src)

        req = src / "requirements.txt"
        if not req.exists():
            continue

        stamp = stamps_dir / src.name
        if (
            not venv_created
            and stamp.exists()
            and stamp.stat().st_mtime >= req.stat().st_mtime
        ):
            continue  # requirements unchanged since last install

        lock = src / "requirements.lock.txt"
        log.info(f"Compiling lockfile for: {src.name}")
        subprocess.run(
            ["uv", "pip", "compile", str(req), "-o", str(lock)],
            check=True,
        )
        pending.append((lock, stamp))

    if not pending:
        return

    log.info(
        f"Installing deps for: {', '.join(lock.parent.name for lock, _ in pending)}"
    )
    install_cmd = ["uv", "pip", "install", "--python", str(venv_python)]
    for lock, _ in pending:
        install_cmd += ["-r", str(lock)]
    subprocess.run(install_cmd, check=True)

    for _, stamp in pending:
        stamp.touch()


def _ensure_venv(venv: Path) -> bool:
    # return value forces reinstall on fresh venv, bypassing stamp check
    if venv.exists():
        return False

    log.warning(f"Creating virtual environment at {venv}")
    subprocess.run(
        ["uv", "venv", str(venv), "--system-site-packages"],
        check=True,
    )
    log.success("Virtual environment created")
    return True


def _make_scripts_executable(src: Path) -> None:
    for script in src.glob("scripts/*.py"):
        mode = script.stat().st_mode
        if not (mode & 0o111):
            script.chmod(mode | 0o111)
            log.success(f"Made executable: {script}")
