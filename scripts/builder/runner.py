"""Command execution with logging and dry-run support."""

from __future__ import annotations

import shlex
import subprocess

from . import logger as log


def run(cmd: list[str], dry_run: bool = False, quiet: bool = False) -> int:
    """Echo then run cmd, streaming its output. Returns exit code (0 on dry-run)."""
    if not quiet:
        log.sep()
        log.step(f"Command: {log.cyan(shlex.join(cmd))}")
        log.sep()
    if dry_run:
        return 0
    return subprocess.run(cmd).returncode


def capture(cmd: list[str]) -> tuple[int, str]:
    """Run cmd capturing stdout; used for queries, never for builds."""
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout
