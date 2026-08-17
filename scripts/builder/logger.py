"""Terminal color logging."""

from __future__ import annotations

import sys

_C = {
    "RED": "\033[0;31m",
    "GREEN": "\033[0;32m",
    "YELLOW": "\033[1;33m",
    "BLUE": "\033[0;34m",
    "CYAN": "\033[0;36m",
    "BOLD": "\033[1m",
    "DIM": "\033[2m",
    "NC": "\033[0m",
}


def _log(level: str, color: str, msg: str) -> None:
    print(
        f"{_C['BOLD']}{_C[color]}[{level}]{_C['NC']} {_C[color]}{msg}{_C['NC']}",
        file=sys.stderr,
    )


def info(msg: str) -> None:
    _log("INFO", "BLUE", msg)


def success(msg: str) -> None:
    _log("SUCCESS", "GREEN", msg)


def warning(msg: str) -> None:
    _log("WARNING", "YELLOW", msg)


def error(msg: str) -> None:
    _log("ERROR", "RED", msg)


def step(msg: str) -> None:
    print(
        f"{_C['BOLD']}{_C['BLUE']}▶{_C['NC']} {_C['BOLD']}{msg}{_C['NC']}",
        file=sys.stderr,
    )


def section(msg: str) -> None:
    print(
        f"\n{_C['BOLD']}{_C['CYAN']}━━━━━━ {msg} ━━━━━━{_C['NC']}",
        file=sys.stderr,
    )


def sep() -> None:
    print(f"{_C['DIM']}{'─' * 52}{_C['NC']}", file=sys.stderr)


def plain(msg: str) -> None:
    # stderr like every other log fn, else it interleaves out of order with them
    print(msg, file=sys.stderr)


def bold(text: str) -> str:
    return f"{_C['BOLD']}{text}{_C['NC']}"


def cyan(text: str) -> str:
    return f"{_C['CYAN']}{text}{_C['NC']}"


def green(text: str) -> str:
    return f"{_C['GREEN']}{text}{_C['NC']}"
