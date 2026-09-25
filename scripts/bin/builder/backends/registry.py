"""Backend registry: each module exposes build/clean/clean_all/stats/lint/list_targets."""

from __future__ import annotations

from types import ModuleType

from . import cmake, ros

_BACKENDS: dict[str, ModuleType] = {"ros": ros, "cmake": cmake}


def get_backend(name: str) -> ModuleType:
    return _BACKENDS[name]
