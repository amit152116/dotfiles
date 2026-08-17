"""Build tool entry point. Invoke via the `build` wrapper, which sources ROS first."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from . import logger as log
from .backends.registry import get_backend
from .config import parse_args, warn_unused_flags
from .detect import detect_backend


def main() -> None:
    cfg, action = parse_args()

    cfg.backend, root = detect_backend(cfg.backend)
    warn_unused_flags(cfg)

    if Path.cwd().resolve() != root:
        log.info(f"Project root: {root}")
        os.chdir(root)

    backend = get_backend(cfg.backend)

    actions = {
        "clean": backend.clean,
        "clean-all": backend.clean_all,
        "stats": backend.stats,
        "lint": backend.lint,
        "list-targets": backend.list_targets,
        "build": backend.build,
    }
    actions[action](cfg)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(130)
    except ValueError as e:  # build-type normalization
        log.error(str(e))
        sys.exit(2)
