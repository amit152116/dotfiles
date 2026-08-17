"""Build configuration: dataclass, arg parsing, persistence."""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

from . import logger as log

_CONFIG_FILE = Path(".build.json")

_CANONICAL_BUILD_TYPES = ["Debug", "Release", "RelWithDebInfo", "MinSizeRel"]

# Fixed aliases take priority over fuzzy matching
_BUILD_TYPE_ALIASES: dict[str, str] = {
    "debug": "Debug",
    "release": "Release",
    "rel": "Release",
    "relwithdebinfo": "RelWithDebInfo",
    "rwdi": "RelWithDebInfo",
    "minsizerel": "MinSizeRel",
    "minsize": "MinSizeRel",
}

_ROS_ONLY_FLAGS = ("packages", "force_deps")
_CMAKE_ONLY_FLAGS = ("targets", "compiler", "install", "lint")


def _default_jobs() -> int:
    nproc = os.cpu_count() or 4
    return max(4, nproc // 2)


def _is_subsequence(pattern: str, text: str) -> bool:
    it = iter(text.lower())
    return all(c in it for c in pattern)


def normalize_build_type(value: str) -> str:
    key = value.lower()

    # 1. Fixed aliases (exact, case-insensitive)
    if key in _BUILD_TYPE_ALIASES:
        return _BUILD_TYPE_ALIASES[key]

    # 2. Prefix match
    prefix = [c for c in _CANONICAL_BUILD_TYPES if c.lower().startswith(key)]
    if len(prefix) == 1:
        return prefix[0]

    # 3. Subsequence match (fzf-style: characters must appear in order)
    subseq = [c for c in _CANONICAL_BUILD_TYPES if _is_subsequence(key, c)]
    if len(subseq) == 1:
        return subseq[0]

    candidates = prefix or subseq or _CANONICAL_BUILD_TYPES
    if len(candidates) > 1:
        raise ValueError(f"Ambiguous build type '{value}'. Matches: {candidates}")
    raise ValueError(f"Unknown build type '{value}'. Valid: {_CANONICAL_BUILD_TYPES}")


@dataclass
class BuildConfig:
    backend: str = ""  # resolved by detect.detect_backend()
    build_type: str = "Debug"
    build_type_explicit: bool = False
    packages: list[str] = field(default_factory=list)  # ros
    targets: list[str] = field(default_factory=list)  # cmake
    build_dir: str = "build"
    sanitizer: str = ""
    verbose: bool = False
    dry_run: bool = False
    ccache: bool = False
    compiler: str = ""
    install_prefix: str = ""
    install: bool = False
    coverage: bool = False
    benchmark: bool = False
    native: bool = False
    run_tests: bool = False
    lint: bool = False
    venv_dir: str = ".venv"
    cmake_generator: str = ""
    extra_cmake_args: list[str] = field(default_factory=list)
    extra_cxx_flags: list[str] = field(default_factory=list)
    parallel_jobs: int = field(default_factory=_default_jobs)
    force_deps: bool = False


def load_prev_config() -> dict:
    if _CONFIG_FILE.exists():
        try:
            return json.loads(_CONFIG_FILE.read_text())
        except (json.JSONDecodeError, OSError):
            pass
    return {}


def _write(data: dict) -> None:
    _CONFIG_FILE.write_text(json.dumps(data, indent=2))


def save_config(cfg: BuildConfig) -> None:
    prev = load_prev_config()
    prev["last"] = asdict(cfg)
    _write(prev)


def save_pkg_configs(
    packages: list[str],
    build_type: str,
    sanitizer: str = "",
    coverage: bool = False,
) -> None:
    """Record per-package build config so any field change triggers a clean rebuild."""
    prev = load_prev_config()
    pkgs: dict = prev.get("pkgs", {})
    for pkg in packages:
        pkgs[pkg] = {
            "build_type": build_type,
            "sanitizer": sanitizer,
            "coverage": coverage,
        }
    prev["pkgs"] = pkgs
    _write(prev)


def cmake_signature(cfg: BuildConfig) -> dict:
    """Fields whose change invalidates an existing cmake cache."""
    return {
        "build_type": cfg.build_type,
        "sanitizer": cfg.sanitizer,
        "coverage": cfg.coverage,
        "compiler": cfg.compiler,
        "generator": cfg.cmake_generator,
    }


def load_cmake_signature() -> dict:
    return load_prev_config().get("cmake", {})


def save_cmake_signature(cfg: BuildConfig) -> None:
    prev = load_prev_config()
    prev["cmake"] = cmake_signature(cfg)
    _write(prev)


def _split_list(value: str) -> list[str]:
    return [x.strip() for x in value.replace(",", " ").split() if x.strip()]


def parse_args() -> tuple[BuildConfig, str]:
    """Returns (config, action).

    action: "build" | "clean" | "clean-all" | "stats" | "lint" | "list-targets"
    """
    import argparse

    p = argparse.ArgumentParser(
        prog="build",
        description="Unified build tool: ROS 2 (colcon) and plain CMake projects.\n"
        "Backend auto-detected: src/*/package.xml -> ros, else CMakeLists.txt -> cmake.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--backend",
        choices=["ros", "cmake"],
        default="",
        help="Force backend instead of auto-detecting",
    )
    p.add_argument(
        "-b",
        "--build",
        dest="build_type",
        default=None,
        metavar="TYPE",
        help="Build type. Fuzzy-matched:\n"
        "  Debug, Release, RelWithDebInfo, MinSizeRel\n"
        "  Shortcuts: d, rele, relw, rwdi, m, msr, ...",
    )
    p.add_argument(
        "-s",
        "--sanitizer",
        choices=["asan", "msan", "tsan", "ubsan"],
        default="",
        metavar="TYPE",
        help="Enable sanitizer: asan, msan, tsan, ubsan (forces Debug)",
    )
    p.add_argument(
        "-p",
        "--pkg",
        dest="packages",
        default="",
        metavar="PKGS",
        help="[ros] Packages to build, comma or space separated",
    )
    p.add_argument(
        "-t",
        "--target",
        dest="targets",
        default="",
        metavar="TGTS",
        help="[cmake] Targets to build, comma or space separated",
    )
    p.add_argument(
        "-d",
        "--build-dir",
        default="build",
        metavar="DIR",
        help="[cmake] Build directory (default: build)",
    )
    p.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=0,
        metavar="N",
        help=f"Parallel jobs (default: nproc/2, min 4 = {_default_jobs()})",
    )
    p.add_argument(
        "-i",
        "--install-prefix",
        default="",
        metavar="PATH",
        help="CMAKE_INSTALL_PREFIX. A prefix, not an action -- see --install",
    )
    p.add_argument(
        "-g",
        "--generator",
        default="",
        metavar="TYPE",
        help="CMake generator: ninja or make (default: ninja if available)",
    )
    p.add_argument(
        "--compiler",
        choices=["gcc", "clang"],
        default="",
        help="[cmake] Compiler toolchain (default: gcc)",
    )
    p.add_argument(
        "--cmake",
        default="",
        metavar="FLAGS",
        help='Extra CMake flags (e.g. --cmake="-DFOO=ON -DBAR=OFF")',
    )
    p.add_argument(
        "--cxx",
        default="",
        metavar="FLAGS",
        help="Extra C++ compiler flags",
    )
    p.add_argument(
        "--ccache",
        action="store_true",
        help="Enable ccache for faster incremental builds",
    )
    p.add_argument(
        "--coverage", action="store_true", help="Enable code coverage (forces Debug)"
    )
    p.add_argument(
        "--bench",
        action="store_true",
        help="Benchmark build: Release + LTO.\n"
        "  [ros] adds NodeMixin comms metrics (METRICS_ON)\n"
        "  [cmake] adds -ffast-math",
    )
    p.add_argument(
        "--native",
        action="store_true",
        help="Tune for the build machine's CPU (-march=native).\n"
        "Binary is NOT portable to other CPUs; build on the deploy target.",
    )
    p.add_argument(
        "--test",
        action="store_true",
        help="Run tests.\n"
        "  [ros] alone: test only. With -b: no-op (tests always compiled)\n"
        "  [cmake] build then ctest (-DBUILD_TESTS=ON, ignores --target)",
    )
    p.add_argument(
        "--install",
        action="store_true",
        help="[cmake] Run cmake --install after a successful build",
    )
    p.add_argument(
        "--lint",
        action="store_true",
        help="[cmake] clang-tidy + cppcheck after build\n  alone: lint only (needs an existing build dir)",
    )
    p.add_argument(
        "-f",
        "--force-deps",
        action="store_true",
        help="[ros] Force rebuild all dependency packages (bypass mtime check)",
    )
    p.add_argument("-v", "--verbose", action="store_true", help="Verbose build output")
    p.add_argument(
        "-n", "--dry-run", action="store_true", help="Print commands without running"
    )
    p.add_argument(
        "--clean",
        action="store_true",
        help="Clean build/install/log for selected packages (or all)",
    )
    p.add_argument(
        "--clean-all",
        action="store_true",
        help="Clean everything: build, install, log, .ccache, coverage",
    )
    p.add_argument(
        "--stats", action="store_true", help="Show build directory statistics"
    )
    p.add_argument(
        "--list-targets",
        action="store_true",
        help="[cmake] List available build targets",
    )

    args = p.parse_args()

    if args.clean_all:
        action = "clean-all"
    elif args.clean:
        action = "clean"
    elif args.stats:
        action = "stats"
    elif args.list_targets:
        action = "list-targets"
    elif args.lint and args.build_type is None and not args.test:
        action = "lint"
    else:
        action = "build"

    build_type = normalize_build_type(args.build_type) if args.build_type else "Debug"

    cfg = BuildConfig(
        backend=args.backend,
        build_type=build_type,
        build_type_explicit=args.build_type is not None,
        packages=_split_list(args.packages),
        targets=_split_list(args.targets),
        build_dir=args.build_dir,
        sanitizer=args.sanitizer,
        verbose=args.verbose,
        dry_run=args.dry_run,
        ccache=args.ccache,
        compiler=args.compiler,
        install_prefix=args.install_prefix,
        install=args.install,
        coverage=args.coverage,
        benchmark=args.bench,
        native=args.native,
        run_tests=args.test,
        lint=args.lint,
        cmake_generator=args.generator,
        extra_cmake_args=args.cmake.split() if args.cmake else [],
        extra_cxx_flags=args.cxx.split() if args.cxx else [],
        parallel_jobs=args.jobs if args.jobs > 0 else _default_jobs(),
        force_deps=args.force_deps,
    )

    if cfg.benchmark:
        cfg.build_type = "Release"
        cfg.build_type_explicit = True
    elif cfg.sanitizer or cfg.coverage:
        # instrumentation needs unoptimized frames; -b is overridden, so say so
        if cfg.build_type_explicit and cfg.build_type != "Debug":
            log.warning(
                f"{cfg.sanitizer or 'coverage'} forces Debug (requested {cfg.build_type})"
            )
        cfg.build_type = "Debug"
        cfg.build_type_explicit = True

    return cfg, action


def warn_unused_flags(cfg: BuildConfig) -> None:
    """Warn about options the resolved backend ignores."""
    ignored = _ROS_ONLY_FLAGS if cfg.backend == "cmake" else _CMAKE_ONLY_FLAGS
    dead = [f for f in ignored if getattr(cfg, f) not in ("", [], False)]
    if dead:
        log.warning(f"{cfg.backend} backend ignores: {', '.join(dead)}")
