"""Construct cmake and colcon flag lists from a BuildConfig."""

from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from . import logger as log

if TYPE_CHECKING:
    from .config import BuildConfig

_SAN_CFG: dict[str, dict[str, str]] = {
    "asan": {
        "linker": "-fsanitize=address,undefined,leak",
        "cxx": "-fsanitize=address,undefined,leak -fno-omit-frame-pointer -fno-optimize-sibling-calls",
        "env": "new_delete_type_mismatch=0:detect_leaks=1:strict_init_order=1:check_initialization_order=1:symbolize=1:verbosity=1",
    },
    "msan": {
        "linker": "-fsanitize=memory",
        "cxx": "-fsanitize=memory -fno-omit-frame-pointer -fno-optimize-sibling-calls",
        "env": "verbosity=1",
    },
    "tsan": {
        "linker": "-fsanitize=thread -pie",
        "cxx": "-fsanitize=thread -fPIE",
        "env": "report_signal_unsafe=0:history_size=7:second_deadlock_stack=1:verbosity=1",
    },
    "ubsan": {
        "linker": "-fsanitize=undefined",
        "cxx": "-fsanitize=undefined -fno-omit-frame-pointer",
        "env": "print_stacktrace=1:halt_on_error=0:verbosity=1",
    },
}

_BUILD_CXX_FLAGS: dict[str, list[str]] = {
    "Release": ["-O3", "-DNDEBUG"],
    "Debug": ["-O0", "-g3"],
    "RelWithDebInfo": ["-O2", "-g3", "-DNDEBUG"],
    "MinSizeRel": ["-Os", "-DNDEBUG"],
}

# NOTE: cmake-only — -Wpedantic drowns third-party ROS package builds in noise
_CMAKE_WARN_FLAGS = ["-Wall", "-Wextra", "-Wpedantic"]

_BUILD_COLCON_FLAGS: dict[str, list[str]] = {
    "Debug": ["--symlink-install", "--continue-on-error"],
    "RelWithDebInfo": ["--symlink-install", "--continue-on-error"],
}

_GENERATORS: dict[str, str] = {
    "ninja": "Ninja",
    "make": "Unix Makefiles",
    "unix makefiles": "Unix Makefiles",
    "makefiles": "Unix Makefiles",
    "xcode": "Xcode",
}


@dataclass
class BuildArgs:
    cmake_flags: list[str] = field(default_factory=list)
    colcon_flags: list[str] = field(default_factory=list)
    generator: str = ""


def _linker_flags(args: BuildArgs, flags: str) -> None:
    # ROS packages are mostly shared libs: EXE-only instrumentation misses them
    args.cmake_flags += [
        f"-DCMAKE_EXE_LINKER_FLAGS={flags}",
        f"-DCMAKE_SHARED_LINKER_FLAGS={flags}",
    ]


def make_build_args(
    cfg: BuildConfig,
    build_type: str | None = None,
    for_deps: bool = False,
) -> BuildArgs:
    """Build cmake/colcon flag lists.

    build_type: override cfg.build_type (ros dep builds use "Release")
    for_deps: skip sanitizer/coverage/user CXX extras
    """
    bt = build_type or cfg.build_type
    is_cmake = cfg.backend == "cmake"
    args = BuildArgs()
    cxx_flags: list[str] = ["-fdiagnostics-color=always"]

    args.generator = resolve_generator(cfg.cmake_generator)
    if cfg.backend == "ros":
        args.cmake_flags += [f"-G{args.generator}"]

    cc, cxx = ("clang", "clang++") if cfg.compiler == "clang" else ("gcc", "g++")
    if cfg.compiler and not shutil.which(cc):
        log.error(f"{cfg.compiler} not found in PATH")
        raise SystemExit(1)
    args.cmake_flags += [f"-DCMAKE_C_COMPILER={cc}", f"-DCMAKE_CXX_COMPILER={cxx}"]

    if cfg.ccache:
        if shutil.which("ccache"):
            args.cmake_flags += [
                "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
                "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
            ]
        else:
            log.warning("ccache not found. Install: sudo apt install ccache")

    if cfg.install_prefix:
        args.cmake_flags += [f"-DCMAKE_INSTALL_PREFIX={cfg.install_prefix}"]

    if not for_deps:
        if cfg.sanitizer:
            san = _SAN_CFG[cfg.sanitizer]
            _linker_flags(args, san["linker"])
            cxx_flags += san["cxx"].split()
            log.info(
                f'To enable, run: export {cfg.sanitizer.upper()}_OPTIONS="{san["env"]}"'
            )

        if cfg.coverage:
            cxx_flags += ["-fprofile-arcs", "-ftest-coverage", "--coverage"]
            _linker_flags(args, "--coverage")
            log.info(
                f"After tests: lcov --capture --directory {cfg.build_dir} "
                "--output-file coverage.info && "
                "genhtml coverage.info --output-directory coverage_report"
            )

        if cfg.benchmark:
            if is_cmake:
                # -ffast-math breaks IEEE semantics; benchmark builds only
                cxx_flags += ["-ffast-math"]
            else:
                # NodeMixin comms metrics (bandwidth, QoS faults). See METRICS_ON
                # in utils/node/node_mixin.hpp.
                cxx_flags += ["-DMETRICS_ON"]

        if cfg.native:
            # Tune for the build CPU. Non-portable: run only on this CPU.
            cxx_flags += ["-march=native", "-mtune=native"]

        if is_cmake and bt in ("Debug", "RelWithDebInfo"):
            cxx_flags += _CMAKE_WARN_FLAGS

        if is_cmake and cfg.run_tests:
            args.cmake_flags += ["-DBUILD_TESTS=ON"]

        cxx_flags += cfg.extra_cxx_flags

    args.cmake_flags += cfg.extra_cmake_args

    cxx_flags += _BUILD_CXX_FLAGS.get(bt, [])

    if bt == "Release" and (not is_cmake or cfg.benchmark):
        # GCC parallel LTO + cross-DSO inlining.
        cxx_flags += ["-flto=auto", "-fno-semantic-interposition"]

    args.cmake_flags += [
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        f"-DCMAKE_CXX_FLAGS={' '.join(cxx_flags)}",
        f"-DCMAKE_BUILD_TYPE={bt}",
        f"-DCMAKE_BUILD_PARALLEL_LEVEL={cfg.parallel_jobs}",
    ]

    args.colcon_flags += _BUILD_COLCON_FLAGS.get(bt, [])
    args.colcon_flags += ["--parallel-workers", str(cfg.parallel_jobs)]

    if not for_deps:
        if cfg.verbose:
            args.colcon_flags += ["--event-handlers", "console_direct+"]
        else:
            args.colcon_flags += ["--event-handlers", "console_cohesion+"]

    return args


def resolve_generator(requested: str) -> str:
    """Canonical CMake generator name; accepts any case, ninja/make shorthands."""
    if requested:
        gen = _GENERATORS.get(requested.lower())
        if not gen:
            log.error(f"Unknown generator: {requested}. Valid: ninja, make, xcode")
            raise SystemExit(1)
        if gen == "Ninja" and not shutil.which("ninja"):
            log.error("Ninja not found. Install: sudo apt install ninja-build")
            raise SystemExit(1)
        log.info(f"Using {gen} (explicit)")
        return gen

    if shutil.which("ninja"):
        log.info("Using Ninja (auto-detected)")
        return "Ninja"

    log.warning("Install Ninja for faster builds: sudo apt install ninja-build")
    return "Unix Makefiles"
