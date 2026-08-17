"""ROS 2 / colcon backend."""

from __future__ import annotations

import shutil
import time
from pathlib import Path

from .. import logger as log
from ..cmake_flags import BuildArgs, make_build_args
from ..config import BuildConfig, load_prev_config, save_config, save_pkg_configs
from ..dep_tracker import get_stale_deps
from ..python_env import setup_python_env
from ..runner import capture, run

_BUILD_ONLY_FLAGS = frozenset([
    "--symlink-install",
    "--continue-on-error",
    "--cmake-clean-first",
    "--cmake-clean-cache",
])


def clean(cfg: BuildConfig) -> None:
    log.section("Cleaning Build Artifacts")
    if not cfg.packages:
        for d in ["build", "install", "log"]:
            shutil.rmtree(d, ignore_errors=True)
        for f in ["compile_commands.json", ".build.json", ".build.config"]:
            Path(f).unlink(missing_ok=True)
        log.success("All build artifacts cleaned")
    else:
        for pkg in cfg.packages:
            for d in ["build", "install", "log"]:
                shutil.rmtree(f"{d}/{pkg}", ignore_errors=True)
        log.success(f"Cleaned: {' '.join(cfg.packages)}")


def clean_all(cfg: BuildConfig) -> None:
    log.section("Cleaning Everything")
    for d in ["build", "install", "log", ".ccache", "coverage_report"]:
        shutil.rmtree(d, ignore_errors=True)
    for f in [
        "compile_commands.json",
        ".build.json",
        ".build.config",
        "clang-tidy.log",
        "cppcheck.log",
        "coverage.info",
    ]:
        Path(f).unlink(missing_ok=True)
    for cmake_file in Path().glob("*.cmake"):
        cmake_file.unlink(missing_ok=True)
    log.success("Workspace fully cleaned")


def stats(cfg: BuildConfig) -> None:
    log.section("Build Statistics")
    for d in ["build", "install"]:
        p = Path(d)
        if not p.exists():
            log.plain(f"  {d}: not built")
            continue
        pkgs = [x for x in p.iterdir() if x.is_dir()]
        rc, out = capture(["du", "-sh", d])
        size = out.split()[0] if rc == 0 else "?"
        log.plain(f"  {d}: {len(pkgs)} packages, {size}")
    _print_last_config()
    _print_ccache_stats()


def _print_last_config() -> None:
    last = load_prev_config().get("last")
    if not last:
        return
    log.plain("\n  Last build:")
    for k in ("backend", "build_type", "sanitizer", "coverage", "parallel_jobs"):
        if k in last:
            log.plain(f"    {k}: {last[k]}")


def _print_ccache_stats() -> None:
    if not shutil.which("ccache"):
        return
    rc, out = capture(["ccache", "-s"])
    if rc == 0:
        log.plain("\n  ccache:")
        for line in out.splitlines():
            log.plain(f"    {line}")


def _clean_stale_pkg_dirs(cfg: BuildConfig) -> None:
    """Wipe build dirs whose recorded build_type/sanitizer/coverage changed."""
    prev = load_prev_config()
    pkgs: dict = prev.get("pkgs", {})
    if not pkgs:
        return

    build_root = Path("build")
    targets = cfg.packages or (
        [p.name for p in build_root.iterdir() if p.is_dir()]
        if build_root.exists()
        else []
    )

    for pkg in targets:
        rec = pkgs.get(pkg)
        if not rec:
            continue
        changed = [
            f"{k}: {rec[k]} → {v}"
            for k, v in [
                ("build_type", cfg.build_type),
                ("sanitizer", cfg.sanitizer),
                ("coverage", cfg.coverage),
            ]
            if str(rec.get(k, "")) != str(v)
        ]
        if not changed:
            continue
        pkg_build = build_root / pkg
        if pkg_build.exists():
            log.warning(f"{pkg}: {', '.join(changed)}, cleaning build dir")
            if not cfg.dry_run:
                shutil.rmtree(pkg_build)


def _colcon_build_cmd(packages: list[str] | None, args: BuildArgs) -> list[str]:
    cmd = ["colcon", "build"]
    if packages:
        cmd += ["--packages-select", *packages]
    return cmd + args.colcon_flags + ["--cmake-args"] + args.cmake_flags


def build(cfg: BuildConfig) -> None:
    if cfg.run_tests and not cfg.build_type_explicit:
        run_tests(cfg)  # `--test` alone means test-only for ROS
        return

    log.section("Build Configuration")
    log.plain("  Backend:  ros (colcon)")
    log.plain(f"  Type:     {cfg.build_type}")
    log.plain(f"  Packages: {', '.join(cfg.packages) if cfg.packages else 'all'}")
    log.plain(f"  Jobs:     {cfg.parallel_jobs}")
    if cfg.sanitizer:
        log.plain(f"  Sanitizer: {cfg.sanitizer}")
    if cfg.ccache:
        log.plain("  ccache: enabled")
    if cfg.coverage:
        log.plain("  Coverage: enabled")
    if cfg.dry_run:
        log.plain("  Mode: DRY RUN")

    if cfg.packages:
        _build_deps(cfg)

    _clean_stale_pkg_dirs(cfg)
    main_args = make_build_args(cfg)

    log.section("Starting Build")
    start = time.time()

    if not cfg.packages:
        log.step("Building all packages...")
    else:
        log.step(f"Building: {log.bold(', '.join(cfg.packages))}")

    rc = run(_colcon_build_cmd(cfg.packages or None, main_args), cfg.dry_run)
    elapsed = int(time.time() - start)

    if cfg.dry_run:
        return

    save_config(cfg)
    if cfg.packages and rc == 0:
        save_pkg_configs(cfg.packages, cfg.build_type, cfg.sanitizer, cfg.coverage)

    if rc != 0:
        log.error(f"Build failed after {elapsed}s")
        raise SystemExit(rc)

    log.plain("")
    log.success(f"Build completed in {elapsed}s")

    _symlink_compile_commands()
    setup_python_env(cfg.packages, cfg.venv_dir)


def _build_deps(cfg: BuildConfig) -> None:
    log.section("Building Dependencies (Release mode)")
    log.step("Checking which deps need rebuild...")

    stale = get_stale_deps(cfg.packages, force=cfg.force_deps)
    if not stale:
        log.success("All dependencies up to date — skipping dep build")
        log.plain("")
        return

    log.info(f"Stale deps: {log.cyan(', '.join(stale))}")
    dep_args = make_build_args(cfg, build_type="Release", for_deps=True)
    log.step(f"Building: {log.bold(', '.join(stale))}")

    rc = run(_colcon_build_cmd(stale, dep_args), cfg.dry_run)
    if rc != 0:
        log.error("Dependency build failed.")
        raise SystemExit(1)

    if not cfg.dry_run:
        save_pkg_configs(stale, "Release")
    log.plain("")
    log.success("Dependencies built in Release mode")
    log.plain("")


def _symlink_compile_commands() -> None:
    src = Path("build/compile_commands.json")
    dst = Path("compile_commands.json")
    if not src.exists():
        return
    if dst.is_symlink() and dst.resolve() == src.resolve():
        return
    dst.unlink(missing_ok=True)
    dst.symlink_to(src)
    log.success("compile_commands.json → build/compile_commands.json")


def run_tests(cfg: BuildConfig, main_args: BuildArgs | None = None) -> None:
    log.section("Running Tests (colcon)")
    colcon_flags = (
        [f for f in main_args.colcon_flags if f not in _BUILD_ONLY_FLAGS]
        if main_args
        else []
    )
    pkg_args = ["--packages-select", *cfg.packages] if cfg.packages else []

    run(["colcon", "test", *pkg_args, *colcon_flags], cfg.dry_run)
    run(["colcon", "test-result", "--verbose"], cfg.dry_run)
    log.success("Tests completed")


def lint(cfg: BuildConfig) -> None:
    # TODO(builder): colcon exposes linters as per-package test deps (ament_lint);
    # a standalone --lint would need per-pkg .clangd/compile_commands wiring.
    log.warning("--lint is cmake-backend only; use `colcon test` for ament linters")


def list_targets(cfg: BuildConfig) -> None:
    log.section("Packages")
    rc, out = capture(["colcon", "list", "--names-only"])
    if rc != 0:
        log.error("colcon list failed")
        raise SystemExit(1)
    for name in out.split():
        log.plain(f"  {name}")
