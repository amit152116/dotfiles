"""Plain CMake project backend (ported from the legacy cbuild script)."""

from __future__ import annotations

import shutil
import time
from pathlib import Path

from .. import logger as log
from ..cmake_flags import make_build_args
from ..config import (
    BuildConfig,
    cmake_signature,
    load_cmake_signature,
    load_prev_config,
    save_cmake_signature,
    save_config,
)
from ..runner import capture, run

_LINT_SRC_DIRS = ("src", "include", "lib", "apps")
_LINT_EXTS = ("*.cpp", "*.cc", "*.cxx", "*.hpp", "*.h")


def _install_dir(cfg: BuildConfig) -> str:
    return cfg.install_prefix or "install"


def clean(cfg: BuildConfig) -> None:
    log.section("Cleaning Build Artifacts")
    shutil.rmtree(cfg.build_dir, ignore_errors=True)
    shutil.rmtree(_install_dir(cfg), ignore_errors=True)
    for f in ["compile_commands.json", ".build.json", ".build.config"]:
        Path(f).unlink(missing_ok=True)
    log.success("Cleaned build artifacts")


def clean_all(cfg: BuildConfig) -> None:
    log.section("Cleaning Everything")
    for d in [cfg.build_dir, _install_dir(cfg), ".ccache", "coverage_report"]:
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
    log.success("Project fully cleaned")


def stats(cfg: BuildConfig) -> None:
    log.section("Build Statistics")
    build = Path(cfg.build_dir)
    if build.is_dir():
        rc, out = capture(["du", "-sh", cfg.build_dir])
        size = out.split()[0] if rc == 0 else "?"
        objs = sum(1 for _ in build.rglob("*.o"))
        log.plain(f"  {cfg.build_dir}: {size}, {objs} object files")
    else:
        log.plain(f"  {cfg.build_dir}: not built")

    inst = Path(_install_dir(cfg))
    if inst.is_dir():
        rc, out = capture(["du", "-sh", str(inst)])
        log.plain(f"  {inst}: {out.split()[0] if rc == 0 else '?'}")

    last = load_prev_config().get("last")
    if last:
        log.plain("\n  Last build:")
        for k in ("build_type", "sanitizer", "compiler", "coverage", "parallel_jobs"):
            if k in last:
                log.plain(f"    {k}: {last[k]}")

    if shutil.which("ccache"):
        rc, out = capture(["ccache", "-s"])
        if rc == 0:
            log.plain("\n  ccache:")
            for line in out.splitlines():
                log.plain(f"    {line}")


def list_targets(cfg: BuildConfig) -> None:
    if not Path(cfg.build_dir).is_dir():
        log.error(f"{cfg.build_dir} not found. Configure first by running a build.")
        raise SystemExit(1)
    log.section("Build Targets")
    rc = run(["cmake", "--build", cfg.build_dir, "--target", "help"], cfg.dry_run)
    if rc != 0:
        log.warning("Could not list targets (generator may not support 'help')")


def _invalidate_stale_cache(cfg: BuildConfig) -> None:
    """Wipe the build dir when a cache-invalidating field changed since last build."""
    prev = load_cmake_signature()
    if not prev:
        return
    cur = cmake_signature(cfg)
    changed = [
        f"{k}: {prev.get(k) or 'none'} → {v or 'none'}"
        for k, v in cur.items()
        if str(prev.get(k, "")) != str(v)
    ]
    if not changed:
        return
    log.warning(f"Config changed ({', '.join(changed)}), cleaning {cfg.build_dir}")
    if not cfg.dry_run:
        shutil.rmtree(cfg.build_dir, ignore_errors=True)


def build(cfg: BuildConfig) -> None:
    args = make_build_args(cfg)

    # ctest needs every test binary, so a target filter would starve it
    targets = [] if cfg.run_tests else cfg.targets
    if cfg.run_tests and cfg.targets:
        log.warning("--test builds all targets; ignoring --target")

    log.section("Build Configuration")
    log.plain("  Backend:  cmake")
    log.plain(f"  Type:     {cfg.build_type}")
    log.plain(f"  Generator: {args.generator}")
    log.plain(f"  Build dir: {cfg.build_dir}")
    log.plain(f"  Targets:  {', '.join(targets) if targets else 'all'}")
    log.plain(f"  Jobs:     {cfg.parallel_jobs}")
    if cfg.compiler:
        log.plain(f"  Compiler: {cfg.compiler}")
    if cfg.sanitizer:
        log.plain(f"  Sanitizer: {cfg.sanitizer}")
    if cfg.ccache:
        log.plain("  ccache: enabled")
    if cfg.coverage:
        log.plain("  Coverage: enabled")
    if cfg.dry_run:
        log.plain("  Mode: DRY RUN")

    _invalidate_stale_cache(cfg)

    start = time.time()

    log.section("Configuring")
    config_cmd = ["cmake", "-B", cfg.build_dir, "-G", args.generator, *args.cmake_flags]
    if cfg.verbose:
        config_cmd.append("--debug-output")
    if run(config_cmd, cfg.dry_run) != 0:
        log.error("CMake configuration failed")
        raise SystemExit(1)

    log.section("Building")
    build_cmd = [
        "cmake",
        "--build",
        cfg.build_dir,
        "--parallel",
        str(cfg.parallel_jobs),
    ]
    if targets:
        build_cmd += ["--target", *targets]
    if cfg.verbose:
        build_cmd.append("--verbose")
    rc = run(build_cmd, cfg.dry_run)
    elapsed = int(time.time() - start)

    if cfg.dry_run:
        return

    if rc != 0:
        log.error(f"Build failed after {elapsed}s")
        raise SystemExit(rc)

    save_config(cfg)
    save_cmake_signature(cfg)
    log.plain("")
    log.success(f"Build completed in {elapsed}s")

    _symlink_compile_commands(cfg)

    if cfg.install:
        install(cfg)
    if cfg.run_tests:
        run_tests(cfg)
    if cfg.lint:
        lint(cfg)

    log.info(f"Build artifacts are in: {cfg.build_dir}/")


def install(cfg: BuildConfig) -> None:
    log.section("Installing")
    if run(["cmake", "--install", cfg.build_dir], cfg.dry_run) != 0:
        log.error("Install failed")
        raise SystemExit(1)
    log.success(f"Installed to {_install_dir(cfg)}")


def run_tests(cfg: BuildConfig) -> None:
    log.section("Running Tests (ctest)")
    if not Path(cfg.build_dir).is_dir() and not cfg.dry_run:
        log.error(f"{cfg.build_dir} not found. Build first.")
        raise SystemExit(1)
    cmd = ["ctest", "--test-dir", cfg.build_dir, "--output-on-failure"]
    if cfg.verbose:
        cmd.append("--verbose")
    rc = run(cmd, cfg.dry_run)
    if rc != 0:
        log.error("Tests failed")
        raise SystemExit(rc)
    log.success("Tests passed")


def lint(cfg: BuildConfig) -> None:
    log.section("Linting")
    ccdb = Path(cfg.build_dir) / "compile_commands.json"
    if not ccdb.exists() and not cfg.dry_run:
        log.error(f"{ccdb} not found. Build first.")
        raise SystemExit(1)

    if shutil.which("clang-tidy"):
        files = [
            str(f)
            for d in _LINT_SRC_DIRS
            if Path(d).is_dir()
            for ext in _LINT_EXTS
            for f in Path(d).rglob(ext)
        ]
        # WARN(builder): file list passed unbatched; a huge tree could exceed ARG_MAX
        if files:
            _tee(["clang-tidy", "-p", cfg.build_dir, *files], "clang-tidy.log", cfg)
        else:
            log.warning(f"No sources found in: {', '.join(_LINT_SRC_DIRS)}")
    else:
        log.warning("clang-tidy not found. Install: sudo apt install clang-tidy")

    if shutil.which("cppcheck"):
        _tee(
            [
                "cppcheck",
                "--enable=all",
                "--suppress=missingIncludeSystem",
                f"--project={ccdb}",
            ],
            "cppcheck.log",
            cfg,
        )
    else:
        log.warning("cppcheck not found. Install: sudo apt install cppcheck")

    log.success("Lint completed")


def _tee(cmd: list[str], logfile: str, cfg: BuildConfig) -> None:
    """Run a linter, streaming to the terminal and appending to logfile."""
    # HACK(builder): shell pipe to tee keeps live output without threading stdout
    import shlex

    piped = ["bash", "-c", f"{shlex.join(cmd)} 2>&1 | tee {shlex.quote(logfile)}"]
    run(piped, cfg.dry_run)


def _symlink_compile_commands(cfg: BuildConfig) -> None:
    src = Path(cfg.build_dir) / "compile_commands.json"
    dst = Path("compile_commands.json")
    if not src.exists():
        return
    if dst.is_symlink() and dst.resolve() == src.resolve():
        return
    dst.unlink(missing_ok=True)
    dst.symlink_to(src)
    log.success(f"compile_commands.json → {src}")
