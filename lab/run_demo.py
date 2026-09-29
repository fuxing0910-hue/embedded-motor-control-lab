#!/usr/bin/env python3
"""Compile, test and run the motor-control simulation using only Python's stdlib."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("step", "load", "saturation")


def find_compiler(override: str | None) -> Path:
    """Prefer an explicit compiler, then PATH, then common Windows installs."""
    if override:
        resolved = shutil.which(override)
        candidate = Path(resolved or override).expanduser()
        if not candidate.is_file():
            raise FileNotFoundError(f"C++ compiler does not exist: {override}")
        return candidate.resolve()
    for name in ("g++", "g++.exe"):
        resolved = shutil.which(name)
        if resolved:
            return Path(resolved).resolve()
    if os.name == "nt":
        for candidate in (
            Path("C:/msys64/ucrt64/bin/g++.exe"),
            Path("C:/msys64/mingw64/bin/g++.exe"),
        ):
            if candidate.is_file():
                return candidate
    raise FileNotFoundError(
        "g++ was not found. Install a C++17-capable GCC compiler, add it to PATH, "
        "or run: python lab/run_demo.py --compiler /path/to/g++"
    )


def run(command: list[str], env: dict[str, str]) -> None:
    print("+ " + subprocess.list2cmdline(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="Path to g++; overrides automatic discovery")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "results")
    parser.add_argument("--duration", type=float, default=12.0, help="Seconds per scenario (default: 12)")
    parser.add_argument("--kp", type=float, default=0.08)
    parser.add_argument("--ki", type=float, default=0.25)
    parser.add_argument("--kd", type=float, default=0.001)
    parser.add_argument("--skip-report", action="store_true", help="Generate CSV files without matplotlib or HTML")
    args = parser.parse_args()

    compiler = find_compiler(args.compiler)
    build_dir = args.build_dir.expanduser().resolve()
    results_dir = args.results_dir.expanduser().resolve()
    build_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # Windows GCC executables and generated programs may need adjacent runtime DLLs.
    env["PATH"] = str(compiler.parent) + os.pathsep + env.get("PATH", "")
    extension = ".exe" if os.name == "nt" else ""
    simulation = build_dir / ("motor_sim" + extension)
    tests = build_dir / ("test_pid" + extension)
    common = [
        str(compiler), "-std=c++17", "-O2", "-Wall", "-Wextra", "-pedantic",
        "-DARDUINO=100", "-I" + str(ROOT), "-I" + str(ROOT / "lab" / "host"),
        str(ROOT / "PID_v1.cpp"), str(ROOT / "lab" / "host" / "clock.cpp"),
        str(ROOT / "lab" / "motor_sim.cpp"),
    ]
    run(common + [str(ROOT / "lab" / "test_pid.cpp"), "-o", str(tests)], env)
    run([str(tests)], env)
    run(common + [str(ROOT / "lab" / "sim_main.cpp"), "-o", str(simulation)], env)
    for scenario in SCENARIOS:
        run([
            str(simulation), "--scenario", scenario, "--duration", str(args.duration),
            "--kp", str(args.kp), "--ki", str(args.ki), "--kd", str(args.kd),
            "--output", str(results_dir / f"{scenario}.csv"),
        ], env)
    if not args.skip_report:
        run([sys.executable, str(ROOT / "lab" / "make_report.py"), "--results-dir", str(results_dir)], env)
        print("\nOpen the standalone report: " + (results_dir / "report.html").as_uri())
    else:
        print("\nCSV results: " + str(results_dir))


if __name__ == "__main__":
    try:
        main()
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"Demo failed: {exc}", file=sys.stderr)
        sys.exit(exc.returncode if isinstance(exc, subprocess.CalledProcessError) else 1)
