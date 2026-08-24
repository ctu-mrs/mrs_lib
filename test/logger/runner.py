#!/usr/bin/env python3


import argparse
import re
import subprocess
from enum import Enum
from pathlib import Path


class Level(Enum):
    debug = (0, "DEBUG")
    info = (1, "INFO")
    warning = (2, "WARN")
    error = (3, "ERROR")
    fatal = (4, "FATAL")

    def ros_name(self) -> str:
        return self.value[1]

    def rank(self) -> int:
        return self.value[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bin-path", type=Path, required=True)
    return parser.parse_args()


def run_binary(path: Path, base_level: Level) -> list[str]:
    print(f"Running test code at path: ${path}")
    res = subprocess.run(
        [path, base_level.ros_name()],
        capture_output=True,
        check=True,
        text=True,
    )
    return res.stderr.strip().split("\n")


def get_ros_prefix(level: Level) -> str:
    return rf".*\[{level.ros_name()}\].*"


def get_patterns(base_level: Level) -> list[re.Pattern[str]]:
    patterns: list[str] = []
    for level in Level:
        if level.rank() < base_level.rank():
            continue
        ros_prefix = get_ros_prefix(level)
        patterns.append(f"{ros_prefix} Hi")
        patterns.append(f"{ros_prefix} Hello world")
        patterns.append(f"{ros_prefix} Only Once")
        patterns.append(f"{ros_prefix} Also this is only once")
        patterns.append(f"{ros_prefix} Throttled a: 0")
        patterns.append(f"{ros_prefix} Throttled b: 0")
        patterns.append(f"{ros_prefix} Throttled a: 3")
        patterns.append(f"{ros_prefix} Throttled b: 3")
    return [re.compile(f"^{x}$") for x in patterns]


def check_output(lines: list[str], base_level: Level) -> int:
    failures = 0
    patterns = get_patterns(base_level)
    if len(lines) != len(patterns):
        print(
            f"Wrong number of output lines:"
            f"- Expected: {len(patterns)}"
            f"- Found: {len(lines)}."
        )
        failures += 1
    else:
        print("[  OK] Line count")
    for i, (line, pattern) in enumerate(zip(lines, patterns)):
        if pattern.match(line) is None:
            print(f"[FAIL] Line {i + 1} does not match the expected pattern:")
            print(f"- Expected (regex): {pattern.pattern}")
            print(f"- Found: {line}")
            failures += 1
        else:
            print(f"[  OK] Line {i + 1}")
    return failures


def main() -> int:
    args: argparse.Namespace = parse_args()
    failed_tests = 0
    for base_level in Level:
        print(f"== {f"{base_level.ros_name()} ":=<{60}}")
        lines = run_binary(args.bin_path, base_level)
        failures = check_output(lines, base_level)
        if failures == 0:
            print("Test OK.")
        else:
            print(f"Test FAIL. ({failures} failures)")
            failed_tests += 1
    return failed_tests


if __name__ == "__main__":
    exit(main())
