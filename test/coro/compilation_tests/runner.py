import argparse
import dataclasses
import json
import re
import subprocess
from pathlib import Path

START_LINE = "// RUNNER HEADER START"
END_LINE = "// RUNNER HEADER END"

OUTPUT_LINE_LEN = 60


@dataclasses.dataclass(frozen=True, kw_only=True)
class TestDescription:
    should_succeed: bool
    expected_diagnostics: list[re.Pattern]


def shallow_checked_cast[T](type: type[T], val: object) -> T:
    if not isinstance(val, type):
        raise TypeError
    return val


def parse_source(source_path: Path) -> TestDescription:
    with open(source_path, "r") as file:
        file_lines = file.read().splitlines()
    header_start_line = None
    for i, line in enumerate(file_lines):
        if line == START_LINE:
            header_start_line = i
            break
    if header_start_line is None:
        raise ValueError("Could not find header start.")
    data_lines = []
    for line in file_lines[header_start_line + 1 :]:
        if not line.startswith("//"):
            raise ValueError(
                f"Expected line to start with '//', found: '{line}'"
            )
        if line == END_LINE:
            break
        data_lines.append(line.removeprefix("//").strip())
    data = json.loads("\n".join(data_lines))
    patterns = [
        re.compile(shallow_checked_cast(str, pattern))
        for pattern in data["expected_diagnostics"]
    ]

    return TestDescription(
        should_succeed=shallow_checked_cast(bool, data["should_succeed"]),
        expected_diagnostics=patterns,
    )


def check_description(description: TestDescription) -> None:
    if description.should_succeed and len(description.expected_diagnostics) > 0:
        raise ValueError(
            "Expected diagnostics should not be used when the test is expected to succeed.\n"
            "It may already be compiled and thus the diagnostic won't be shown again."
        )
    if (
        not description.should_succeed
        and len(description.expected_diagnostics) == 0
    ):
        print(
            "Warning: Test expected to fail but no expected diagnostics provided.\n"
            "  This is not recommended because it may fail for different reason than you want."
        )


def print_compiler_output(result: subprocess.CompletedProcess[bytes]) -> None:
    msg = "Compiler stdout:"
    print(msg, "-" * (OUTPUT_LINE_LEN - 1 - len(msg)))
    print(result.stdout.decode())
    msg = "Compiler stderr:"
    print(msg, "-" * (OUTPUT_LINE_LEN - 1 - len(msg)))
    print(result.stderr.decode())
    print("-" * (OUTPUT_LINE_LEN - len(msg)))


def run_test(
    cmake_command: str,
    target: str,
    description: TestDescription,
) -> bool:
    result = subprocess.run(
        [cmake_command, "--build", ".", "--target", target], capture_output=True
    )
    print_compiler_output(result)
    failed = result.returncode == 0
    if failed != description.should_succeed:
        print(
            "FAIL: Result not expected:\n"
            f"  Should fail: {description.should_succeed}\n"
            f"  Exit code: {result.returncode}"
        )
        return False
    decoded_stdout = result.stdout.decode()
    decoded_stderr = result.stderr.decode()
    output = f"{decoded_stdout}\n{decoded_stderr}"
    patterns_ok = True
    for pattern in description.expected_diagnostics:
        found = pattern.search(output)
        if found is None:
            print(
                "FAIL: Pattern not matched\n"
                f"Output does not match this pattern: {pattern}"
            )
            patterns_ok = False
    return patterns_ok


def print_result(is_success: bool) -> None:
    if is_success:
        print(f"{" SUCCESS ":=^{OUTPUT_LINE_LEN}}")
    else:
        print(f"{" FAILURE ":=^{OUTPUT_LINE_LEN}}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmake-command", type=str, required=True)
    parser.add_argument("--target", type=str, required=True)
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()

    description = parse_source(args.source)
    check_description(description)
    is_success = run_test(args.cmake_command, args.target, description)
    print_result(is_success)
    exit(0 if is_success else 1)


if __name__ == "__main__":
    main()
