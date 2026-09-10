"""Check Arm compiler selection without building or programming firmware."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile


def find_complete_compiler(repo):
    suffix = ".exe" if os.name == "nt" else ""
    candidates = []
    configured = os.environ.get("ARM_GNU_TOOLCHAIN_BIN_DIR")
    if configured:
        candidates.append(Path(configured) / ("arm-none-eabi-gcc" + suffix))
    discovered = shutil.which("arm-none-eabi-gcc")
    if discovered:
        candidates.append(Path(discovered))
    for directory, pattern in (
        (Path("/opt/ST"), "STM32CubeCLT_*"),
        (Path("/opt/st"), "stm32cubeclt_*"),
        (Path("C:/ST"), "STM32CubeCLT_*"),
    ):
        candidates.extend(
            installation / "GNU-tools-for-STM32/bin" / ("arm-none-eabi-gcc" + suffix)
            for installation in directory.glob(pattern)
        )
    for compiler in candidates:
        if not compiler.is_file():
            continue
        try:
            specs = subprocess.run(
                [str(compiler), "-print-file-name=nano.specs"],
                capture_output=True, text=True, timeout=10, check=True,
            ).stdout.strip()
            if not Path(specs).is_absolute() or not Path(specs).is_file():
                continue
            subprocess.run(
                [str(compiler), "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16",
                 "-mfloat-abi=hard", "-fsyntax-only",
                 str(repo / "toolchains/check_standard_headers.c")],
                capture_output=True, timeout=10, check=True,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        return compiler.resolve()
    return None


def cache_values(build):
    values = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.split(":", 1)[0]] = value
    return values


def check_selection(build, repo, compiler):
    values = cache_values(build)
    expected_bin = compiler.parent
    assert Path(values["CMAKE_TOOLCHAIN_FILE"]).resolve() == repo / "toolchains/arm-none-eabi.cmake"
    assert Path(values["ARM_GNU_TOOLCHAIN_BIN_DIR"]).resolve() == expected_bin
    assert Path(values["CMAKE_C_COMPILER"]).resolve() == compiler
    suffix = ".exe" if os.name == "nt" else ""
    for variable, executable in (
        ("ASM_COMPILER", "gcc"), ("CXX_COMPILER", "g++"),
        ("OBJCOPY", "objcopy"), ("SIZE", "size"),
        ("AR", "ar"), ("RANLIB", "ranlib"),
    ):
        selected = Path(values["CMAKE_" + variable]).resolve()
        expected = (expected_bin / ("arm-none-eabi-" + executable + suffix)).resolve()
        assert selected == expected, (variable, selected, expected)


def configure(cmake, ninja, repo, build, environment, extra=(), expect_success=True):
    result = subprocess.run(
        [cmake, "-S", str(repo), "-B", str(build), "-G", "Ninja",
         "-DCMAKE_MAKE_PROGRAM=" + ninja, "-DCMAKE_BUILD_TYPE=Debug", *extra],
        env=environment, capture_output=True, text=True, timeout=60,
    )
    output = result.stdout + result.stderr
    if expect_success:
        assert result.returncode == 0, output
    else:
        assert result.returncode != 0, output
        assert "A complete Arm GNU Toolchain" in output, output


def main():
    cmake, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    generated_toolchain = repo / "cmake/gcc-arm-none-eabi.cmake"
    generated_project = repo / "cmake/stm32cubemx/CMakeLists.txt"
    ninja = shutil.which("ninja") or shutil.which("ninja-build")
    if not generated_toolchain.is_file() or not generated_project.is_file() or not ninja:
        print("SKIP: Arm toolchain integration requires generated CubeMX files and Ninja.")
        return 77
    compiler = find_complete_compiler(repo)
    if compiler is None:
        print("SKIP: no complete Arm GNU Toolchain with nano.specs and standard headers.")
        return 77
    environment = os.environ.copy()
    for variable in ("CC", "CXX", "ASM"):
        environment.pop(variable, None)
    with tempfile.TemporaryDirectory(prefix="arm-toolchain-", dir=output) as temporary:
        directory = Path(temporary)
        build = directory / "selected"
        configure(cmake, ninja, repo, build, environment, (
            "-DCMAKE_TOOLCHAIN_FILE=" + str(generated_toolchain),
            "-DCMAKE_C_COMPILER=" + str(compiler),
        ))
        check_selection(build, repo, compiler)
        shadow_bin = directory / "shadow-bin"
        shadow_bin.mkdir()
        shadow_compiler = shadow_bin / ("arm-none-eabi-gcc.exe" if os.name == "nt" else "arm-none-eabi-gcc")
        shadow_compiler.write_text("This is deliberately not a compiler.\n", encoding="utf-8")
        shadow_compiler.chmod(0o755)
        environment["PATH"] = str(shadow_bin) + os.pathsep + environment.get("PATH", "")
        configure(cmake, ninja, repo, build, environment)
        check_selection(build, repo, compiler)
        configure(cmake, ninja, repo, directory / "invalid", environment, (
            "-DCMAKE_TOOLCHAIN_FILE=" + str(generated_toolchain),
            "-DCMAKE_C_COMPILER=" + str(directory / "missing-arm-none-eabi-gcc"),
        ), expect_success=False)
    print("Arm toolchain: redirect, explicit compiler, cached selection, siblings and rejection passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
