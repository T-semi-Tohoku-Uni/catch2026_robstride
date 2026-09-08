"""Windows/MSVC host tests and deterministic stress CSVs; no device access."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--sanitizers", action="store_true")
args = parser.parse_args()
root = Path(__file__).resolve().parent.parent
cmake = Path(os.environ["LOCALAPPDATA"]) / "stm32cube/bundles/cmake/4.0.1+st.3/bin/cmake.exe"
if not cmake.exists():
    cmake = Path(shutil.which("cmake") or "cmake")
ctest = cmake.with_name("ctest.exe")
build = root / "build" / ("host-asan" if args.sanitizers else "host")
env = os.environ.copy()
if args.sanitizers:
    msvc = Path("C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Tools/MSVC")
    runtime = sorted(msvc.glob("*/bin/Hostx64/x64/clang_rt.asan_dynamic-x86_64.dll"))[-1].parent
    env["PATH"] = str(runtime) + os.pathsep + env["PATH"]

def run(*command):
    subprocess.run([str(x) for x in command], check=True, cwd=root, env=env)

run(cmake, "-S", root / "tests", "-B", build, "-G", "Visual Studio 17 2022", "-A", "x64",
    "-DCG_SANITIZERS=" + ("ON" if args.sanitizers else "OFF"))
run(cmake, "--build", build, "--config", "Debug")
run(ctest, "--test-dir", build, "-C", "Debug", "--output-on-failure")
for rate in (100, 200):
    output = build / f"stress_{rate}.csv"
    with output.open("w", encoding="ascii") as stream:
        subprocess.run([str(build / "Debug" / f"cg_test_{rate}.exe"), "--stress"],
                       check=True, cwd=root, env=env, stdout=stream)
    print(output)
