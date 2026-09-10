"""Compile extracted firmware callbacks with GCC/Clang or native MSVC."""
from pathlib import Path
import os
import subprocess


def compile_host(compiler, repo, generated, binary, defines=()):
    includes = [repo / "tests/stubs", repo / "Core/Inc"]
    if Path(compiler).name.lower() == "cl.exe":
        args = [compiler, "/nologo", "/std:c11", "/utf-8", "/W3", "/WX", "/UNDEBUG"]
        args += ["/D" + value for value in defines]
        args += ["/I" + str(path) for path in includes]
        args += [str(generated), "/Fe:" + str(binary), "/Fo:" + str(binary.with_suffix('.obj'))]
    else:
        args = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-UNDEBUG"]
        if os.environ.get("CATCH_HOST_SANITIZERS") == "1":
            args += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        args += ["-D" + value for value in defines]
        args += ["-I" + str(path) for path in includes]
        args += [str(generated), "-o", str(binary), "-lm"]
    subprocess.run(args, check=True)
