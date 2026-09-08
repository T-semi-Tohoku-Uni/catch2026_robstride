"""Compile the actual firmware scheduling callback into host regression tests."""
from pathlib import Path
import sys
source = Path(sys.argv[1]).read_text(encoding="utf-8")
start = source.index("void HAL_TIM_PeriodElapsedCallback(")
brace = source.index("{", start)
depth = 1
end = brace + 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
Path(sys.argv[2]).write_text(source[start:end], encoding="utf-8")
