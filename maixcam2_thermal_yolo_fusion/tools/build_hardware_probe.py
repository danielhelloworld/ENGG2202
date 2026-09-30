"""Build a self-contained real-hardware test for MaixVision Run Current File."""
import ast
from pathlib import Path

app = Path(__file__).resolve().parents[1]
source = (app / "main.py").read_text(encoding="utf-8")
tree = ast.parse(source)
entry = tree.body[-1]
assert isinstance(entry, ast.If) and "__name__" in ast.unparse(entry.test)
library = "\n".join(source.splitlines()[:entry.lineno - 1])
runner = (app / "tools/hardware_probe_runner.py").read_text(encoding="utf-8")
header = (
    "# MAIXVISION HARDWARE TEST: open THIS generated file and use Run Current File.\n"
    "# Real hardware diagnostic: runs for 90 seconds; no synthetic fallback.\n"
    'if __name__ == "__main__":\n'
    '    print("MAIX_PROBE START hardware_probe.py | real hardware test", flush=True)\n'
)
bundle = header + library + "\n" + runner
ast.parse(bundle)
output = app / "hardware_probe.py"
output.write_text(bundle, encoding="utf-8")
print(output)
