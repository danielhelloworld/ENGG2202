"""Build a standalone board entrypoint without importing or operating hardware."""
from pathlib import Path
import ast
import hashlib
import json

ROOT = Path(__file__).resolve().parent


def build():
    paths = [ROOT / p for p in ("src/config.py", "vendor/protocol.py", "vendor/motor_base.py",
                                "src/service.py", "src/mechanical_only.py", "src/ui.py")]
    chunks = ["# Generated standalone MaixCAM2 motor demo; CONFIG is editable below.\n"]
    hashes = {}
    for path in paths:
        code = path.read_text(encoding="utf-8")
        ast.parse(code)
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        chunks.append("\n# ---- " + str(path.relative_to(ROOT)) + " ----\n" + code)
    result = "\n".join(chunks).rstrip() + "\n"
    ast.parse(result)
    (ROOT / "main.py").write_text(result, encoding="utf-8")
    (ROOT / "source_hashes.json").write_text(json.dumps(hashes, indent=2) + "\n", encoding="utf-8")
    print("Built", ROOT / "main.py", len(result.splitlines()), "lines")


if __name__ == "__main__":
    build()
