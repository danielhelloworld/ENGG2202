"""Generate a standalone MaixVision browser-control entry without using hardware."""
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def build():
    parts = ["src/config.py", "vendor/protocol.py", "vendor/motor_base.py",
             "src/service.py", "src/mechanical_only.py", "src/web.py"]
    chunks, hashes = ["# Generated standalone MaixCAM2 browser controller. Edit CONFIG / WEB_CONFIG.\n"], {}
    for name in parts:
        path = ROOT / name
        code = path.read_text(encoding="utf-8")
        ast.parse(code)
        chunks.append("\n# ---- " + name + " ----\n" + code)
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    source = "\n".join(chunks)
    ast.parse(source)
    (ROOT / "main_web.py").write_text(source, encoding="utf-8")
    (ROOT / "web_source_hashes.json").write_text(json.dumps(hashes, indent=2)+"\n", encoding="utf-8")
    print("Built", ROOT / "main_web.py")


if __name__ == "__main__":
    build()
