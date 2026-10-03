"""Rebuild main.py from maintained sections and pinned local source snapshots."""
from pathlib import Path
import ast
import hashlib
import json

ROOT = Path(__file__).resolve().parent


def build():
    chunks = ["# Generated standalone MaixVision entry. Edit CONFIG below for board use.\n"]
    paths = [ROOT / "src/config.py", ROOT / "vendor/micarray.py", ROOT / "vendor/protocol.py"]
    paths += [ROOT / "src" / (name + ".py") for name in ("core", "motor", "vision", "runtime")]
    hashes = {}
    for path in paths:
        code = path.read_text(encoding="utf-8")
        ast.parse(code)
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        chunks.append("\n# ---- " + str(path.relative_to(ROOT)) + " ----\n" + code + "\n")
    bundle = "".join(chunks).rstrip() + "\n"
    ast.parse(bundle)
    (ROOT / "main.py").write_text(bundle, encoding="utf-8")
    zero_code = (ROOT / "src/zero_runtime.py").read_text(encoding="utf-8")
    zero_bundle = ("".join(chunks[:-1]) + zero_code).rstrip() + "\n"
    ast.parse(zero_bundle)
    (ROOT / "calibrate_zero.py").write_text(zero_bundle, encoding="utf-8")
    hashes["src/zero_runtime.py"] = hashlib.sha256((ROOT / "src/zero_runtime.py").read_bytes()).hexdigest()
    (ROOT / "source_hashes.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    print("Built", ROOT / "main.py", len(bundle.splitlines()), "lines")


if __name__ == "__main__":
    build()
