"""Create standalone MaixVision main.py; original applications remain untouched."""
import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def before_main(source):
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.If) and "__name__" in ast.unparse(node.test):
            return "\n".join(source.splitlines()[:node.lineno-1]) + "\n"
    return source


def build():
    thermal_dir = ROOT.parent / "thermal_yolo_fusion"
    thermal = "\n".join((thermal_dir / "src" / (name+".py")).read_text(encoding="utf-8") for name in ("config", "thermal", "calibration", "fusion"))
    detector_dir = ROOT.parent / "thermal_uav_detector"
    detector = (detector_dir / "detector.py").read_text(encoding="utf-8")
    adapter = (detector_dir / "adapter.py").read_text(encoding="utf-8")
    sound = before_main((ROOT.parent / "micarray_direction/micarray_direction.py").read_text(encoding="utf-8"))
    core = (ROOT / "core.py").read_text(encoding="utf-8")
    demo_ast = ast.parse((ROOT / "demo.py").read_text(encoding="utf-8"))
    render = next(ast.unparse(n) for n in demo_ast.body if isinstance(n, ast.FunctionDef) and n.name == "render")
    bundle = '# Generated MaixVision entry. Edit core.py/board_runtime.py then run build.py.\n'
    bundle += 'thermal_api = {"__name__": "thermal_adapter"}\nexec(' + repr(thermal) + ', thermal_api)\n'
    bundle += 'detector_api = {"__name__": "thermal_detector"}\nexec(' + repr(detector) + ', detector_api)\n'
    bundle += 'adapter_api = {"__name__": "thermal_projection"}\nexec(' + repr(adapter) + ', adapter_api)\n'
    bundle += 'sound_api = {"__name__": "sound_adapter"}\nexec(' + repr(sound) + ', sound_api)\n'
    bundle += core + '\nSCENARIOS = []\n' + render + '\n'
    bundle += (ROOT / "thermal_integration.py").read_text(encoding="utf-8") + '\n'
    bundle += (ROOT / "board_runtime.py").read_text(encoding="utf-8")
    ast.parse(bundle)
    (ROOT / "main.py").write_text(bundle, encoding="utf-8")
    from core import DEFAULT
    if not (ROOT / "config.json").exists():
        (ROOT / "config.json").write_text(json.dumps(DEFAULT, indent=2), encoding="utf-8")
    for name in ("THIRD_PARTY.md", "LICENSE_SIPEED.txt"):
        (ROOT / name).write_bytes((thermal_dir / name).read_bytes())
    print("Built standalone main.py")


if __name__ == "__main__":
    build()
