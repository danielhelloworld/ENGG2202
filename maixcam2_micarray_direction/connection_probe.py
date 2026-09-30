"""MaixVision 上运行的 MA-USB8 连接只读探测，不打开设备或修改 pinmux。"""

import glob
import json
import shutil
import subprocess


def read_text(path, limit=6000):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return handle.read(limit).strip()
    except OSError as exc:
        return "unavailable: " + str(exc)


def command_output(name, *args):
    if not shutil.which(name):
        return "not installed"
    try:
        result = subprocess.run(
            [name, *args], capture_output=True, text=True, timeout=5, check=False
        )
        return {
            "exit_code": result.returncode,
            "stdout": result.stdout[:6000].strip(),
            "stderr": result.stderr[:1500].strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "unavailable: " + str(exc)


def maix_info():
    try:
        from maix import pinmap, sys, uart
    except Exception as exc:
        return {"error": "maix import failed: " + repr(exc)}
    info = {}
    try:
        info["device_id"] = sys.device_id()
    except Exception as exc:
        info["device_id_error"] = repr(exc)
    try:
        info["uart_devices"] = list(uart.list_devices())
    except Exception as exc:
        info["uart_devices_error"] = repr(exc)
    try:
        candidates = {}
        for pin in pinmap.get_pins():
            funcs = [str(value) for value in pinmap.get_pin_functions(pin)]
            ports = [value for value in funcs if "UART" in value.upper()]
            if ports:
                try:
                    current = str(pinmap.get_pin_function(pin))
                except Exception as exc:
                    current = "unavailable: " + repr(exc)
                candidates[str(pin)] = {"uart_functions": ports, "current": current}
        info["uart_pin_candidates"] = candidates
    except Exception as exc:
        info["pinmap_error"] = repr(exc)
    return info


def main():
    report = {
        "tty_acm": sorted(glob.glob("/dev/ttyACM*")),
        "tty_usb": sorted(glob.glob("/dev/ttyUSB*")),
        "tty_s": sorted(glob.glob("/dev/ttyS*")),
        "alsa_cards": read_text("/proc/asound/cards"),
        "alsa_pcm": read_text("/proc/asound/pcm"),
        "lsusb": command_output("lsusb"),
        "arecord_list": command_output("arecord", "-l"),
        "maix": maix_info(),
    }
    print("MAIX_MIC_PROBE " + json.dumps(report, ensure_ascii=False, default=str))
    print("MAIX_MIC_PROBE END: no serial port opened, no audio recorded, no pin changed")


if __name__ == "__main__":
    main()
