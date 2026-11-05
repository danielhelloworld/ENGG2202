"""Desktop interactive synthetic demo. 1-6 scenarios; S saves weights; Q exits."""
import argparse
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
from core import Fusion, load_config, save_config, observation, thermal_candidates

ROOT = Path(__file__).resolve().parent
SCENARIOS = ["5m / all sensors", "10m / visual only", "Far / sound + thermal",
             "Visual dropout", "Wrong-direction sound", "Stale sensor packets"]


def fixture(index, now, step):
    x, y = .48 + .15 * math.sin(now * .4), .43 + .08 * math.cos(now * .5)
    obs = []
    if index != 2 and not (index == 3 and step % 90 > 45):
        obs.append(observation("vision", x, y, .93, now, step, "drone"))
    if index != 1:
        stamp = now - 2 if index == 5 else now
        obs.append(observation("sound", .90 if index == 4 else x + .025, y, .82, stamp, step))
        obs.append(observation("thermal", x - .015, y + .012, .88, stamp, step))
    return obs


def render(tracks, observations, config, title, details="", background=None, thermal=None):
    canvas = np.full((720, 1100, 3), (25, 20, 15), np.uint8)
    def text(s, x, y, scale=.55, color=(215, 220, 225)):
        cv2.putText(canvas, str(s), (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    text("DRONE / MULTISENSOR EVIDENCE", 28, 38, .8, (235, 235, 235))
    text(title, 28, 67, .58, (80, 200, 255))
    x0, y0, w, h = 25, 95, 680, 430
    if background is not None:
        canvas[y0:y0+h, x0:x0+w] = cv2.resize(background, (w, h))
    else:
        canvas[y0:y0+h, x0:x0+w] = (48, 38, 26)
        for x in range(x0, x0+w, 68):
            cv2.line(canvas, (x, y0), (x, y0+h), (65, 52, 38))
        for y in range(y0, y0+h, 43):
            cv2.line(canvas, (x0, y), (x0+w, y), (65, 52, 38))
    colors = {"vision": (100, 240, 100), "sound": (240, 170, 70), "thermal": (60, 140, 255)}
    for o in observations:
        px, py = int(x0+o["x"]*w), int(y0+o["y"]*h)
        cv2.circle(canvas, (px, py), 9, colors[o["kind"]], 2)
        offset = {"vision": -12, "sound": 6, "thermal": 24}[o["kind"]]
        text(o["kind"], px+30, py+offset, .4, colors[o["kind"]])
    for t in tracks:
        px, py = int(x0+t["x"]*w), int(y0+t["y"]*h)
        cv2.rectangle(canvas, (px-24, py-20), (px+24, py+20), (100, 230, 230), 2)
        text("#%d %s" % (t["id"], t["state"]), max(30, px-80), max(115, py-30), .42)
    text("SENSOR WEIGHTS / no missing-channel renormalization", 25, 558, .5)
    for i, kind in enumerate(colors):
        x = 25 + 230*i
        text("%s %.0f%%" % (kind.upper(), config["weights"][kind]*100), x, 587, .5, colors[kind])
        cv2.rectangle(canvas, (x, 600), (x+int(200*config["weights"][kind]), 613), colors[kind], -1)
    text("Evidence score is NOT accuracy or class probability.", 25, 650, .52)
    text(details[:115], 25, 685, .43)
    text("TARGET EVIDENCE", 735, 115, .65)
    t = tracks[0] if tracks else None
    text(t["state"] if t else "NO TARGET", 735, 155, .51, (80, 220, 255))
    text("Score: %.3f" % t["score"] if t else "Score: --", 735, 192, .7)
    text("New visual hits: %d" % t["hits"] if t else "New visual hits: 0", 735, 224)
    for i, k in enumerate(colors):
        value = t["contributions"][k] if t else 0
        yy = 270 + i*70
        text("%s contribution %.3f" % (k, value), 735, yy, .48)
        cv2.rectangle(canvas, (735, yy+12), (735+int(value*320), yy+25), colors[k], -1)
    if thermal is not None:
        thumb = cv2.applyColorMap(np.asarray(thermal, np.uint8), cv2.COLORMAP_INFERNO)
        canvas[480:660, 735:975] = cv2.resize(thumb, (240, 180))
        text("THERMAL / original 160x120", 735, 465, .43)
    else:
        for i, s in enumerate(SCENARIOS):
            text("%d  %s" % (i+1, s), 735, 470+i*30, .43)
    return canvas


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke-ui", action="store_true", help="Open real window, exercise sliders, exit after 12 frames")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--config", default=str(ROOT / "config.json"))
    args = parser.parse_args()
    config = load_config(args.config)
    engine = Fusion(config)
    scenario, step, message = 0, 0, "1-6: scenario | S: save weights | Q: quit"
    if not args.headless:
        cv2.namedWindow("Drone sensor demo", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Drone sensor demo", 1100, 800)
        cv2.createTrackbar("Vision 60-100", "Drone sensor demo", round(config["weights"]["vision"]*100)-60, 40, lambda x: None)
        aux = config["weights"]["sound"]+config["weights"]["thermal"]
        cv2.createTrackbar("Sound share of auxiliary", "Drone sensor demo", round(100*config["weights"]["sound"]/aux) if aux else 50, 100, lambda x: None)
        if args.smoke_ui:
            cv2.setTrackbarPos("Vision 60-100", "Drone sensor demo", 20)
            cv2.setTrackbarPos("Sound share of auxiliary", "Drone sensor demo", 60)
    records = []
    try:
        while not args.headless or step < args.frames*6:
            if args.headless:
                index = step // args.frames
                if index != scenario:
                    scenario, engine = index, Fusion(config)
                now = step/15
            else:
                now = time.monotonic()
                v = (.60 + cv2.getTrackbarPos("Vision 60-100", "Drone sensor demo")/100)
                share = cv2.getTrackbarPos("Sound share of auxiliary", "Drone sensor demo")/100
                config["weights"] = dict(vision=v, sound=(1-v)*share, thermal=(1-v)*(1-share))
            obs = fixture(scenario, now, step)
            tracks, rejected = engine.update(obs, now)
            canvas = render(tracks, obs, config, "SYNTHETIC / " + SCENARIOS[scenario], message)
            if args.headless:
                if step % args.frames == args.frames-1:
                    dest = ROOT / "validation"
                    dest.mkdir(exist_ok=True)
                    cv2.imwrite(str(dest / ("scenario_%d.png" % (scenario+1))), canvas)
                    records.append(dict(scenario=SCENARIOS[scenario], tracks=tracks, rejected=rejected))
            else:
                cv2.imshow("Drone sensor demo", canvas)
                key = cv2.waitKey(50) & 255
                if key in (27, ord("q")) or cv2.getWindowProperty("Drone sensor demo", cv2.WND_PROP_VISIBLE) < 1:
                    break
                if ord("1") <= key <= ord("6"):
                    scenario, engine = key-ord("1"), Fusion(config)
                if key == ord("s"):
                    save_config(args.config, config)
                    message = "Saved fusion weights to config.json"
            step += 1
            if args.smoke_ui and step >= 12:
                assert abs(config["weights"]["vision"]-.8) < 1e-6
                assert abs(config["weights"]["sound"]-.12) < 1e-6
                print("Real OpenCV window and slider readback passed")
                break
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
    if args.headless:
        (ROOT / "validation" / "scenarios.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
        print("Six synthetic scenarios rendered; no hardware/model inference.")


if __name__ == "__main__":
    main()
