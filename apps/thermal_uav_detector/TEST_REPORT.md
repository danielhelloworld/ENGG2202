# Test report — 2026-11-06

Environment: local Python 3 standard library, no device. Command: `python3 -m unittest discover -s apps/thermal_uav_detector/tests -v` → **6 tests passed**. Command: `cd apps/thermal_uav_detector && python3 replay.py --synthetic --output synthetic_results.jsonl` → 12 SYNTHETIC frames, median processing 51.02 ms/frame on this host. Output: `synthetic_results.jsonl`.

The unit tests cover single-pixel survival, repeated frames not adding hits, three-frame confirmation, stale/future/invalid temperature rejection, no-frame dropout, short hold and reacquisition, ambiguous association rejection, missing/invalid calibration rejection, and uniform gain change, valid adapter output, resolution/distance mismatch, and broad-motion suppression. The synthetic sequence includes a moving point, larger hot block, masked fixed bad pixel, two-frame occlusion, gain change, and broad background change. Broad change produces `MOTION_UNCERTAIN` and disables fusion evidence for the cooldown. These are synthetic outcomes, not detection rates.

No real annotated replay, board run, temperature measurement, or 5/10/30/50/100 m validation was performed. Detection rate, missed detections, false positives/minute, localization error and tracking interruption rate require complete labeled capture sessions. No train/test split or accuracy claim is possible without them.

The board single-file build was exercised against a temporary copy of the branch's actual `config.py`, `thermal.py`, attribution and license; `build.py` completed and `board_main.py` passed `py_compile`. This is a build check, not a board run.
