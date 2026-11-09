"""MaixVision entry using the existing bundled drone_sensor_demo reader.

Place this directory alongside apps/drone_sensor_demo after its build.py has generated
main.py. Does not start camera, motor, or sound. Check UART2 occupancy first.
"""
import time
from detector import Detector


def run(reader):
    detector = Detector()
    reader.start()
    try:
        while True:
            frame, status, count = reader.snapshot()
            if frame is not None:
                frame = dict(frame, reader_count=count)
            result = detector.update(frame, time.monotonic())
            print('THERMAL_UAV', result['status'], len(result['candidates']),
                  [(t['track_id'], t['center'], t['track_state']) for t in result['tracks']])
            time.sleep(.1)
    finally:
        reader.close()


if __name__ == '__main__':
    # The existing generated main.py embeds thermal_api and CONFIG. Import it only
    # when that entry is refactored to expose the API without starting its app.
    raise SystemExit('Inject an existing ThermalReader instance: maixvision.run(reader). See HANDOFF.md')
