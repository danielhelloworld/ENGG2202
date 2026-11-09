"""Bridge the existing overlay calibration to the independent thermal detector.

Bundled namespaces keep the original implementations and their globals isolated.
No UART is opened here. All geometry originates from the existing Alignment UI.
"""
import json
import numpy as np


def make_alignment(config, resolution):
    c = dict(thermal_api['CONFIG'])
    c['calibration_distances_m'] = [.5, 1., 2., 3., 5., 10., 30., 50., 100.]
    c.update(config.get('thermal_alignment', {}))
    c.update(camera_fps=config['camera_fps'], max_thermal_age=config['max_age']['thermal'],
             max_pair_delta=config['pair_delta'])
    return thermal_api['Alignment'](c, resolution, config['alignment_path'])


def normalized_overlay_matrix(alignment):
    """Undo letterboxing with cv2 pixel centres, then normalize RGB pixels.

    Alignment.matrix already includes thermal orientation and manual tuning.
    Never flip again, or normalize by the display width/height.
    """
    x, y, _, _, sx, sy = alignment.viewport
    iw, ih = alignment.source_size
    inverse_view = np.array([[1/sx, 0, -(x+(sx-1)/2)/sx],
                             [0, 1/sy, -(y+(sy-1)/2)/sy], [0, 0, 1.]])
    return np.diag([1/iw, 1/ih, 1.]) @ inverse_view @ alignment.matrix


def fusion_calibration(alignment, fixed_camera):
    """Recheck the EFFECTIVE mapping against stored independent check points.

    Manual-only overlays remain useful for display. They do not claim measured
    accuracy. A post-fit edit must still pass the original independent checks.
    A distance slot certifies only that measured distance, never an invented range.
    """
    matrix = normalized_overlay_matrix(alignment)
    verified, reason = False, 'NO POINT CAL'
    if not fixed_camera:
        reason = 'FIXED CAMERA REQUIRED'
    elif alignment.homography is not None:
        iw, ih = alignment.source_size
        effective = np.diag([iw, ih, 1.]) @ matrix
        stats = alignment.registration_stats
        try:
            thermal_api['check_registration'](effective, stats['check_thermal_points'],
                stats['check_visible_points'], alignment.source_size, stats,
                alignment.config['calibration_max_error_px'])
            verified, reason = True, 'POINT CAL CHECKED'
        except (ValueError, KeyError, TypeError):
            reason = 'TUNED MAPPING NEEDS CAL'
    return dict(thermal_to_rgb=matrix.tolist(), resolution=list(alignment.source_size),
                thermal_size=[160, 120], raw_unflipped=True, verified=verified,
                fixed_camera=bool(fixed_camera),
                distance_range_m=[alignment.distance, alignment.distance]), reason


class ThermalIntegration:
    def __init__(self, config, alignment):
        self.config, self.alignment = config, alignment
        options = dict(config.get('thermal_detector', {}))
        options['max_age'] = config['max_age']['thermal']
        self.detector = detector_api['Detector'](options)
        self.latest_detection = None

    def geometry_key(self):
        return json.dumps([self.alignment.distance, self.alignment.matrix.tolist(),
                           self.alignment.registration_stats, self.config['fixed_camera']], sort_keys=True)

    def update(self, frame, reader_count, now, paused=False):
        # Preserve the mailbox sequence; repeated RGB loops cannot create hits.
        supplied = dict(frame, reader_count=reader_count) if frame is not None else None
        result = self.detector.update(supplied, now)
        if result['valid']:
            self.latest_detection = result
        elif result['status'] != 'DUPLICATE':
            self.latest_detection = None
        calibration, reason = fusion_calibration(self.alignment, self.config['fixed_camera'])
        observations = [] if paused else adapter_api['observations'](
            result, calibration, self.alignment.source_size, self.alignment.distance)
        if paused:
            reason = 'CALIBRATING - FUSION PAUSED'
        return result, observations, reason

    def draw(self, canvas, now, visible_at):
        result = self.latest_detection
        if (not result or result['motion_quality'] < 1 or
                not 0 <= now-result['received_at'] <= self.config['max_age']['thermal'] or
                abs(visible_at-result['received_at']) > self.config['pair_delta']):
            return
        cv = thermal_api['cv2']
        for candidate in result['candidates']:
            x, y, w, h = candidate['bbox']
            corners = np.array([[[x-.5,y-.5], [x+w-.5,y-.5],
                                  [x+w-.5,y+h-.5], [x-.5,y+h-.5]]], np.float64)
            mapped = cv.perspectiveTransform(corners, self.alignment.matrix)[0]
            if not np.isfinite(mapped).all():
                continue
            cv.polylines(canvas, [np.rint(mapped).astype(np.int32)], True, (255,170,70), 1)
            tx, ty = mapped.mean(axis=0)
            thermal_api['draw_text_rgb'](canvas, 'T#%s' % candidate['track_id'],
                np.clip(tx+6,0,canvas.shape[1]-60), np.clip(ty,118,canvas.shape[0]-32),
                (255,170,70), .40)


def draw_fusion_tracks(canvas, tracks, alignment):
    """Positions share the RGB viewport used by the overlay, including padding."""
    x, y, w, h, _, _ = alignment.viewport
    iw, ih = alignment.source_size
    for track in tracks:
        px = round(x + (track['x']*iw + .5)*w/iw - .5)
        py = round(y + (track['y']*ih + .5)*h/ih - .5)
        color = (80, 255, 100) if track['state'] == 'VISUAL_CONFIRMED' else (255, 170, 70)
        thermal_api['cv2'].drawMarker(canvas, (px, py), color, markerSize=18)
        thermal_api['draw_text_rgb'](canvas, '#%d %s %.3f' %
            (track['id'], track['state'], track['score']), max(0, px-80),
            max(118, min(canvas.shape[0]-72, py-16)), color, .40)
