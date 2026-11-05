"""Convert measured thermal observations to drone_sensor_demo/core.py's observation schema.

thermal_to_rgb maps unflipped raw [x,y,1] to normalized RGB [u,v,1].
No guessed projection, coordinate or visual track identity is ever emitted.
"""
import math


def _project(matrix, x, y):
    if not isinstance(matrix, (list, tuple)) or len(matrix) != 3 or any(len(r) != 3 for r in matrix):
        return None
    try:
        m = [[float(v) for v in row] for row in matrix]
        if not all(math.isfinite(v) for row in m for v in row):
            return None
        denom = m[2][0]*x + m[2][1]*y + m[2][2]
        if abs(denom) < 1e-9:
            return None
        u = (m[0][0]*x + m[0][1]*y + m[0][2])/denom
        v = (m[1][0]*x + m[1][1]*y + m[1][2])/denom
        return (u, v) if math.isfinite(u) and math.isfinite(v) and 0 <= u <= 1 and 0 <= v <= 1 else None
    except (ValueError, TypeError, ZeroDivisionError):
        return None


def observations(result, calibration, rgb_resolution, distance_m=None):
    """Return fusion observations only for real detections and verified geometry.

    calibration needs matrix, resolution, thermal_size, raw_unflipped=True,
    verified=True, and a measured distance interval. A bare legacy matrix is
    deliberately insufficient; wrap it in this metadata after independent checks.
    """
    if not result.get('valid') or result.get('motion_quality', 1) < 1:
        return []
    c = calibration or {}
    if not c.get('verified') or c.get('raw_unflipped') is not True or c.get('thermal_size') != [160, 120]:
        return []
    if c.get('resolution') != list(rgb_resolution) or not c.get('fixed_camera', False):
        return []
    bounds = c.get('distance_range_m')
    if not bounds or distance_m is None or len(bounds) != 2 or not bounds[0] <= distance_m <= bounds[1]:
        return []
    out = []
    for item in result['candidates']:
        if item.get('track_id') is None:
            continue
        track = next((t for t in result['tracks'] if t['track_id'] == item['track_id']), None)
        if not track or not track['detected_this_frame'] or track['track_state'] != 'CONFIRMED':
            continue
        p = _project(c.get('thermal_to_rgb'), *item['center'])
        if p is None:
            continue
        out.append(dict(kind='thermal', x=p[0], y=p[1], score=item['quality_score'],
                        timestamp=result['received_at'], token='thermal:%s' % item['observation_id'],
                        calibrated=True, thermal_track_id=item['track_id']))
    return out
