"""Step-response metrics in degrees; missing crossings never become a passing zero."""

import math
import statistics


def step_metrics(times, measured, start, target, *, band_deg=.5, tail_seconds=.75):
    if (len(times) != len(measured) or len(times) < 3
            or not all(math.isfinite(v) for v in [*times, *measured, start, target, band_deg])
            or any(b <= a for a, b in zip(times, times[1:]))
            or abs(target-start) < 1e-8 or band_deg <= 0 or tail_seconds <= 0
            or times[-1]-times[0] < tail_seconds):
        raise ValueError('a complete finite, increasing step response is required')
    progress = [(q-start)/(target-start) for q in measured]

    def crossing(level):
        for i, value in enumerate(progress):
            if value >= level:
                if i == 0:
                    return times[0]
                fraction = (level-progress[i-1])/(value-progress[i-1])
                return times[i-1]+fraction*(times[i]-times[i-1])
        return None

    t10, t90 = crossing(.1), crossing(.9)
    errors = [target-q for q in measured]
    tail = [e for t, e in zip(times, errors, strict=True) if t >= times[-1]-tail_seconds]
    last_outside = max((i for i, e in enumerate(errors) if abs(e) > band_deg), default=-1)
    settling = times[last_outside+1] if last_outside+1 < len(times) else None
    if settling is not None and times[-1]-settling < tail_seconds:
        settling = None
    gaps = [b-a for a, b in zip(times, times[1:])]
    overshoot = max(0., (max(progress)-1)*100)
    result = {'rise_time_10_90_s': None if t10 is None or t90 is None else t90-t10,
              'time_to_90_s': t90, 'overshoot_percent': overshoot,
              'settling_time_s': settling, 'settling_band_deg': band_deg,
              'steady_error_deg': statistics.mean(tail),
              'steady_peak_error_deg': max(map(abs, tail)),
              'rms_error_deg': math.sqrt(statistics.mean(e*e for e in errors)),
              'max_sample_gap_s': max(gaps), 'samples': len(times),
              'effective_sample_hz': (len(times)-1)/(times[-1]-times[0]),
              'median_sample_hz': 1/statistics.median(gaps)}
    # Acceptance is an explicit small-step criterion, not a motor specification.
    result['passed'] = (t90 is not None and settling is not None and settling <= 3.
                        and result['steady_peak_error_deg'] <= band_deg
                        and overshoot <= 35 and max(gaps) <= .1)
    return result
