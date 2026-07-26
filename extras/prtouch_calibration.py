# prtouch_v2 calibration math - pure functions, no MCU/reactor dependency
#
# Clean-room rewrite of cal_tri_data()/get_valid_ch() from Creality's prtouch_v2_wrapper.py
# (GPLv3, see reference/prtouch_v2_wrapper.py lines 653-763), read completely and traced in
# ../ANALYSIS.md sec 4. Every function here takes plain numbers/lists and returns plain numbers -
# no Klipper objects - so it can be tested standalone (see test_prtouch_calibration.py) against
# synthetic data without a real printer.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math


def select_valid_channels(tri_chs_bitmask, pres_cnt=4):
    """Which pressure channels actually reported a trigger (get_valid_ch-equivalent).

    The original also computes each channel's geometric distance from the current XY to its
    bed corner, but that distance is diagnostic-only in cal_tri_data - it picks a "nearest
    channel" purely for a debug log line, while the actual calibration averages over every
    channel whose trigger bit is set, regardless of distance. Not carried over here; a clean
    rewrite only needs the trigger bitmask.
    """
    return [ch for ch in range(pres_cnt) if tri_chs_bitmask & (1 << ch)]


def filter_pressure_series(raw_values, use_adc, acq_ms, hftr_cut, lftr_k1):
    """z-score outlier rejection + high-pass + low-pass filter (cal_tri_data/filter_datas_prtouch
    math, ANALYSIS.md sec 4) - matches the firmware's own parallel computation so host and MCU
    agree on the same trigger tick. Strain-gauge sensors (use_adc=False) get all three stages;
    ADC/piezo sensors (use_adc=True) get only the low-pass stage, matching the original exactly.
    """
    values = list(raw_values)
    n = len(values)
    if n < 3:
        return values

    if not use_adc:
        # 1. z-score outlier rejection (threshold=2): replace an outlier sample with its
        # predecessor. Boundary samples (first/last two) are left untouched, matching the
        # original's range(1, n-2).
        mean = sum(values) / n
        variance = sum((v - mean) ** 2 for v in values) / n
        std = math.sqrt(variance)
        if std > 0:
            for i in range(1, n - 2):
                if abs(values[i] - mean) / std > 2:
                    values[i] = values[i - 1]

        # 2. High-pass filter (single-pole, matches the firmware's own filter so host and MCU
        # agree on tick timing).
        rc = 1. / (2. * math.pi * hftr_cut)
        coff = rc / (rc + 1. / (1000. / acq_ms))
        hp = [0.0]
        for i in range(1, n):
            hp.append((values[i] - values[i - 1] + hp[-1]) * coff)
        values = hp

    # 3. Low-pass filter (applied regardless of sensor type).
    for i in range(1, n):
        values[i] = values[i - 1] * (1 - lftr_k1) + values[i] * lftr_k1
    return values


def find_trigger_index(filtered_values):
    """Locate the trigger sample index within a filtered pressure series.

    The normalize-to-[0,1] -> tilt-angle -> rotate -> take-minimum trick (cal_tri_data,
    ANALYSIS.md sec 4): flattens slow signal drift across the probe window so the real trigger
    dip is findable as a global minimum even when the whole series is trending up or down.
    """
    n = len(filtered_values)
    min_val, max_val = min(filtered_values), max(filtered_values)
    span = max_val - min_val
    if span <= 0:
        # Flat signal (e.g. a disconnected/stuck sensor) - no meaningful trigger dip exists.
        # The original divides by this span unconditionally and would raise ZeroDivisionError;
        # env_self_check() is meant to catch a stuck sensor before this point is ever reached,
        # but that self-test is deliberately out of scope for v1 (ANALYSIS.md sec 7/8), so this
        # guards the same failure mode the self-test would have caught.
        return n - 1
    normalized = [(v - min_val) / span for v in filtered_values]
    angle = math.atan((normalized[-1] - normalized[0]) / n)
    sin_a, cos_a = math.sin(-angle), math.cos(-angle)
    rotated = [i * sin_a + normalized[i] * cos_a for i in range(n)]
    return rotated.index(min(rotated))


def interpolate_trigger_step(step_ticks, step_values, trigger_tick):
    """Linear-interpolate the step-buffer position at the pressure trigger tick.

    Matches cal_tri_data's step-side half exactly: find the two step samples straddling
    trigger_tick, interpolate between them; fall back to the last sample if the trigger tick
    lands outside the step buffer's own window (mirrors the original's default-to-last-sample
    behavior when no straddling pair is found, and its "step_d_buf[i] != 0" guard against
    matching a zero-step point sitting at the ramp's very start).
    """
    n = len(step_ticks)
    step_tri_index = n - 1
    step_tri_tick = step_ticks[-1]
    for i in range(n - 1):
        if step_values[i] != 0 and (
                step_ticks[i] <= trigger_tick <= step_ticks[i + 1]
                or step_ticks[i] == trigger_tick):
            step_tri_index = i
            step_tri_tick = step_ticks[i]
            break

    out_step = step_values[-1]
    if 0 < step_tri_index < n - 1:
        denom = step_ticks[step_tri_index + 1] - step_ticks[step_tri_index]
        frac = (trigger_tick - step_tri_tick) / denom if denom else 0.0
        out_step = step_values[step_tri_index] + (
            step_values[step_tri_index + 1] - step_values[step_tri_index]) * frac
    return out_step


def compute_trigger_z(step_samples, pres_samples, step_tri_time, pres_tri_time,
                       tri_chs_bitmask, start_step, start_pos_z, mm_per_step,
                       use_adc, acq_ms, hftr_cut, lftr_k1, pres_cnt=4, z_offset=0.0):
    """Top-level cal_tri_data() replacement - one call per probe cycle.

    step_samples/pres_samples are the raw buffers from PrtouchMCU.collect_step_samples()/
    collect_pres_samples() (list of dicts with 'tick'/'step' or 'tick'/'ch0'..'ch3'). Averages
    the resulting Z across every channel that reported a trigger; raises ValueError if none did
    (the MCU-side trigger detection, ANALYSIS.md sec 2, found nothing - a real no-trigger
    condition the caller should treat as a failed probe attempt, not silently accept).
    """
    valid_channels = select_valid_channels(tri_chs_bitmask, pres_cnt)
    if not valid_channels:
        raise ValueError("no pressure channel reported a trigger (tri_chs=0x%x)"
                          % tri_chs_bitmask)

    step_ticks = [s['tick'] - step_tri_time for s in step_samples]
    step_values = [s['step'] for s in step_samples]

    results = []
    for ch in valid_channels:
        pres_ticks = [p['tick'] - pres_tri_time for p in pres_samples]
        raw_ch = [p['ch%d' % ch] for p in pres_samples]
        filtered = filter_pressure_series(raw_ch, use_adc, acq_ms, hftr_cut, lftr_k1)
        trigger_index = find_trigger_index(filtered)
        trigger_tick = pres_ticks[trigger_index]
        out_step = interpolate_trigger_step(step_ticks, step_values, trigger_tick)
        trigger_z = (start_step - out_step) * mm_per_step
        results.append(start_pos_z - trigger_z + z_offset)

    return sum(results) / len(results)
