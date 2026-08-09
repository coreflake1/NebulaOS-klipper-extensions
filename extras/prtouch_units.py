# Unit-conversion helpers for the prtouch_v2/z_compensate wire protocol.
#
# Every one of these was previously inline arithmetic, scattered across prtouch_probe.py
# (mm/speed -> MCU step-timing fields) and prtouch_mcu.py (four separate, independently
# duplicated `/ 10000.` tick-to-second conversions in the async response handlers and both
# _repair_*_samples methods - a real "same conversion written out four times" duplication
# flagged during the 2026-08-06 non-motion audit). Extracted here, with explicit units in
# every name, so each conversion can be tested once in isolation instead of only implicitly
# via the orchestration layer.
#
# Behavior is unchanged - every call site was verified byte-for-byte against its prior
# inline form before being replaced (see test_prtouch_units.py).
#
# This file may be distributed under the terms of the GNU GPLv3 license.

#: The real device's async response ticks arrive as MCU ticks scaled by this factor
#: (confirmed against every one of the reference's own equivalent handlers - every
#: `params['tri_time'] / 10000` in reference/prtouch_v2_wrapper.py uses this exact divisor,
#: e.g. its manual_get_steps handling at line 621).
MCU_TICK_SCALE = 10000.

#: start_pres_prtouch's tri_hftr_cut/tri_lftr_k1 fields are floats sent as fixed-point
#: integers scaled by this factor (confirmed against reference lines 1029-1030's own
#: `int(use_tri_hftr_cut * 1000)` / `int(use_tri_lftr_k1 * 1000)`).
FIXED_POINT_SCALE = 1000


def mcu_ticks_to_seconds(ticks):
    """Async response 'tri_time'/'tick*' fields (raw MCU tick counts) -> seconds."""
    return ticks / MCU_TICK_SCALE


def seconds_to_mcu_ticks(seconds):
    """Inverse of mcu_ticks_to_seconds - not currently sent anywhere (the host never
    constructs a tick value, only ever consumes one), provided for symmetry/testability."""
    return seconds * MCU_TICK_SCALE


def to_fixed_point(value, scale=FIXED_POINT_SCALE):
    """A float config value (e.g. tri_hftr_cut=2.0, tri_lftr_k1=0.7) -> the integer field
    start_pres_prtouch actually sends over the wire."""
    return int(value * scale)


def distance_mm_to_step_count(distance_mm, mm_per_step):
    """How many whole MCU step pulses cover a given distance. Truncates (not rounds) -
    matches the reference's own `int(run_dis / self.mm_per_step)` exactly (get_step_cnts,
    reference line 767)."""
    return int(distance_mm / mm_per_step)


def step_count_to_step_us(distance_mm, speed_mm_s, step_count):
    """Per-step pulse period (microseconds) needed to cover distance_mm at speed_mm_s in
    exactly step_count pulses. Caller must guard step_count == 0 (division by zero) -
    matches the reference's own get_step_cnts, which returns (0, 0, 0) for that case rather
    than calling this at all."""
    return int((distance_mm / speed_mm_s) * 1000. * 1000. / step_count)


def distance_mm_to_acc_ctl_cnt(acc_ctl_mm, mm_per_step):
    """Acceleration-window distance (mm) -> step-count field start_step_prtouch's
    acc_ctl_cnt expects."""
    return int(acc_ctl_mm / mm_per_step)


def step_count_to_distance_mm(step_count, mm_per_step):
    """Inverse of distance_mm_to_step_count - steps actually reported back by the MCU
    (e.g. step_samples[-1]['step']) -> a physical distance."""
    return step_count * mm_per_step


def duty_fraction_to_scaled_units(duty_fraction, scale=100000):
    """config_step_prtouch/config_pres_prtouch's sys_time_duty field: a small fraction
    (config default 0.001) scaled by 100000 into an integer - a distinct scale from
    FIXED_POINT_SCALE (1000), kept as its own named conversion rather than overloading
    to_fixed_point with a caller-supplied scale that could silently drift from either."""
    return int(duty_fraction * scale)


def probe_timeout_seconds(distance_mm, speed_mm_s, margin_s=2.0):
    """How long the host should wait for a probe cycle's response buffers to fill before
    giving up - matches prtouch_probe.py's own `down_min_z / self.tri_z_down_spd + 2.0` and
    the reference's identical `down_min_z / use_tri_z_down_spd + 2` (run_step_prtouch)."""
    return distance_mm / speed_mm_s + margin_s
