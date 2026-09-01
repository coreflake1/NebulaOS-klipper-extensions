# NebulaOS automatic probe/nozzle pairing primitive (Phase 2 calibration-
# framework mission; bounded-descent contact-safety rewrite).
#
# The shared math behind NEBULAOS_Z_OFFSET_CALIBRATE METHOD=LOAD_CELL: at a
# physical bed point P, take a FRESH automatic BLTouch probe reading (this
# doubles as the locally-expected surface Z the bounded-descent envelope is
# derived from), then move the actual nozzle over the exact same P and take
# one or more validated, envelope-bounded HX711 contact readings, and
# derive the probe's true Z-offset from the two raw readings.
#
# Derived directly from pinned upstream Klipper (58bd67db...)'s own
# arithmetic, not invented: klippy/extras/probe.py's
# ProbeCommandHelper.probe_calibrate_finalize() computes
#     z_offset = offsets[2] - mpresult.bed_z + ppos.bed_z
# where ppos.bed_z = raw_trigger_z - current_z_offset (manual_probe.py's
# create_probe_result) and mpresult.bed_z is the RAW toolhead Z a human (or,
# here, the load cell) accepts as nozzle-bed contact. Substituting and
# simplifying, the current z_offset cancels algebraically:
#     z_offset_new = raw_probe_trigger_z - raw_nozzle_contact_z
# both measured in the SAME toolhead-Z coordinate frame. Because both
# readings are taken back-to-back in whatever gcode-offset frame happens to
# be active at the time, any currently-active SET_GCODE_OFFSET Z also
# cancels the same way - this result does not depend on it.
#
# Also independent of [axis_twist_compensation]'s state, whether or not
# that section is configured: axis twist's own _update_z_compensation_value
# (klippy/extras/axis_twist_compensation.py) only ever corrects a
# ProbeResult's .bed_z field, never .test_z (the raw trigger position this
# module reads) - confirmed directly from the pinned source, not assumed.
# ZOffsetProbe.touch_probe() bypasses the probe:update_results event
# entirely (see nebulaos_z_offset_probe.py's own header), so it was never
# affected either. There is therefore no need to clear/disable axis twist
# before measuring a Z-offset with this primitive.
#
# ---------------------------------------------------------------------
# Bounded-descent contact-safety envelope (Phase 2 mission)
# ---------------------------------------------------------------------
# The real hardware incident this rewrite is built on (overnight HX711
# investigation, 2026-08-31): nebulaos_z_offset_probe.py computed
# last_fit_delta but never validated it, and the force-trigger threshold
# alone does not bound how deep the toolhead is ever COMMANDED to move -
# by the time a bad fit is detected, the descent has already happened.
#
# This module now derives, for every physical point, a hard floor the
# actual probing_move() target can never exceed:
#     minimum_allowed_z = predicted_surface_z - max_contact_descent_mm
# where predicted_surface_z is the SAME fresh CR-Touch reading already
# taken for the probe/nozzle pair (raw_probe_trigger_z) - the two
# measurements are taken at the identical physical XY, so it is a
# reasonable, real, freshly-measured local reference, not a cached or
# nominal one. max_contact_descent_mm is read from the ZOffsetProbe
# instance's own config (nebulaos_z_offset_probe.py); when it is not
# configured (not yet hardware-qualified), this function refuses to
# command ANY nozzle contact motion at all, failing closed with a
# CONTACT_SAFETY_LIMIT_UNQUALIFIED error - see that module's own comment
# for why no invented default is used. The same fail-closed treatment
# applies to max_abs_fit_delta and, when pro_cnt>1, the repeatability
# bounds (min_accepted_samples/max_repeatability_range/
# max_repeatability_stddev) - all caller-supplied, all optional, all
# unqualified-safe.
#
# Every individual contact is recorded as a ContactSample - accepted or
# rejected, always appended, never silently dropped - and the aggregate
# RepeatabilityResult/PairedMeasurement.accepted flag is the ONLY thing a
# caller may use to decide whether to stage a Z-offset, transition to
# COMPLETE, or call SAVE_CONFIG (see nebulaos_calibration.py).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import math

from . import probe as probe_module

ContactSample = collections.namedtuple('ContactSample', [
    'sample_index', 'starting_z', 'commanded_floor_z',
    'raw_trigger_z', 'fitted_contact_z', 'fit_delta',
    'fit_valid', 'motion_safe', 'accepted', 'rejection_reason'])

RepeatabilityResult = collections.namedtuple('RepeatabilityResult', [
    'sample_count', 'accepted_count', 'samples',
    'mean', 'minimum', 'maximum', 'range', 'stddev',
    'accepted', 'rejection_reason'])

PairedMeasurement = collections.namedtuple('PairedMeasurement', [
    'x', 'y', 'contact_id',
    'predicted_surface_z', 'commanded_floor_z',
    'raw_probe_trigger_z', 'raw_nozzle_contact_z',
    'repeatability', 'probe_z_offset',
    'accepted', 'rejection_reason'])


def _require_safety_limits(printer, z_offset_probe, x, y, pro_cnt,
                            max_abs_fit_delta, min_accepted_samples,
                            max_repeatability_range,
                            max_repeatability_stddev):
    """Fail-closed preflight: refuses ANY nozzle contact motion (called
    before the nozzle is ever moved toward the bed) unless every
    safety-critical constant this run actually needs is configured. No
    invented production defaults - see nebulaos_z_offset_probe.py's own
    comment on max_contact_descent_mm."""
    name = getattr(z_offset_probe, '_name', 'nebulaos_z_offset_probe')
    max_contact_descent_mm = getattr(z_offset_probe, 'max_contact_descent_mm', None)
    if max_contact_descent_mm is None:
        raise printer.command_error(
            "CONTACT_SAFETY_LIMIT_UNQUALIFIED: max_contact_descent_mm is "
            "not configured on [%s] - the bounded-descent envelope cannot "
            "be computed, refusing nozzle contact motion at X=%.3f Y=%.3f"
            % (name, x, y))
    if max_abs_fit_delta is None:
        raise printer.command_error(
            "CONTACT_SAFETY_LIMIT_UNQUALIFIED: max_abs_fit_delta is not "
            "configured - the measurement-quality gate cannot be applied, "
            "refusing nozzle contact motion at X=%.3f Y=%.3f" % (x, y))
    if pro_cnt > 1 and (min_accepted_samples is None
                         or max_repeatability_range is None
                         or max_repeatability_stddev is None):
        raise printer.command_error(
            "CONTACT_SAFETY_LIMIT_UNQUALIFIED: repeatability validation "
            "bounds (min_accepted_samples/max_repeatability_range/"
            "max_repeatability_stddev) are not fully configured for "
            "pro_cnt=%d - refusing nozzle contact motion at X=%.3f Y=%.3f"
            % (pro_cnt, x, y))
    return max_contact_descent_mm


def measure_probe_nozzle_pair(printer, x, y, probe_x_offset, probe_y_offset,
                               horizontal_move_z, z_offset_probe,
                               down_min_z, pro_cnt=1,
                               travel_speed=None, probe_lift_speed=None,
                               max_abs_fit_delta=None,
                               min_accepted_samples=None,
                               max_repeatability_range=None,
                               max_repeatability_stddev=None):
    """Take one bounded, quality-gated, optionally-repeated (probe, nozzle)
    measurement at bed point (x, y). Returns a PairedMeasurement whose
    `accepted` flag is the single authoritative answer to "may a caller
    stage this result" - see this module's own header comment.

    probe_x_offset/probe_y_offset: the registered probe's own configured
    XY offset (e.g. BLTouch's [bltouch] x_offset/y_offset) - the probe TIP
    is moved to (x - probe_x_offset, y - probe_y_offset) so that the probe
    itself ends up physically over (x, y), exactly mirroring upstream
    axis_twist_compensation.py's own _calculate_test_points().

    horizontal_move_z: a safe hover height used for every XY traverse in
    this sequence, so the toolhead never drags either the probe or the
    nozzle across the bed at print height - applied before EVERY XY move,
    not just the first.

    z_offset_probe: a nebulaos_z_offset_probe.ZOffsetProbe instance (or
    anything exposing the same touch_probe(down_min_z, pro_cnt=1,
    minimum_allowed_z=...)/get_status(eventtime) contract) whose own
    max_contact_descent_mm config supplies the bounded-descent envelope.

    down_min_z: the same absolute stepper-limit-relative depth floor
    touch_probe() has always accepted - kept as an independent, coarser
    safety layer underneath the new envelope (see touch_probe()'s own
    comment: the tighter of the two always wins).

    max_abs_fit_delta/min_accepted_samples/max_repeatability_range/
    max_repeatability_stddev: measurement-quality and repeatability
    acceptance bounds (Phase 2 mission, §7/§9/§10) - all optional, all
    fail-closed when a value this call actually needs is missing (see
    _require_safety_limits above). Tests may use synthetic values; no
    production default is invented here.

    travel_speed/probe_lift_speed: motion speeds for the repositioning
    moves; None lets toolhead.manual_move fall back to whatever speed was
    last used. Callers should normally pass explicit values.

    Raises the same exceptions run_single_probe()/touch_probe() would
    raise on a HARD failure path (unhomed axis, sensor error, insufficient
    fit data, no valid trigger before the commanded floor, etc.) -
    deliberately does not catch or reinterpret those; only a QUALITY
    rejection (excessive fit delta, out-of-envelope result, failed
    repeatability) is represented as `accepted=False` in the returned
    PairedMeasurement rather than an exception, since the contact itself
    completed safely and the toolhead is already lifted clear.
    """
    toolhead = printer.lookup_object('toolhead')
    gcode = printer.lookup_object('gcode')
    probe_obj = printer.lookup_object('probe')
    reactor = printer.get_reactor()

    max_contact_descent_mm = _require_safety_limits(
        printer, z_offset_probe, x, y, pro_cnt, max_abs_fit_delta,
        min_accepted_samples, max_repeatability_range,
        max_repeatability_stddev)

    def hover():
        toolhead.manual_move([None, None, horizontal_move_z], probe_lift_speed)

    # Step 1-2: fresh CR-Touch measurement at physical P. This doubles as
    # the locally-expected surface Z the bounded-descent envelope is
    # derived from - the SAME reading, not a second/cached one.
    hover()
    toolhead.manual_move([x - probe_x_offset, y - probe_y_offset, None],
                         travel_speed)
    probe_gcmd = gcode.create_gcode_command("", "", {})
    ppos = probe_module.run_single_probe(probe_obj, probe_gcmd)
    raw_probe_trigger_z = ppos.test_z
    predicted_surface_z = raw_probe_trigger_z

    if not math.isfinite(predicted_surface_z):
        raise printer.command_error(
            "measure_probe_nozzle_pair: CR-Touch reading at X=%.3f Y=%.3f "
            "is not finite (%r) - cannot derive a bounded-descent "
            "envelope, refusing nozzle contact" % (x, y, predicted_surface_z))

    commanded_floor_z = predicted_surface_z - max_contact_descent_mm

    # Step 3-4: lift clear, then move the NOZZLE (toolhead origin, no
    # offset) to the exact same physical (x, y).
    hover()
    toolhead.manual_move([x, y, None], travel_speed)

    contact_id = reactor.monotonic()
    samples = []
    accepted_zs = []
    for i in range(pro_cnt):
        starting_z = toolhead.get_position()[2]
        raw_z = fitted_z = fit_delta = None
        fit_valid = motion_safe = accepted = False
        rejection_reason = None
        try:
            fitted_z = z_offset_probe.touch_probe(
                down_min_z, pro_cnt=1, minimum_allowed_z=commanded_floor_z)
        except Exception as e:
            samples.append(ContactSample(
                sample_index=i, starting_z=starting_z,
                commanded_floor_z=commanded_floor_z,
                raw_trigger_z=None, fitted_contact_z=None, fit_delta=None,
                fit_valid=False, motion_safe=False, accepted=False,
                rejection_reason="contact_error: %s"
                                  % (str(e).strip() or e.__class__.__name__,)))
            raise

        status = z_offset_probe.get_status(reactor.monotonic())
        raw_z = status['last_raw_trigger_z']
        fit_delta = status['last_fit_delta']
        fit_valid = (raw_z is not None and fitted_z is not None
                     and fit_delta is not None
                     and math.isfinite(raw_z) and math.isfinite(fitted_z)
                     and math.isfinite(fit_delta))
        motion_safe = fit_valid and raw_z >= commanded_floor_z - 1e-6
        if not fit_valid:
            rejection_reason = "non_finite_or_missing_fit_result"
        elif not motion_safe:
            rejection_reason = "result_outside_commanded_envelope"
        elif abs(fit_delta) > max_abs_fit_delta:
            rejection_reason = ("excessive_fit_delta(%.4f>%.4f)"
                                 % (abs(fit_delta), max_abs_fit_delta))
        else:
            accepted = True

        samples.append(ContactSample(
            sample_index=i, starting_z=starting_z,
            commanded_floor_z=commanded_floor_z,
            raw_trigger_z=raw_z, fitted_contact_z=fitted_z,
            fit_delta=fit_delta, fit_valid=fit_valid,
            motion_safe=motion_safe, accepted=accepted,
            rejection_reason=rejection_reason))
        if accepted:
            accepted_zs.append(fitted_z)

    # Leave the toolhead lifted clear of the bed before handing control
    # back - the caller may be about to traverse to a different XY point.
    hover()

    valid_count = len(accepted_zs)
    mean = minimum = maximum = rng = stddev = None
    if valid_count:
        mean = sum(accepted_zs) / valid_count
        minimum = min(accepted_zs)
        maximum = max(accepted_zs)
        rng = maximum - minimum
        if valid_count > 1:
            variance = sum((v - mean) ** 2 for v in accepted_zs) / (valid_count - 1)
            stddev = math.sqrt(variance)
        else:
            stddev = 0.0

    repeat_accepted = True
    repeat_rejection = None
    if valid_count == 0:
        repeat_accepted = False
        repeat_rejection = "no_accepted_samples"
    elif min_accepted_samples is not None and valid_count < min_accepted_samples:
        repeat_accepted = False
        repeat_rejection = ("insufficient_accepted_samples(%d<%d)"
                             % (valid_count, min_accepted_samples))
    elif max_repeatability_range is not None and rng is not None \
            and rng > max_repeatability_range:
        repeat_accepted = False
        repeat_rejection = ("repeatability_range_exceeded(%.4f>%.4f)"
                             % (rng, max_repeatability_range))
    elif max_repeatability_stddev is not None and stddev is not None \
            and stddev > max_repeatability_stddev:
        repeat_accepted = False
        repeat_rejection = ("repeatability_stddev_exceeded(%.4f>%.4f)"
                             % (stddev, max_repeatability_stddev))

    repeatability = RepeatabilityResult(
        sample_count=len(samples), accepted_count=valid_count,
        samples=samples, mean=mean, minimum=minimum, maximum=maximum,
        range=rng, stddev=stddev, accepted=repeat_accepted,
        rejection_reason=repeat_rejection)

    probe_z_offset = None
    if repeat_accepted:
        probe_z_offset = raw_probe_trigger_z - mean

    return PairedMeasurement(
        x=x, y=y, contact_id=contact_id,
        predicted_surface_z=predicted_surface_z,
        commanded_floor_z=commanded_floor_z,
        raw_probe_trigger_z=raw_probe_trigger_z,
        raw_nozzle_contact_z=mean,
        repeatability=repeatability,
        probe_z_offset=probe_z_offset,
        accepted=repeat_accepted, rejection_reason=repeat_rejection)
