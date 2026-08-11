# Load-cell safety hardening mission (2026-08-09) - tests for the guards added this mission
# on top of the pre-existing offline audit (test_prtouch_orchestration.py already covers
# no-trigger recovery, retry bounds, and cleanup-on-exception; this file covers what's new):
#   - max_probe_travel_mm / max_probe_duration_s: hard ceilings independent of a caller's own
#     down_min_z, checked before any motion is ever armed / across an entire retry sequence.
#   - the pre-motion baseline guard (_evaluate_baseline/_check_baseline_safe): invalid/non-
#     finite/saturated sensor data, and the OPT-IN "already triggered before movement"
#     baseline_reference/baseline_deviation_max check.
#   - read_diagnostics()/get_status(): zero-motion, cached, never itself touches the MCU.
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_probe_safety_hardening -v (this fork's own layout - klippy/extras/
# is a real Python package named 'extras', not 'klippy_extras' - see NebulaOS-firmware's
# klippy_extras/ mirror of this same file for that repo's own invocation form)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import atexit
import json
import os
import shutil
import tempfile
import unittest

from . import prtouch_probe
from . import prtouch_test_support as fake
from . import prtouch_v2

_TEMP_DIRS = []
atexit.register(lambda: [shutil.rmtree(d, ignore_errors=True) for d in _TEMP_DIRS])


def _build(prtouch_overrides=None, prime_trusted_baseline=True):
    """prime_trusted_baseline=True (the default) immediately confirms a TRUSTED_REFERENCE from
    the default -250000 mock reading before returning, so every test written before the
    2026-08-12 three-state model (NO_REFERENCE/BOOTSTRAP_CANDIDATE/TRUSTED_REFERENCE - see
    prtouch_probe.py's own __init__ comment) - which assumes touch_probe() can proceed past the
    baseline guard on a freshly-built probe - keeps working unchanged. Tests that specifically
    exercise the bootstrap/candidate/confirm mechanics themselves (PersistedBaselineGuardTest,
    parts of SensorConsistencyGuardTest) pass False to see the real NO_REFERENCE starting
    state."""
    # Every test gets its own throwaway baseline_persist_path unless it explicitly overrides
    # one (see PersistedBaselineGuardTest, which deliberately reuses one path across multiple
    # _build() calls to simulate separate Klipper sessions reading the same file) - this keeps
    # ordinary tests isolated from each other and from whatever the real default
    # (/opt/printer_data/prtouch_baseline.json) resolves to on the machine running these
    # tests, which may not exist/be writable here at all.
    if prtouch_overrides is None or 'baseline_persist_path' not in prtouch_overrides:
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        overrides = dict(prtouch_overrides or {})
        overrides['baseline_persist_path'] = os.path.join(tmp_dir, 'prtouch_baseline.json')
        prtouch_overrides = overrides
    printer, mcu, pins, values = fake.build_environment(prtouch_overrides)
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    mcu.set_query_response('deal_avgs_prtouch',
                            {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
    if prime_trusted_baseline:
        pv2.probe.check_sensor_consistency()
        pv2.probe.confirm_bootstrap_baseline()

    # Every no-trigger/guard-failure path in these tests ends in _fail()'s own safety lift
    # (safe_move_z), which - like any real probe attempt that never fills its buffer - falls
    # through to the manual_get_steps/manual_get_pres repair-query path. Generic zero-filled
    # repair data, same convention as test_prtouch_orchestration.py's own _build().
    def _step_repair(call):
        i = call.args[1]
        return {'oid': pv2.mcu.step_oid, 'index': i, 'tri_time': 0,
                'tick0': i * 100, 'tick1': (i + 1) * 100, 'tick2': (i + 2) * 100,
                'tick3': (i + 3) * 100, 'step0': i, 'step1': i + 1, 'step2': i + 2,
                'step3': i + 3}

    def _pres_repair(call):
        i = call.args[1]
        return {'oid': pv2.mcu.pres_oid, 'index': i, 'tri_time': 0, 'tri_chs': 0, 'buf_cnt': 32,
                'tick_0': i * 100, 'ch0_0': 0, 'ch1_0': 0, 'ch2_0': 0, 'ch3_0': 0,
                'tick_1': (i + 1) * 100, 'ch0_1': 0, 'ch1_1': 0, 'ch2_1': 0, 'ch3_1': 0}

    mcu.set_query_response('manual_get_steps', _step_repair)
    mcu.set_query_response('manual_get_pres', _pres_repair)
    return printer, mcu, pv2


class MaxTravelGuardTest(unittest.TestCase):
    def test_down_min_z_over_ceiling_is_rejected_before_any_motion(self):
        _, mcu, pv2 = _build()
        pv2.probe.max_probe_travel_mm = 10.0
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2.probe.touch_probe(50.0, retries=1, pro_cnt=1)
        # nothing armed at all - no start_step_prtouch call of any kind, not even a stop.
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_down_min_z_at_or_under_ceiling_is_allowed_through_the_guard(self):
        # doesn't need to succeed a full probe (no trigger armed here) - only needs to prove
        # the travel guard itself did not block it, i.e. real motion was actually attempted.
        _, mcu, pv2 = _build()
        pv2.probe.max_probe_travel_mm = 10.0
        with self.assertRaises(Exception):  # will fail later, on no-trigger exhaustion
            pv2.probe.touch_probe(5.0, retries=1, pro_cnt=1)
        self.assertTrue(mcu.all_calls('start_step_prtouch'),
                         "a within-ceiling request must actually attempt motion")

    def test_config_default_matches_z_compensate_own_maxval(self):
        # z_offset_down_min_z's own config bound (z_compensate.py) has maxval=50 - the shared
        # PrtouchProbe default must not be tighter than that, or a real, already-validated
        # z_compensate config could be rejected by this OTHER layer's own default.
        _, _, pv2 = _build()
        self.assertGreaterEqual(pv2.probe.max_probe_travel_mm, 50.0)


class MaxDurationGuardTest(unittest.TestCase):
    def test_exceeding_total_duration_stops_retrying_even_with_retries_remaining(self):
        _, mcu, pv2 = _build()
        pv2.probe.max_probe_duration_s = 0.0  # already "expired" before the first attempt completes
        with self.assertRaises(Exception) as ctx:
            pv2.probe.touch_probe(1.0, retries=50, pro_cnt=1)
        self.assertIn("max_probe_duration_s", str(ctx.exception))
        # far fewer than 50 attempts actually happened.
        down_arms = [c for c in mcu.all_calls('start_step_prtouch')
                     if c.by_field['step_cnt'] > 0 and c.by_field['dir'] == 0]
        self.assertLess(len(down_arms), 5)


class InvalidSensorDataGuardTest(unittest.TestCase):
    def test_non_finite_channel_blocks_before_motion(self):
        _, mcu, pv2 = _build()
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': float('nan'), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_saturated_channel_beyond_max_baseline_abs_blocks_before_motion(self):
        _, mcu, pv2 = _build()
        pv2.probe.max_baseline_abs = 1000.0
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -999999, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_a_plausible_real_hardware_style_baseline_is_allowed_through(self):
        # -250,000-ish is this project's own documented real at-rest reading (NON_MOTION_
        # VALIDATION.md/DESIGN.md) - the default guard must not reject the actual hardware's
        # own normal signal, only genuinely implausible readings.
        _, mcu, pv2 = _build()
        with self.assertRaises(Exception):  # still fails later (no trigger armed), not here
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertTrue(mcu.all_calls('start_step_prtouch'),
                         "a realistic at-rest baseline must not be blocked by the sensor guard")

    def test_mid_sequence_sensor_failure_aborts_remaining_retries(self):
        # first attempt's baseline read is fine (both the pre-motion check and attempt 1's own
        # per-attempt check, each of which now consumes sensor_consistency_reads deal_avgs
        # calls - see check_sensor_consistency()); every deal_avgs call from attempt 2's check
        # onward is bad, simulating a connector working loose mid-sequence - must not be
        # allowed to keep trying on later attempts either (see prtouch_probe.py's own comment).
        _, mcu, pv2 = _build()
        good_calls = 2 * pv2.probe.sensor_consistency_reads  # pre-motion check + attempt 1's check
        calls = {'n': 0}

        def _response(call):
            calls['n'] += 1
            if calls['n'] <= good_calls:
                return {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0}
            return {'oid': pv2.mcu.pres_oid, 'ch0': float('nan'), 'ch1': 0, 'ch2': 0, 'ch3': 0}

        mcu.set_query_response('deal_avgs_prtouch', _response)
        with self.assertRaises(Exception) as ctx:
            pv2.probe.touch_probe(1.0, retries=10, pro_cnt=1)
        self.assertIn("baseline check failed", str(ctx.exception))
        down_arms = [c for c in mcu.all_calls('start_step_prtouch')
                     if c.by_field['step_cnt'] > 0 and c.by_field['dir'] == 0]
        self.assertEqual(len(down_arms), 1, "must stop at the attempt where the sensor went bad")


class AlreadyTriggeredGuardTest(unittest.TestCase):
    """baseline_reference/baseline_deviation_max - opt-in, off by default (see prtouch_probe.py
    __init__'s own comment on why no default reference value is safe to assume)."""

    def test_disabled_by_default_even_with_a_dramatically_different_reading(self):
        _, mcu, pv2 = _build()
        self.assertIsNone(pv2.probe.baseline_reference)
        # isolates the OPT-IN baseline_reference feature specifically (this test's own
        # subject) from the separate, always-on auto-learned TRUSTED_REFERENCE drift check
        # (check_sensor_consistency's own subject, covered elsewhere) - both would otherwise
        # reject a -100000 reading against _build()'s primed -250000 reference.
        pv2.probe.sensor_baseline_max_drift = 1e9
        # even a reading that would obviously look "different" to a human has nothing to be
        # compared against with no reference configured - must not block on that basis alone.
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -100000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        with self.assertRaises(Exception):  # fails later (no trigger), not on this guard
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertTrue(mcu.all_calls('start_step_prtouch'))

    def test_configured_reference_blocks_a_reading_that_deviates_too_far(self):
        _, mcu, pv2 = _build()
        pv2.probe.baseline_reference = [-250000.0, 0.0, 0.0, 0.0]
        pv2.probe.baseline_deviation_max = 1000.0
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -240000, 'ch1': 0, 'ch2': 0, 'ch3': 0})  # 10000 off
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_configured_reference_allows_a_reading_within_tolerance(self):
        _, mcu, pv2 = _build()
        pv2.probe.baseline_reference = [-250000.0, 0.0, 0.0, 0.0]
        pv2.probe.baseline_deviation_max = 5000.0
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -251000, 'ch1': 0, 'ch2': 0, 'ch3': 0})  # 1000 off
        with self.assertRaises(Exception):  # fails later (no trigger), not on this guard
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertTrue(mcu.all_calls('start_step_prtouch'))

    def test_config_requires_both_reference_and_deviation_together(self):
        printer, mcu, pins, values = fake.build_environment(
            {'baseline_reference': '-250000, 0, 0, 0'})
        config = fake.make_prtouch_v2_config(printer, pins, values)
        with self.assertRaises(fake.ConfigError):
            prtouch_v2.PRTouchV2(config)


class SensorConsistencyGuardTest(unittest.TestCase):
    """check_sensor_consistency() - 2026-08-11 physical-qualification closure mission. Live
    testing found a single deal_avgs_prtouch read (the guard above) insufficient: after a raw
    step operation, READ_PRES intermittently returned near-zero/partial-magnitude garbage
    (-1, -63923, -127864, against a real ~-256000 baseline) that is individually finite and
    well under max_baseline_abs, so it passed the single-read guard outright. These tests
    reproduce that exact live pattern against the new multi-read consistency gate."""

    def test_consistent_reads_with_no_trusted_reference_are_a_bootstrap_candidate_not_healthy(self):
        # 2026-08-12 root-cause mission: an internally-consistent reading with nothing trusted
        # to compare against must NOT authorize probing on its own - see
        # confirm_bootstrap_baseline() for the explicit human-driven promotion path.
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        diag = pv2.probe.check_sensor_consistency()
        self.assertFalse(diag['ok'])
        self.assertEqual(diag['state'], 'bootstrap_candidate')
        self.assertIsNone(pv2.probe._auto_baseline)
        self.assertIsNotNone(pv2.probe._bootstrap_candidate)
        self.assertAlmostEqual(pv2.probe._bootstrap_candidate[0], -250000, delta=1)

    def test_confirm_bootstrap_baseline_promotes_candidate_to_trusted_reference(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        self.assertFalse(pv2.probe.check_sensor_consistency()['ok'])
        values = pv2.probe.confirm_bootstrap_baseline()
        self.assertAlmostEqual(values[0], -250000, delta=1)
        self.assertIsNotNone(pv2.probe._auto_baseline)
        self.assertIsNone(pv2.probe._bootstrap_candidate,
                           "the candidate must be cleared once promoted")
        # now genuinely trusted - a matching subsequent read is healthy without confirmation.
        diag = pv2.probe.check_sensor_consistency()
        self.assertTrue(diag['ok'])
        self.assertEqual(diag['state'], 'healthy')

    def test_confirm_bootstrap_baseline_with_no_candidate_raises(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2.probe.confirm_bootstrap_baseline()

    def test_bootstrap_candidate_cannot_authorize_touch_probe(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        self.assertFalse(pv2.probe.check_sensor_consistency()['ok'])
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertIn('PRTOUCH_CONFIRM_BASELINE', str(ctx.exception))
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [],
                          "an unconfirmed bootstrap candidate must never authorize real motion")

    def test_flickering_reads_are_rejected_as_unstable(self):
        # the exact live-observed pattern: individually-plausible values that disagree wildly
        # with each other from one read to the next within a single check.
        _, mcu, pv2 = _build()  # primed with a -250000 TRUSTED_REFERENCE
        baseline_before = list(pv2.probe._auto_baseline)
        it = iter([-1, -127864, -63923])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            lambda call: {'oid': pv2.mcu.pres_oid, 'ch0': next(it), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2.probe.check_sensor_consistency()
        self.assertFalse(diag['ok'])
        self.assertEqual(diag['state'], 'unstable')
        self.assertIn('disagree', diag['reason'])
        self.assertEqual(pv2.probe._auto_baseline, baseline_before,
                          "a rejected batch must never establish/poison the trusted reference")

    def test_individually_bad_read_is_rejected_as_corrupted_not_unstable(self):
        _, mcu, pv2 = _build()
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': float('nan'), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2.probe.check_sensor_consistency()
        self.assertFalse(diag['ok'])
        self.assertEqual(diag['state'], 'corrupted')

    def test_stable_but_drifted_reading_is_rejected_once_a_baseline_is_established(self):
        # internally consistent (zero spread) but far from what this session already learned
        # to trust - the spread check alone cannot catch this, only the drift check can.
        # _build()'s default priming already establishes a TRUSTED_REFERENCE at -250000.
        _, mcu, pv2 = _build()

        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -100000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        second = pv2.probe.check_sensor_consistency()
        self.assertFalse(second['ok'])
        self.assertEqual(second['state'], 'unstable')
        self.assertIn('drifted', second['reason'])

    def test_recovering_to_a_genuinely_consistent_reading_is_healthy_again(self):
        # a rejected check must not be a permanent lockout - once real, consistent, plausible
        # data (matching the already-trusted reference) comes back, the guard must accept it
        # again. _build()'s default priming already establishes a TRUSTED_REFERENCE at -250000.
        _, mcu, pv2 = _build()

        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': float('nan'), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        self.assertFalse(pv2.probe.check_sensor_consistency()['ok'])

        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        good = pv2.probe.check_sensor_consistency()
        self.assertTrue(good['ok'])
        self.assertEqual(good['state'], 'healthy')

    def test_touch_probe_refuses_to_arm_any_motion_on_an_unstable_sensor(self):
        _, mcu, pv2 = _build()
        it = iter([-1, -127864, -63923])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            lambda call: {'oid': pv2.mcu.pres_oid, 'ch0': next(it), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertIn('unstable', str(ctx.exception))
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [],
                          "an unstable sensor must block motion just like a corrupted one")

    def test_get_status_exposes_sensor_state(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        pv2.probe.check_sensor_consistency()
        status = pv2.get_status(0.0)
        self.assertEqual(status['sensor_state'], 'bootstrap_candidate')
        self.assertFalse(status['sensor_has_trusted_reference'])
        self.assertTrue(status['sensor_bootstrap_candidate_pending'])
        pv2.probe.confirm_bootstrap_baseline()
        pv2.probe.check_sensor_consistency()
        status = pv2.get_status(0.0)
        self.assertEqual(status['sensor_state'], 'healthy')
        self.assertTrue(status['sensor_has_trusted_reference'])
        self.assertFalse(status['sensor_bootstrap_candidate_pending'])

    def test_default_thresholds_tolerate_this_projects_own_documented_real_idle_noise(self):
        # real healthy back-to-back reads captured live this session spread across ~300 counts
        # (-255752..-256031) - the default guard must not reject the actual hardware's own
        # normal noise floor, once there's a trusted reference to check drift against.
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        pv2.probe.sensor_consistency_reads = 4
        it = iter([-255978, -256031, -255855, -255752, -255978, -256031, -255855, -255752])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            lambda call: {'oid': pv2.mcu.pres_oid, 'ch0': next(it), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        pv2.probe.check_sensor_consistency()
        pv2.probe.confirm_bootstrap_baseline()
        diag = pv2.probe.check_sensor_consistency()
        self.assertTrue(diag['ok'], diag['reason'])
        self.assertEqual(diag['state'], 'healthy')


class PersistedBaselineGuardTest(unittest.TestCase):
    """check_sensor_consistency()'s persisted baseline (2026-08-12 root-cause mission) - closes
    a real gap in the original 2026-08-11 session-local-only version: a corrupted-but-stable
    sensor reading present when Klipper restarts had nothing persisted to compare against, so
    it could get learned as the new 'healthy' baseline. Each _build() call here creates a
    genuinely fresh PrtouchProbe/PRTouchV2 instance - the same object construction a real
    Klipper restart goes through - so reusing one baseline_persist_path across two _build()
    calls is a faithful simulation of two separate Klipper sessions on the same printer."""

    def test_trusted_reference_survives_a_simulated_klipper_restart(self):
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')

        _, mcu1, pv2_1 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        self.assertEqual(pv2_1.probe.check_sensor_consistency()['state'], 'bootstrap_candidate')
        pv2_1.probe.confirm_bootstrap_baseline()
        self.assertTrue(os.path.exists(path))

        # A second, independent instance sharing only the persisted file - not the first
        # instance's Python object - is the real test of "survives a restart".
        _, mcu2, pv2_2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        self.assertIsNotNone(pv2_2.probe._auto_baseline,
                              "a fresh instance must load the persisted TRUSTED_REFERENCE on "
                              "connect, not start blank like session-local-only baselines did")
        self.assertAlmostEqual(pv2_2.probe._auto_baseline[0], -250000, delta=1)
        second = pv2_2.probe.check_sensor_consistency()
        self.assertEqual(second['state'], 'healthy',
                          "a restarted session with an existing TRUSTED_REFERENCE must not "
                          "need re-confirmation - only the very first establishment does")

    def test_corrupted_reading_after_restart_does_not_silently_become_the_new_reference(self):
        # The exact failure mode this mission was asked to close: sensor is corrupted (but
        # internally self-consistent) at the moment a new session starts, with nothing
        # session-local to compare against - only the file persisted by the PRIOR confirmed
        # session can catch it.
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')

        _, mcu1, pv2_1 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        pv2_1.probe.check_sensor_consistency()
        pv2_1.probe.confirm_bootstrap_baseline()
        with open(path) as f:
            persisted_before = json.load(f)

        _, mcu2, pv2_2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        mcu2.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2_2.mcu.pres_oid, 'ch0': -1, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2_2.probe.check_sensor_consistency()
        self.assertFalse(diag['ok'])
        self.assertEqual(diag['state'], 'unstable')

        with open(path) as f:
            persisted_after = json.load(f)
        self.assertEqual(persisted_before, persisted_after,
                          "a corrupted-but-stable reading must never overwrite the persisted "
                          "TRUSTED_REFERENCE, even across a simulated restart")

        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
            pv2_2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertEqual(mcu2.all_calls('start_step_prtouch'), [])

    def test_a_corrupted_sensor_present_since_the_very_first_boot_never_self_promotes(self):
        # The scenario the 3-state model (NO_REFERENCE/BOOTSTRAP_CANDIDATE/TRUSTED_REFERENCE)
        # exists for: a sensor that is corrupted-but-stable from the very first boot (no prior
        # good session ever ran) would pass every self-consistency check, every single restart,
        # forever - only an explicit human confirm can ever trust it, and a machine that never
        # gets one must stay refused indefinitely, not eventually "time out" into trusted.
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')
        for _ in range(5):  # simulate 5 separate restarts, all still on the fresh/never-confirmed
            _, mcu, pv2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
            diag = pv2.probe.check_sensor_consistency()
            self.assertEqual(diag['state'], 'bootstrap_candidate')
            self.assertFalse(diag['ok'])
            with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError):
                pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        self.assertFalse(os.path.exists(path),
                          "a never-confirmed candidate must never reach disk at all")

    def test_missing_persistent_reference_creates_a_bootstrap_candidate_not_a_trusted_one(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)  # fresh per-test tmp dir, no file yet
        self.assertIsNone(pv2.probe._auto_baseline)
        diag = pv2.probe.check_sensor_consistency()
        self.assertEqual(diag['state'], 'bootstrap_candidate')
        self.assertFalse(diag['ok'])
        self.assertIsNotNone(pv2.probe._bootstrap_candidate)

    def test_corrupt_persisted_file_is_treated_as_no_reference_not_a_crash(self):
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')
        with open(path, 'w') as f:
            f.write('{not valid json')

        _, mcu, pv2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        self.assertIsNone(pv2.probe._auto_baseline)
        diag = pv2.probe.check_sensor_consistency()
        self.assertEqual(diag['state'], 'bootstrap_candidate')

    def test_malformed_baseline_field_is_treated_as_no_reference_not_a_crash(self):
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')
        with open(path, 'w') as f:
            json.dump({'baseline': 'not a list'}, f)

        _, mcu, pv2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        self.assertIsNone(pv2.probe._auto_baseline)
        self.assertEqual(pv2.probe.check_sensor_consistency()['state'], 'bootstrap_candidate')

    def test_normal_drift_within_tolerance_updates_the_persisted_reference(self):
        tmp_dir = tempfile.mkdtemp(prefix='prtouch_baseline_test_')
        _TEMP_DIRS.append(tmp_dir)
        path = os.path.join(tmp_dir, 'prtouch_baseline.json')

        _, mcu, pv2 = _build({'baseline_persist_path': path}, prime_trusted_baseline=False)
        pv2.probe.check_sensor_consistency()
        pv2.probe.confirm_bootstrap_baseline()
        with open(path) as f:
            first_value = json.load(f)['baseline'][0]
        self.assertAlmostEqual(first_value, -250000, delta=1)

        # within sensor_baseline_max_drift (default 10000) - legitimate slow drift, not a fault.
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -255000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2.probe.check_sensor_consistency()
        self.assertEqual(diag['state'], 'healthy')
        with open(path) as f:
            second_value = json.load(f)['baseline'][0]
        self.assertAlmostEqual(second_value, -255000, delta=1,
                                msg="normal in-tolerance drift must still update the persisted "
                                    "reference, not freeze it forever at the confirmed value")

    def test_get_status_exposes_trusted_reference_and_candidate_pending_flags(self):
        _, mcu, pv2 = _build(prime_trusted_baseline=False)
        pv2.probe.check_sensor_consistency()
        status = pv2.get_status(0.0)
        self.assertFalse(status['sensor_has_trusted_reference'])
        self.assertTrue(status['sensor_bootstrap_candidate_pending'])
        pv2.probe.confirm_bootstrap_baseline()
        status = pv2.get_status(0.0)
        self.assertTrue(status['sensor_has_trusted_reference'])
        self.assertFalse(status['sensor_bootstrap_candidate_pending'])


class DiagnosticsAreZeroMotionAndCachedTest(unittest.TestCase):
    def test_read_diagnostics_never_sends_a_step_command(self):
        _, mcu, pv2 = _build()
        pv2.probe.read_diagnostics()
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_get_status_before_any_reading_reports_no_reading_taken_yet_not_a_fabricated_value(self):
        _, _, pv2 = _build(prime_trusted_baseline=False)
        status = pv2.get_status(0.0)
        self.assertIsNone(status['sensor_ok'])
        self.assertIsNone(status['raw'])
        self.assertIn("no reading taken yet", status['sensor_reason'])

    def test_get_status_never_itself_triggers_an_mcu_read(self):
        _, mcu, pv2 = _build()
        pv2.probe.read_diagnostics()  # populate the cache once
        calls_before = len(mcu.all_calls('deal_avgs_prtouch'))
        for _ in range(5):
            pv2.get_status(0.0)
        self.assertEqual(len(mcu.all_calls('deal_avgs_prtouch')), calls_before,
                          "get_status() must be a pure cache read, never a fresh MCU query")

    def test_get_status_reflects_the_most_recent_real_reading(self):
        _, mcu, pv2 = _build()
        pv2.probe.read_diagnostics()
        self.assertTrue(pv2.get_status(0.0)['sensor_ok'])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': float('nan'), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        pv2.probe.read_diagnostics()
        status = pv2.get_status(0.0)
        self.assertFalse(status['sensor_ok'])
        self.assertIn("non-finite", status['sensor_reason'])

    def test_read_pres_gcode_updates_the_same_cache_get_status_reads(self):
        _, mcu, pv2 = _build()
        gcmd = fake.FakeGCmd()
        pv2.cmd_READ_PRES(gcmd)
        self.assertTrue(pv2.get_status(0.0)['sensor_ok'])
        self.assertIn("ch0=-250000", gcmd.responses[0])

    def test_a_real_touch_probe_attempt_also_updates_the_cache(self):
        _, mcu, pv2 = _build()
        with self.assertRaises(Exception):  # no trigger armed - expected to fail
            pv2.probe.touch_probe(1.0, retries=1, pro_cnt=1)
        status = pv2.get_status(0.0)
        self.assertIsNotNone(status['raw'], "a real probe attempt's own baseline read must "
                              "populate the diagnostic cache too, not just explicit READ_PRES")


if __name__ == '__main__':
    unittest.main()
