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
import unittest

from . import prtouch_probe
from . import prtouch_test_support as fake
from . import prtouch_v2


def _build(prtouch_overrides=None):
    printer, mcu, pins, values = fake.build_environment(prtouch_overrides)
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    mcu.set_query_response('deal_avgs_prtouch',
                            {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0})

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

    def test_consistent_reads_are_healthy_and_establish_auto_baseline(self):
        _, mcu, pv2 = _build()
        diag = pv2.probe.check_sensor_consistency()
        self.assertTrue(diag['ok'])
        self.assertEqual(diag['state'], 'healthy')
        self.assertIsNotNone(pv2.probe._auto_baseline)
        self.assertAlmostEqual(pv2.probe._auto_baseline[0], -250000, delta=1)

    def test_flickering_reads_are_rejected_as_unstable(self):
        # the exact live-observed pattern: individually-plausible values that disagree wildly
        # with each other from one read to the next within a single check.
        _, mcu, pv2 = _build()
        it = iter([-1, -127864, -63923])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            lambda call: {'oid': pv2.mcu.pres_oid, 'ch0': next(it), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2.probe.check_sensor_consistency()
        self.assertFalse(diag['ok'])
        self.assertEqual(diag['state'], 'unstable')
        self.assertIn('disagree', diag['reason'])
        self.assertIsNone(pv2.probe._auto_baseline,
                           "a rejected batch must never establish/poison the auto-baseline")

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
        _, mcu, pv2 = _build()
        first = pv2.probe.check_sensor_consistency()
        self.assertEqual(first['state'], 'healthy')

        mcu.set_query_response(
            'deal_avgs_prtouch',
            {'oid': pv2.mcu.pres_oid, 'ch0': -100000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        second = pv2.probe.check_sensor_consistency()
        self.assertFalse(second['ok'])
        self.assertEqual(second['state'], 'unstable')
        self.assertIn('drifted', second['reason'])

    def test_recovering_to_a_genuinely_consistent_reading_is_healthy_again(self):
        # a rejected check must not be a permanent lockout - once real, consistent, plausible
        # data comes back, the guard must accept it again.
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
        _, mcu, pv2 = _build()
        pv2.probe.check_sensor_consistency()
        self.assertEqual(pv2.get_status(0.0)['sensor_state'], 'healthy')

    def test_default_thresholds_tolerate_this_projects_own_documented_real_idle_noise(self):
        # real healthy back-to-back reads captured live this session spread across ~300 counts
        # (-255752..-256031) - the default guard must not reject the actual hardware's own
        # normal noise floor.
        _, mcu, pv2 = _build()
        pv2.probe.sensor_consistency_reads = 4
        it = iter([-255978, -256031, -255855, -255752])
        mcu.set_query_response(
            'deal_avgs_prtouch',
            lambda call: {'oid': pv2.mcu.pres_oid, 'ch0': next(it), 'ch1': 0, 'ch2': 0, 'ch3': 0})
        diag = pv2.probe.check_sensor_consistency()
        self.assertTrue(diag['ok'], diag['reason'])
        self.assertEqual(diag['state'], 'healthy')


class DiagnosticsAreZeroMotionAndCachedTest(unittest.TestCase):
    def test_read_diagnostics_never_sends_a_step_command(self):
        _, mcu, pv2 = _build()
        pv2.probe.read_diagnostics()
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_get_status_before_any_reading_reports_no_reading_taken_yet_not_a_fabricated_value(self):
        _, _, pv2 = _build()
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
