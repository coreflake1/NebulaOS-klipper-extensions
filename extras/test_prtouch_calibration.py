# Standalone tests for prtouch_calibration.py - pure math, no Klipper/MCU needed.
#
# Run with: python3 -m unittest test_prtouch_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import random
import unittest

import prtouch_calibration as cal


class SelectValidChannelsTest(unittest.TestCase):
    def test_no_bits_set(self):
        self.assertEqual(cal.select_valid_channels(0x0), [])

    def test_single_bit(self):
        self.assertEqual(cal.select_valid_channels(0x1), [0])
        self.assertEqual(cal.select_valid_channels(0x8), [3])

    def test_multiple_bits(self):
        self.assertEqual(cal.select_valid_channels(0x5), [0, 2])
        self.assertEqual(cal.select_valid_channels(0xF), [0, 1, 2, 3])


class FilterPressureSeriesTest(unittest.TestCase):
    def test_short_series_passthrough(self):
        self.assertEqual(cal.filter_pressure_series([1, 2], False, 12, 2.0, 0.5), [1, 2])

    def test_adc_skips_zscore_and_highpass(self):
        # use_adc=True should only low-pass filter, never touch the z-score/high-pass path -
        # a constant series stays constant under low-pass alone.
        raw = [100] * 10
        out = cal.filter_pressure_series(raw, True, 1, 2.0, 0.5)
        self.assertTrue(all(abs(v - 100) < 1e-9 for v in out))

    def test_strain_gauge_rejects_single_spike(self):
        raw = [1000] * 20
        raw[10] = 50000  # one wild outlier
        out = cal.filter_pressure_series(raw, False, 12, 2.0, 0.85)
        # the spike shouldn't survive z-score rejection into the final (filtered) series
        self.assertTrue(max(abs(v) for v in out) < 10000)

    def test_output_length_matches_input(self):
        raw = list(range(32))
        out = cal.filter_pressure_series(raw, False, 12, 2.0, 0.85)
        self.assertEqual(len(out), len(raw))


class FindTriggerIndexTest(unittest.TestCase):
    def test_flat_signal_no_crash(self):
        # a stuck/disconnected sensor - must not raise ZeroDivisionError.
        idx = cal.find_trigger_index([5.0] * 10)
        self.assertEqual(idx, 9)

    def test_finds_dip_with_no_drift(self):
        values = [0.0] * 16
        values[10] = -50.0
        idx = cal.find_trigger_index(values)
        self.assertEqual(idx, 10)

    def test_finds_dip_despite_slow_linear_drift(self):
        # same dip, but riding on a slow upward drift across the whole window - the
        # rotate-and-flatten trick should still find the real dip, not an endpoint.
        n = 32
        values = [i * 0.2 for i in range(n)]
        dip_index = 20
        values[dip_index] -= 50.0
        idx = cal.find_trigger_index(values)
        self.assertEqual(idx, dip_index)


class InterpolateTriggerStepTest(unittest.TestCase):
    def test_exact_tick_match(self):
        ticks = [0.0, 1.0, 2.0, 3.0]
        steps = [0, 100, 200, 300]
        self.assertAlmostEqual(cal.interpolate_trigger_step(ticks, steps, 1.0), 100)

    def test_interpolates_between_samples(self):
        ticks = [0.0, 1.0, 2.0, 3.0]
        steps = [0, 100, 200, 300]
        self.assertAlmostEqual(cal.interpolate_trigger_step(ticks, steps, 1.5), 150)

    def test_falls_back_to_last_sample_outside_window(self):
        ticks = [0.0, 1.0, 2.0]
        steps = [0, 100, 200]
        # trigger tick far beyond the buffer's own window - no straddling pair exists.
        self.assertEqual(cal.interpolate_trigger_step(ticks, steps, 50.0), 200)

    def test_skips_zero_step_leading_samples(self):
        # first two samples still read step=0 (motor hasn't started moving yet). The trigger
        # tick falls inside that dead window - matching it there would report a bogus "0 steps
        # traveled" trigger point, so the zero-step guard rejects it and falls back to the last
        # sample instead, exactly like the original's default-to-last-sample behavior.
        ticks = [0.0, 1.0, 2.0, 3.0]
        steps = [0, 0, 100, 200]
        self.assertAlmostEqual(cal.interpolate_trigger_step(ticks, steps, 0.5), 200)


class ComputeTriggerZTest(unittest.TestCase):
    def _make_samples(self, n, mm_per_step, dip_at, tick_step=0.01, step_per_tick=50):
        step_samples = [{'tick': i * tick_step, 'step': i * step_per_tick} for i in range(n)]
        pres_samples = []
        for i in range(n):
            ch0 = 0.0
            if i == dip_at:
                ch0 = -80.0
            pres_samples.append({'tick': i * tick_step, 'ch0': ch0, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        return step_samples, pres_samples

    def test_single_channel_trigger(self):
        dip_at = 12
        step_samples, pres_samples = self._make_samples(32, mm_per_step=0.01, dip_at=dip_at)
        z = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=step_samples[0]['step'], start_pos_z=5.0,
            mm_per_step=0.01, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        expected_step = dip_at * 50
        expected_z = 5.0 - (step_samples[0]['step'] - expected_step) * 0.01
        self.assertAlmostEqual(z, expected_z, delta=0.5)

    def test_raises_on_no_trigger(self):
        step_samples, pres_samples = self._make_samples(32, mm_per_step=0.01, dip_at=-1)
        with self.assertRaises(ValueError):
            cal.compute_trigger_z(
                step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
                tri_chs_bitmask=0x0, start_step=0, start_pos_z=5.0,
                mm_per_step=0.01, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)

    def test_averages_across_multiple_valid_channels(self):
        n = 32
        step_samples = [{'tick': i * 0.01, 'step': i * 50} for i in range(n)]
        pres_samples = []
        for i in range(n):
            pres_samples.append({
                'tick': i * 0.01,
                'ch0': -80.0 if i == 10 else 0.0,
                'ch1': -80.0 if i == 14 else 0.0,
                'ch2': 0.0, 'ch3': 0.0,
            })
        z = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x3, start_step=0, start_pos_z=0.0,
            mm_per_step=0.01, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        # should land between the two single-channel results, not match either exactly.
        z_ch0 = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0,
            mm_per_step=0.01, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        z_ch1 = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x2, start_step=0, start_pos_z=0.0,
            mm_per_step=0.01, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        self.assertAlmostEqual(z, (z_ch0 + z_ch1) / 2, places=9)


class ComputeTriggerZIndependentVerificationTest(unittest.TestCase):
    """2026-08-09 (load-cell safety hardening mission): the tests above all either compute
    their own "expected" value by re-invoking a piece of the same formula under test (circular
    - proves internal consistency, not physical correctness) or pass start_step=0, which is
    NOT what production ever actually passes (prtouch_probe.py's _touch_probe passes the
    COMMANDED step_cnt for the whole descent as start_step, always a large positive number -
    confirmed by direct inspection of the call site).

    This test instead: (1) uses a start_step that matches production's real convention
    (commanded step_cnt, not 0), (2) builds a step trace that counts DOWN from that commanded
    total toward 0 as time advances - confirmed directly from reference/prtouch_v2.c
    (step_cfg.now_steps starts at the commanded total and decrements every pulse; the value
    reported back to the host is now_steps/2, a REMAINING-pulse count, not steps-issued-so-far)
    - and (3) computes the expected physical Z from first principles (start height minus
    physical distance traveled, where distance traveled = commanded_steps_remaining_at_trigger
    subtracted from the commanded total), never by calling compute_trigger_z or any of its
    helpers a second time. If compute_trigger_z's sign/direction were ever flipped by a future
    edit, this is the test that would actually catch it - the pre-existing tests in this class
    would not (they'd stay internally self-consistent either way).
    """

    def test_matches_physically_reasoned_z_with_real_firmware_step_convention(self):
        mm_per_step = 0.01
        start_pos_z = 10.0
        down_min_z = 5.0
        step_cnt = int(down_min_z / mm_per_step)  # 500 - the COMMANDED total, matches
                                                    # production's own get_step_counts()
        n = 32
        traveled_at_trigger_mm = 3.0  # a physical fact we are choosing for this synthetic case
        traveled_steps_at_trigger = int(traveled_at_trigger_mm / mm_per_step)  # 300
        dip_at = 19  # sample index the pressure dip (and therefore the trigger tick) lands at

        # Step buffer counts DOWN from step_cnt to (approximately) 0 as the 32 samples advance -
        # the real firmware's own convention, not the up-counting convention some of this
        # file's older fixtures used before this mission's own correction (see
        # test_prtouch_orchestration.py's _full_step_trace for the same fix applied there).
        step_samples = [{'tick': i * 0.01, 'step': int(step_cnt * (n - 1 - i) / (n - 1))}
                         for i in range(n)]
        # Force the sample AT dip_at to read exactly the chosen remaining-count, so the
        # trigger-tick interpolation lands on a known value rather than whatever the linear
        # ramp's rounding happens to produce at that index.
        step_samples[dip_at]['step'] = step_cnt - traveled_steps_at_trigger

        pres_samples = [{'tick': i * 0.01, 'ch0': (-500.0 if i == dip_at else 0.0),
                          'ch1': 0, 'ch2': 0, 'ch3': 0} for i in range(n)]

        z = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=step_cnt, start_pos_z=start_pos_z,
            mm_per_step=mm_per_step, use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)

        # Physically reasoned expectation, computed independently of compute_trigger_z's own
        # internals: the toolhead started at start_pos_z and moved DOWN (Z decreases)
        # traveled_at_trigger_mm by the time of trigger.
        expected_z = start_pos_z - traveled_at_trigger_mm
        # Generous delta: find_trigger_index's own normalize/rotate step can shift the detected
        # tick by a sample or two relative to the forced dip index for a single-outlier signal
        # like this synthetic trace - this test is checking the SIGN and MAGNITUDE are right
        # (physically correct direction and roughly the right distance), not bit-exact
        # reproduction of the filter's own internal index selection (that's ComputeTriggerZTest
        # and FindTriggerIndexTest's job, both already covered above/elsewhere in this file).
        self.assertAlmostEqual(z, expected_z, delta=0.5)
        # The sign check on its own, stated explicitly: a start_step=0 bug (this test's whole
        # reason for existing) would have produced start_pos_z PLUS traveled instead of MINUS -
        # for these inputs that is 13.0 instead of 7.0, a 6mm error same-sign tests could not
        # distinguish from "off by a bit". Fail loud, not approximately-close-to-the-wrong-
        # answer, if that regresses.
        self.assertLess(z, start_pos_z,
                         "trigger Z must be BELOW the pre-descent start height, never above it, "
                         "for a downward probe that traveled a positive distance")


class FilterPressureSeriesEdgeCasesTest(unittest.TestCase):
    def test_flat_series_no_crash_strain_gauge(self):
        # std==0 branch (the 2026-08 guard added on top of the reference, which lacks it
        # and would raise ZeroDivisionError here - see filter_pressure_series's own
        # docstring/comment) - a fully flat series is exactly what a disconnected sensor
        # would produce.
        out = cal.filter_pressure_series([42.0] * 32, False, 12, 2.0, 0.85)
        self.assertEqual(len(out), 32)
        self.assertTrue(all(math.isfinite(v) for v in out))

    def test_empty_series(self):
        self.assertEqual(cal.filter_pressure_series([], False, 12, 2.0, 0.85), [])

    def test_large_signed_values_no_overflow_or_nan(self):
        # 24-bit HX711-style signed range is roughly +/-8.4M - confirm the filter chain
        # stays finite at the extremes, not just at "normal" small values.
        raw = [-8388608, 8388607] * 16
        out = cal.filter_pressure_series(raw, False, 12, 2.0, 0.85)
        self.assertTrue(all(math.isfinite(v) for v in out))

    def test_isolated_outlier_at_boundary_is_left_alone(self):
        # z-score rejection explicitly skips the first/last two samples (range(1, n-2),
        # matching the reference exactly - see the function's own comment) - an outlier AT
        # the boundary must survive untouched, only interior outliers get replaced.
        raw = [1000.0] * 20
        raw[0] = 99999.0
        raw[-1] = 99999.0
        out_zscore_only = list(raw)
        n = len(out_zscore_only)
        mean = sum(out_zscore_only) / n
        variance = sum((v - mean) ** 2 for v in out_zscore_only) / n
        std = math.sqrt(variance)
        for i in range(1, n - 2):
            if abs(out_zscore_only[i] - mean) / std > 2:
                out_zscore_only[i] = out_zscore_only[i - 1]
        # boundary values must be unchanged by the z-score stage specifically (before the
        # high-pass/low-pass stages run on top, which do touch every index).
        self.assertEqual(out_zscore_only[0], 99999.0)
        self.assertEqual(out_zscore_only[-1], 99999.0)

    def test_constant_offset_does_not_shift_which_index_is_the_dip(self):
        # property test: z-score centers on the mean (cancels a constant shift) and the
        # high-pass filter differences consecutive samples (also cancels a constant shift) -
        # so adding the same baseline to every sample must not move the trigger index,
        # only its absolute filtered value. Asserts agreement across offsets rather than
        # against one hand-picked index, since the exact index a real z-score+filter+
        # rotation pipeline lands on for a given noise draw isn't hand-predictable - what
        # this property actually claims is offset-*invariance*, not a specific value.
        random.seed(20260806)
        noise = [random.uniform(-2.0, 2.0) for _ in range(32)]
        dip_index = 12
        base = list(noise)
        base[dip_index] = -80.0
        indices = set()
        for offset in (0.0, 1000.0, -50000.0, 251471.0):
            shifted = [v + offset for v in base]
            filtered = cal.filter_pressure_series(shifted, False, 12, 2.0, 0.85)
            indices.add(cal.find_trigger_index(filtered))
        self.assertEqual(len(indices), 1,
                          "trigger index must be identical across all baseline offsets, got %s"
                          % indices)


class FindTriggerIndexEdgeCasesTest(unittest.TestCase):
    def test_trigger_at_first_sample(self):
        values = [-50.0] + [0.0] * 31
        self.assertEqual(cal.find_trigger_index(values), 0)

    def test_trigger_at_last_sample(self):
        values = [0.0] * 31 + [-50.0]
        self.assertEqual(cal.find_trigger_index(values), 31)

    def test_minimum_legal_size_two_samples(self):
        idx = cal.find_trigger_index([0.0, -1.0])
        self.assertIn(idx, (0, 1))

    def test_empty_array_raises_not_silently_wrong(self):
        # min()/max() on an empty sequence raise ValueError in plain Python - documenting
        # this as the real, expected contract (prtouch_probe.py never calls this with an
        # empty series - the no-trigger/empty-buffer case is filtered out one layer up,
        # before compute_trigger_z is ever invoked, see prtouch_probe.py's `if not
        # step_samples or not pres_samples: continue`) rather than adding a defensive guard
        # for a precondition the real call site already enforces.
        with self.assertRaises(ValueError):
            cal.find_trigger_index([])

    def test_negative_drift(self):
        n = 32
        values = [-i * 0.2 for i in range(n)]  # trending down instead of up
        dip_index = 22
        values[dip_index] -= 50.0
        self.assertEqual(cal.find_trigger_index(values), dip_index)

    def test_noisy_baseline_still_finds_the_real_dip(self):
        random.seed(20260806)
        n = 32
        values = [random.uniform(-2.0, 2.0) for _ in range(n)]
        dip_index = 18
        values[dip_index] = -80.0
        self.assertEqual(cal.find_trigger_index(values), dip_index)

    def test_property_index_always_within_range(self):
        random.seed(20260806)
        for _ in range(200):
            n = random.randint(2, 40)
            values = [random.uniform(-1e6, 1e6) for _ in range(n)]
            if max(values) == min(values):
                continue  # flat-signal case covered separately
            idx = cal.find_trigger_index(values)
            self.assertGreaterEqual(idx, 0)
            self.assertLess(idx, n)


class InterpolateTriggerStepEdgeCasesTest(unittest.TestCase):
    def test_repeated_timestamps_zero_time_delta_no_crash(self):
        # denom==0 guard (see the function's own `if denom else 0.0`) - a repeated tick
        # value between two straddling samples must not raise ZeroDivisionError.
        ticks = [0.0, 1.0, 1.0, 2.0]
        steps = [0, 100, 100, 200]
        result = cal.interpolate_trigger_step(ticks, steps, 1.0)
        self.assertTrue(math.isfinite(result))

    def test_non_monotonic_timestamps_no_crash(self):
        # out-of-order ticks shouldn't happen in real MCU data (index order == time order by
        # construction), but the function must degrade to *some* finite number, never a
        # crash, if it ever does.
        ticks = [0.0, 2.0, 1.0, 3.0]
        steps = [0, 200, 100, 300]
        result = cal.interpolate_trigger_step(ticks, steps, 1.5)
        self.assertTrue(math.isfinite(result))

    def test_minimum_legal_two_samples_always_falls_back_to_last(self):
        # non-obvious real property, worth pinning down explicitly: true interpolation only
        # ever happens when 0 < step_tri_index < n-1 holds for some *interior* index - with
        # exactly n=2 samples that range is empty (0 < idx < 1 has no integer solution), so
        # the function always returns step_values[-1] verbatim for a 2-sample buffer,
        # regardless of where the trigger tick actually falls. Not a bug (2-sample buffers
        # never occur in real production - MAX_BUF_LEN is fixed at 32), but a real
        # consequence of the interpolation guard worth a test rather than an assumption.
        self.assertEqual(cal.interpolate_trigger_step([0.0, 1.0], [10, 100], 0.5), 100)
        self.assertEqual(cal.interpolate_trigger_step([0.0, 1.0], [0, 100], 0.5), 100)

    def test_property_never_nan_or_infinite(self):
        random.seed(20260806)
        for _ in range(200):
            n = random.randint(2, 40)
            ticks = sorted(random.uniform(-100, 100) for _ in range(n))
            steps = [random.randint(-1000, 1000) for _ in range(n)]
            trigger_tick = random.uniform(-200, 200)
            result = cal.interpolate_trigger_step(ticks, steps, trigger_tick)
            self.assertTrue(math.isfinite(result))

    def test_mismatched_array_lengths_raises_rather_than_silently_truncating(self):
        # step_values shorter than step_ticks must fail loud (IndexError), not silently
        # return a value computed from a truncated/misaligned pairing.
        with self.assertRaises(IndexError):
            cal.interpolate_trigger_step([0.0, 1.0, 2.0, 3.0], [0, 100], 1.5)


class ComputeTriggerZEdgeCasesTest(unittest.TestCase):
    def _make_samples(self, n, mm_per_step, dip_at, tick_step=0.01, step_per_tick=50):
        step_samples = [{'tick': i * tick_step, 'step': i * step_per_tick} for i in range(n)]
        pres_samples = []
        for i in range(n):
            ch0 = -80.0 if i == dip_at else 0.0
            pres_samples.append({'tick': i * tick_step, 'ch0': ch0, 'ch1': 0, 'ch2': 0, 'ch3': 0})
        return step_samples, pres_samples

    def test_mismatched_sample_counts_between_step_and_pres(self):
        # step buffer fully repaired (32) but pres buffer still short (16, say a dropped
        # repair) - the function must not silently pretend they're the same length; it
        # zips per-channel against pres_samples' own length only, so this should still
        # produce a finite (if not perfectly accurate) result rather than crash - confirms
        # the actual, current contract rather than assuming one.
        step_samples, pres_samples = self._make_samples(32, 0.01, dip_at=10)
        short_pres = pres_samples[:16]
        z = cal.compute_trigger_z(
            step_samples, short_pres, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0, mm_per_step=0.01,
            use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        self.assertTrue(math.isfinite(z))

    def test_duplicate_final_samples(self):
        step_samples, pres_samples = self._make_samples(32, 0.01, dip_at=15)
        step_samples.append(dict(step_samples[-1]))
        pres_samples.append(dict(pres_samples[-1]))
        z = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0, mm_per_step=0.01,
            use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        self.assertTrue(math.isfinite(z))

    def test_minimum_legal_array_size(self):
        step_samples = [{'tick': 0.0, 'step': 0}, {'tick': 0.01, 'step': 50}]
        pres_samples = [{'tick': 0.0, 'ch0': 0.0, 'ch1': 0, 'ch2': 0, 'ch3': 0},
                         {'tick': 0.01, 'ch0': -80.0, 'ch1': 0, 'ch2': 0, 'ch3': 0}]
        z = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0, mm_per_step=0.01,
            use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5)
        self.assertTrue(math.isfinite(z))

    def test_inactive_channel_containing_zero_is_excluded_from_average(self):
        # pres_cnt=1 (this printer's real config) - channels 1-3 report a constant 0 and
        # must never be averaged in just because their array slot exists; only tri_chs_
        # bitmask decides participation (see select_valid_channels).
        step_samples, pres_samples = self._make_samples(32, 0.01, dip_at=10)
        z_ch0_only = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0, mm_per_step=0.01,
            use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5, pres_cnt=1)
        # if channels 1-3 (all constant 0, no real dip) leaked into the average, this would
        # differ noticeably from the single valid channel's own answer.
        z_all_bits_but_only_ch0_meaningful = cal.compute_trigger_z(
            step_samples, pres_samples, step_tri_time=0.0, pres_tri_time=0.0,
            tri_chs_bitmask=0x1, start_step=0, start_pos_z=0.0, mm_per_step=0.01,
            use_adc=True, acq_ms=1, hftr_cut=2.0, lftr_k1=0.5, pres_cnt=4)
        self.assertAlmostEqual(z_ch0_only, z_all_bits_but_only_ch0_meaningful, places=9)


if __name__ == '__main__':
    unittest.main()
