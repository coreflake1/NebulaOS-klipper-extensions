# Standalone tests for prtouch_calibration.py - pure math, no Klipper/MCU needed.
#
# Run with: python3 -m unittest test_prtouch_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
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


if __name__ == '__main__':
    unittest.main()
