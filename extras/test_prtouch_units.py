# Unit-conversion tests - proves each named helper in prtouch_units.py matches the exact
# inline arithmetic it replaced (both the production call sites and the reference source).
#
# Run with: python3 -m unittest test_prtouch_units -v (bare import, no relative-import
# dependency - this module has none).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import random
import unittest

import prtouch_units as units


class McuTickConversionTest(unittest.TestCase):
    def test_matches_original_inline_divisor(self):
        self.assertAlmostEqual(units.mcu_ticks_to_seconds(12345), 1.2345)

    def test_zero(self):
        self.assertEqual(units.mcu_ticks_to_seconds(0), 0.0)

    def test_round_trip(self):
        for ticks in (0, 1, 10000, 999999, -500):
            seconds = units.mcu_ticks_to_seconds(ticks)
            self.assertAlmostEqual(units.seconds_to_mcu_ticks(seconds), ticks)


class FixedPointTest(unittest.TestCase):
    def test_matches_reference_scale_factor(self):
        # reference/prtouch_v2_wrapper.py: int(use_tri_hftr_cut * 1000)
        self.assertEqual(units.to_fixed_point(2.0), 2000)
        self.assertEqual(units.to_fixed_point(0.7), 700)

    def test_truncates_not_rounds(self):
        # int() truncation, matching the reference's own int(x*1000) exactly (not round()).
        self.assertEqual(units.to_fixed_point(0.6669), 666)


class DutyFractionTest(unittest.TestCase):
    def test_default_sys_time_duty_matches_original_inline_value(self):
        # config default 0.001 -> the exact value previously hardcoded inline
        # (int(self.sys_time_duty * 100000)) at both add_config_cmd call sites.
        self.assertEqual(units.duty_fraction_to_scaled_units(0.001), 100)


class DistanceStepConversionTest(unittest.TestCase):
    def test_matches_reference_truncation(self):
        # reference get_step_cnts: int(run_dis / self.mm_per_step) - truncates toward zero.
        self.assertEqual(units.distance_mm_to_step_count(2.0, 0.01), 200)
        self.assertEqual(units.distance_mm_to_step_count(1.999, 0.01), 199)  # truncated, not rounded

    def test_step_us_matches_reference_formula(self):
        # reference: step_us = int(((run_dis / run_spd) * 1000 * 1000) / step_cnt)
        distance, speed, step_cnt = 2.0, 1.0, 200
        expected = int(((distance / speed) * 1000 * 1000) / step_cnt)
        self.assertEqual(units.step_count_to_step_us(distance, speed, step_cnt), expected)

    def test_acc_ctl_cnt_matches_reference_formula(self):
        # reference: acc_ctl_cnt = int(self.acc_ctl_mm / self.mm_per_step)
        self.assertEqual(units.distance_mm_to_acc_ctl_cnt(0.25, 0.01), 25)

    def test_steps_to_distance_round_trip_within_one_step(self):
        mm_per_step = 0.01
        for distance in (0.0, 1.0, 2.0, 199.99, 500.0):
            step_cnt = units.distance_mm_to_step_count(distance, mm_per_step)
            back = units.step_count_to_distance_mm(step_cnt, mm_per_step)
            self.assertLessEqual(abs(back - distance), mm_per_step)

    def test_sign_convention_negative_steps_negative_distance(self):
        # steps_to_mm must preserve sign - _lift_after_down relies on this for its
        # `if traveled <= 0: return` early-exit guard on an over-traveled probe.
        self.assertLess(units.step_count_to_distance_mm(-5, 0.01), 0)
        self.assertGreater(units.step_count_to_distance_mm(5, 0.01), 0)
        self.assertEqual(units.step_count_to_distance_mm(0, 0.01), 0)

    def test_property_step_count_always_matches_int_division(self):
        random.seed(20260806)
        for _ in range(200):
            distance = random.uniform(-100, 100)
            mm_per_step = random.uniform(0.001, 0.1)
            self.assertEqual(units.distance_mm_to_step_count(distance, mm_per_step),
                              int(distance / mm_per_step))


class ProbeTimeoutTest(unittest.TestCase):
    def test_matches_prtouch_probe_default_2s_margin(self):
        # prtouch_probe.py's own down_min_z / self.tri_z_down_spd + 2.0
        self.assertAlmostEqual(units.probe_timeout_seconds(2.0, 1.0), 4.0)

    def test_matches_safe_move_z_5s_margin(self):
        # prtouch_probe.py's safe_move_z: distance / speed + 5.0
        self.assertAlmostEqual(units.probe_timeout_seconds(10.0, 5.0, margin_s=5.0), 7.0)

    def test_matches_reference_formula_exactly(self):
        # reference: down_min_z / use_tri_z_down_spd + 2 (run_step_prtouch's own poll bound)
        down_min_z, speed = 25.0, 1.0
        expected = down_min_z / speed + 2
        self.assertAlmostEqual(units.probe_timeout_seconds(down_min_z, speed), expected)

    def test_property_always_finite_and_positive_for_positive_inputs(self):
        random.seed(20260806)
        for _ in range(200):
            distance = random.uniform(0.01, 500)
            speed = random.uniform(0.01, 50)
            t = units.probe_timeout_seconds(distance, speed)
            self.assertTrue(math.isfinite(t))
            self.assertGreater(t, 0)


if __name__ == '__main__':
    unittest.main()
