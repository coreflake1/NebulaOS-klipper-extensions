# Tests for NEBULAOS_AXIS_TWIST_CALIBRATE (extras/nebulaos_calibration.py,
# Phase 2 calibration-framework mission).
#
# Two layers, matching the confidence levels the project's own rules ask
# for:
#   1. Fast orchestration tests against FakeAxisTwistCompensation/
#      FakeCalibrater - a hand-written stand-in that mirrors the real
#      upstream _finalize_calibration() algorithm exactly (mean-center),
#      used to test THIS module's own preflight/sequencing/state/error
#      logic without needing the real pinned Klipper source on disk.
#   2. RealUpstreamParityTest - imports the REAL pinned
#      axis_twist_compensation.py directly (klippy/extras/, 58bd67db...)
#      and drives this module's own coordinator against the REAL
#      AxisTwistCompensation/Calibrater objects, proving the fake in layer
#      1 is not merely self-consistent but actually matches upstream.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_axis_twist -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import os
import sys
import types
import unittest

from . import prtouch_test_support as fake

if 'extras.probe' not in sys.modules:
    _placeholder = types.ModuleType('extras.probe')
    _placeholder.run_single_probe = lambda probe_obj, gcmd: None
    sys.modules['extras.probe'] = _placeholder

from . import nebulaos_calibration
from . import nebulaos_probe_pair


# ---------------------------------------------------------------------
# Layer 1 fakes
# ---------------------------------------------------------------------

class FakeCalibrater:
    """Mirrors the REAL upstream Calibrater._finalize_calibration()
    algorithm (58bd67db..., axis_twist_compensation.py) exactly - mean-
    center the raw results, stage per-axis config fields, activate the
    live compensation array. RealUpstreamParityTest below proves this
    mirroring is accurate, not just self-consistent."""

    def __init__(self, compensation):
        self.compensation = compensation
        self.results = None
        self.current_axis = None
        self.gcmd = None
        self.configname = 'axis_twist_compensation'
        self.finalize_calls = 0

    def _finalize_calibration(self):
        self.finalize_calls += 1
        avg = sum(self.results) / len(self.results)
        self.results = [avg - x for x in self.results]
        if self.current_axis == 'X':
            self.compensation.z_compensations = self.results
        elif self.current_axis == 'Y':
            self.compensation.zy_compensations = self.results
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, "
            "offsets: %s, mean z_offset: %f" % (self.results, avg))


class FakeAxisTwistCompensation:
    def __init__(self, calibrate_start_x=20., calibrate_end_x=200., calibrate_y=117.5,
                 calibrate_start_y=40., calibrate_end_y=200., calibrate_x=117.5):
        self.calibrate_start_x = calibrate_start_x
        self.calibrate_end_x = calibrate_end_x
        self.calibrate_y = calibrate_y
        self.calibrate_start_y = calibrate_start_y
        self.calibrate_end_y = calibrate_end_y
        self.calibrate_x = calibrate_x
        self.z_compensations = []
        self.zy_compensations = []
        self.clear_calls = []
        self.calibrater = FakeCalibrater(self)

    def clear_compensations(self, axis=None):
        # Mirrors real upstream exactly: per-axis-only clearing.
        self.clear_calls.append(axis)
        if axis is None:
            self.z_compensations = []
            self.zy_compensations = []
        elif axis == 'X':
            self.z_compensations = []
        elif axis == 'Y':
            self.zy_compensations = []


class FakeProbeOffsets:
    def __init__(self, x_offset=0., y_offset=27., z_offset=0.):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset


class FakeCmdHelper:
    def __init__(self, name='bltouch'):
        self.name = name


class FakeProbeObj:
    def __init__(self, x_offset=0., y_offset=27., z_offset=0.):
        self.probe_offsets = FakeProbeOffsets(x_offset, y_offset, z_offset)
        self.cmd_helper = FakeCmdHelper()

    def get_offsets(self, gcmd=None):
        o = self.probe_offsets
        return o.x_offset, o.y_offset, o.z_offset


class FakeZOffsetProbe:
    def __init__(self, is_calibrated=True):
        self._is_calibrated = is_calibrated

    def get_status(self, eventtime):
        return {'is_calibrated': self._is_calibrated}


class FakeConfigFile:
    def __init__(self):
        self.set_calls = []

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))


def _build(axis_twist=None, z_offset_probe=None, probe_obj=None,
           config_overrides=None):
    printer = fake.FakePrinter()
    gcode = fake.FakeGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('probe', probe_obj if probe_obj is not None else FakeProbeObj())
    printer.add_object('configfile', FakeConfigFile())
    if axis_twist is not None:
        printer.add_object('axis_twist_compensation', axis_twist)
    if z_offset_probe is not None:
        printer.add_object('nebulaos_z_offset_probe', z_offset_probe)

    values = dict(config_overrides or {})
    config = fake.FakeConfig(values, section='nebulaos_calibration', printer=printer)
    coordinator = nebulaos_calibration.NebulaOSCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, coordinator


def _constant_measurements(value, count):
    """Every sample reports the identical raw (probe_trigger, contact)
    pair - the canonical "flat bed" case: normalized compensations must
    all be exactly zero."""
    seq = [value] * count
    return _measurement_sequence(seq)


def _measurement_sequence(probe_minus_contact_values):
    """Builds a stub for nebulaos_probe_pair.measure_probe_nozzle_pair that
    returns, in order, one PairedMeasurement per call whose
    probe_z_offset equals the given value (raw_probe_trigger_z fixed at
    that value, raw_nozzle_contact_z fixed at 0 - the specific split
    between the two doesn't matter, only the difference does, per this
    module's own parity note)."""
    calls = []

    def stub(printer, x, y, probe_x_offset, probe_y_offset, horizontal_move_z,
              z_offset_probe, down_min_z, pro_cnt=1, travel_speed=None,
              probe_lift_speed=None):
        i = len(calls)
        calls.append((x, y))
        value = probe_minus_contact_values[i]
        return nebulaos_probe_pair.PairedMeasurement(
            x=x, y=y, raw_probe_trigger_z=value, raw_nozzle_contact_z=0.0,
            probe_z_offset=value)
    stub.calls = calls
    return stub


class _StubbedPairMixin:
    def _run_with_stub(self, coordinator, gcmd, stub):
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = stub
        try:
            coordinator.cmd_axis_twist_calibrate(gcmd)
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig


# ---------------------------------------------------------------------
# Geometry / point generation
# ---------------------------------------------------------------------

class BedPointGenerationTest(unittest.TestCase):
    def test_x_axis_three_points_matches_hand_computed_upstream_formula(self):
        # Upstream: x_axis_range = end - start; interval = range/(n-1);
        # x_i = start + i*interval; y constant = calibrate_y.
        # start=20, end=200 -> range=180, n=3 -> interval=90.
        comp = FakeAxisTwistCompensation(calibrate_start_x=20., calibrate_end_x=200.,
                                          calibrate_y=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        self.assertEqual(points, [(20.0, 117.5), (110.0, 117.5), (200.0, 117.5)])

    def test_y_axis_three_points_matches_hand_computed_upstream_formula(self):
        # start=40, end=200 -> range=160, n=3 -> interval=80.
        comp = FakeAxisTwistCompensation(calibrate_start_y=40., calibrate_end_y=200.,
                                          calibrate_x=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'Y', 3)
        self.assertEqual(points, [(117.5, 40.0), (117.5, 120.0), (117.5, 200.0)])

    def test_x_axis_five_points(self):
        comp = FakeAxisTwistCompensation(calibrate_start_x=0., calibrate_end_x=100.,
                                          calibrate_y=50.)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 5)
        xs = [p[0] for p in points]
        self.assertEqual(xs, [0.0, 25.0, 50.0, 75.0, 100.0])
        self.assertTrue(all(p[1] == 50. for p in points))

    def test_two_point_minimum(self):
        comp = FakeAxisTwistCompensation(calibrate_start_x=10., calibrate_end_x=20.,
                                          calibrate_y=5.)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 2)
        self.assertEqual(points, [(10.0, 5.0), (20.0, 5.0)])

    def test_missing_x_config_raises_value_error(self):
        comp = FakeAxisTwistCompensation()
        comp.calibrate_start_x = None
        with self.assertRaises(ValueError) as ctx:
            nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        self.assertIn('calibrate_start_x', str(ctx.exception))

    def test_missing_y_config_raises_value_error(self):
        comp = FakeAxisTwistCompensation()
        comp.calibrate_end_y = None
        with self.assertRaises(ValueError):
            nebulaos_calibration.axis_twist_bed_points(comp, 'Y', 3)

    def test_probe_offset_applied_by_the_shared_primitive_not_here(self):
        # axis_twist_bed_points() returns NOZZLE-target bed points -
        # exactly like upstream's own bed_points list. The probe-side
        # offset subtraction is nebulaos_probe_pair's job (already proven
        # in test_nebulaos_probe_pair.py) - this test only guards against
        # a future accidental double-application.
        comp = FakeAxisTwistCompensation(calibrate_start_x=20., calibrate_end_x=200.,
                                          calibrate_y=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        self.assertEqual(points[0], (20.0, 117.5))  # NOT offset by y_offset=27


# ---------------------------------------------------------------------
# Math / normalization parity (sign-regression focus)
# ---------------------------------------------------------------------

class NormalizationMathTest(unittest.TestCase, _StubbedPairMixin):
    def test_constant_measurements_normalize_to_all_zeros(self):
        axis_twist = FakeAxisTwistCompensation()
        z_probe = FakeZOffsetProbe(True)
        printer, gcode, coord = _build(axis_twist, z_probe)
        stub = _constant_measurements(1.234, 3)
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'LOAD_CELL'}), stub)
        self.assertEqual(axis_twist.z_compensations, [0.0, 0.0, 0.0])

    def test_positive_gradient_produces_expected_symmetric_pattern(self):
        # Raw values 0, 1, 2 -> avg=1 -> compensations = [1, 0, -1].
        axis_twist = FakeAxisTwistCompensation()
        z_probe = FakeZOffsetProbe(True)
        printer, gcode, coord = _build(axis_twist, z_probe)
        stub = _measurement_sequence([0.0, 1.0, 2.0])
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'LOAD_CELL'}), stub)
        self.assertEqual(axis_twist.z_compensations, [1.0, 0.0, -1.0])

    def test_negative_gradient_produces_sign_flipped_pattern(self):
        # Raw values 2, 1, 0 -> avg=1 -> compensations = [-1, 0, 1] - the
        # exact sign-mirror of the positive-gradient case; proves the
        # formula isn't accidentally using abs() or a fixed sign.
        axis_twist = FakeAxisTwistCompensation()
        z_probe = FakeZOffsetProbe(True)
        printer, gcode, coord = _build(axis_twist, z_probe)
        stub = _measurement_sequence([2.0, 1.0, 0.0])
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'LOAD_CELL'}), stub)
        self.assertEqual(axis_twist.z_compensations, [-1.0, 0.0, 1.0])

    def test_y_axis_uses_zy_compensations_not_z_compensations(self):
        axis_twist = FakeAxisTwistCompensation()
        z_probe = FakeZOffsetProbe(True)
        printer, gcode, coord = _build(axis_twist, z_probe)
        stub = _measurement_sequence([0.0, 1.0, 2.0])
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'Y', 'METHOD': 'LOAD_CELL'}), stub)
        self.assertEqual(axis_twist.zy_compensations, [1.0, 0.0, -1.0])
        self.assertEqual(axis_twist.z_compensations, [])  # X untouched

    def test_asymmetric_five_point_gradient(self):
        axis_twist = FakeAxisTwistCompensation()
        z_probe = FakeZOffsetProbe(True)
        printer, gcode, coord = _build(axis_twist, z_probe)
        raw = [0.0, 0.5, 1.5, 1.0, 2.0]
        avg = sum(raw) / len(raw)
        expected = [avg - r for r in raw]
        stub = _measurement_sequence(raw)
        self._run_with_stub(
            coord, fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'LOAD_CELL', 'SAMPLE_COUNT': '5'}),
            stub)
        for got, want in zip(axis_twist.z_compensations, expected):
            self.assertAlmostEqual(got, want, places=9)


# ---------------------------------------------------------------------
# State application: X/Y independence
# ---------------------------------------------------------------------

class StateApplicationTest(unittest.TestCase, _StubbedPairMixin):
    def test_x_calibration_updates_x_only(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        self.assertEqual(coord.axis_twist_x_state, 'complete')
        self.assertEqual(coord.axis_twist_y_state, 'idle')
        self.assertIsNotNone(coord.axis_twist_x_result)
        self.assertIsNone(coord.axis_twist_y_result)

    def test_y_calibration_updates_y_only(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'Y'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        self.assertEqual(coord.axis_twist_y_state, 'complete')
        self.assertEqual(coord.axis_twist_x_state, 'idle')

    def test_both_leaves_x_and_y_active_concurrently(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(
            coord, fake.FakeGCmd({'AXIS': 'BOTH'}),
            _measurement_sequence([0.0, 1.0, 2.0, 0.0, 1.0, 2.0]))
        self.assertEqual(coord.axis_twist_x_state, 'complete')
        self.assertEqual(coord.axis_twist_y_state, 'complete')
        self.assertEqual(axis_twist.z_compensations, [1.0, 0.0, -1.0])
        self.assertEqual(axis_twist.zy_compensations, [1.0, 0.0, -1.0])

    def test_recalibrate_x_preserves_existing_y(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'Y'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        y_before = list(axis_twist.zy_compensations)
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X'}),
                             _measurement_sequence([5.0, 6.0, 7.0]))
        self.assertEqual(axis_twist.zy_compensations, y_before)
        self.assertEqual(axis_twist.z_compensations, [1.0, 0.0, -1.0])

    def test_recalibrate_y_preserves_existing_x(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        x_before = list(axis_twist.z_compensations)
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'Y'}),
                             _measurement_sequence([5.0, 6.0, 7.0]))
        self.assertEqual(axis_twist.z_compensations, x_before)
        self.assertEqual(axis_twist.zy_compensations, [1.0, 0.0, -1.0])

    def test_clear_compensations_called_with_only_the_target_axis(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        self.assertEqual(axis_twist.clear_calls, ['X'])

    def test_both_clears_x_then_y_separately_not_both_at_once(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(
            coord, fake.FakeGCmd({'AXIS': 'BOTH'}),
            _measurement_sequence([0.0, 1.0, 2.0, 0.0, 1.0, 2.0]))
        self.assertEqual(axis_twist.clear_calls, ['X', 'Y'])


# ---------------------------------------------------------------------
# Persistence staging
# ---------------------------------------------------------------------

class PersistenceStagingTest(unittest.TestCase, _StubbedPairMixin):
    def test_no_save_config_call_anywhere(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'BOTH'}),
                             _measurement_sequence([0.0, 1.0, 2.0, 0.0, 1.0, 2.0]))
        self.assertNotIn('SAVE_CONFIG', gcode.scripts_run)

    def test_no_restart_script_run(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run_with_stub(coord, fake.FakeGCmd({'AXIS': 'X'}),
                             _measurement_sequence([0.0, 1.0, 2.0]))
        for s in gcode.scripts_run:
            self.assertNotIn('RESTART', s.upper())


# ---------------------------------------------------------------------
# Preflight / failure behavior
# ---------------------------------------------------------------------

class PreflightTest(unittest.TestCase):
    def test_axis_required_no_default(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({}))
        self.assertIn('AXIS', str(ctx.exception))

    def test_unknown_axis_rejected(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'Z'}))

    def test_unknown_method_rejected(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'BOGUS'}))

    def test_no_axis_twist_compensation_configured_raises(self):
        printer, gcode, coord = _build(axis_twist=None, z_offset_probe=FakeZOffsetProbe(True))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('axis_twist_compensation', str(ctx.exception))

    def test_no_load_cell_configured_raises(self):
        printer, gcode, coord = _build(axis_twist=FakeAxisTwistCompensation(),
                                        z_offset_probe=None)
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('METHOD=MANUAL', str(ctx.exception))

    def test_uncalibrated_load_cell_raises(self):
        printer, gcode, coord = _build(axis_twist=FakeAxisTwistCompensation(),
                                        z_offset_probe=FakeZOffsetProbe(False))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('LOAD_CELL_CALIBRATE', str(ctx.exception))


class FailureBehaviorTest(unittest.TestCase):
    def _stub_that_fails_at(self, fail_index, count):
        calls = []

        def stub(printer, x, y, probe_x_offset, probe_y_offset, horizontal_move_z,
                  z_offset_probe, down_min_z, pro_cnt=1, travel_speed=None,
                  probe_lift_speed=None):
            i = len(calls)
            calls.append((x, y))
            if i == fail_index:
                raise fake.CommandError("simulated sensor failure at sample %d" % (i + 1))
            return nebulaos_probe_pair.PairedMeasurement(
                x=x, y=y, raw_probe_trigger_z=float(i), raw_nozzle_contact_z=0.0,
                probe_z_offset=float(i))
        return stub

    def _run(self, coordinator, gcmd, stub):
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = stub
        try:
            with self.assertRaises(fake.CommandError):
                coordinator.cmd_axis_twist_calibrate(gcmd)
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

    def test_first_sample_failure_aborts_axis_leaves_no_compensation(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run(coord, fake.FakeGCmd({'AXIS': 'X', 'SAMPLE_COUNT': '3'}),
                   self._stub_that_fails_at(0, 3))
        self.assertEqual(coord.axis_twist_x_state, 'error')
        self.assertEqual(axis_twist.z_compensations, [])  # cleared, never re-populated
        self.assertEqual(axis_twist.calibrater.finalize_calls, 0)

    def test_middle_sample_failure_aborts_axis(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run(coord, fake.FakeGCmd({'AXIS': 'X', 'SAMPLE_COUNT': '5'}),
                   self._stub_that_fails_at(2, 5))
        self.assertEqual(coord.axis_twist_x_state, 'error')
        self.assertEqual(axis_twist.calibrater.finalize_calls, 0)

    def test_final_sample_failure_aborts_axis_no_partial_publish(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        self._run(coord, fake.FakeGCmd({'AXIS': 'X', 'SAMPLE_COUNT': '3'}),
                   self._stub_that_fails_at(2, 3))
        self.assertEqual(coord.axis_twist_x_state, 'error')
        # Even though 2 of 3 samples succeeded, finalize must NEVER be
        # called with a partial set - this is the single most important
        # invariant this slice requires.
        self.assertEqual(axis_twist.calibrater.finalize_calls, 0)
        self.assertEqual(axis_twist.z_compensations, [])

    def test_non_finite_measurement_aborts_axis(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        stub = _measurement_sequence([0.0, float('nan'), 2.0])
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = stub
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X', 'SAMPLE_COUNT': '3'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.axis_twist_x_state, 'error')
        self.assertEqual(axis_twist.calibrater.finalize_calls, 0)

    def test_failed_axis_does_not_disturb_previously_valid_opposite_axis(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        # First, a real successful Y calibration.
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _measurement_sequence([0.0, 1.0, 2.0])
        try:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'Y', 'SAMPLE_COUNT': '3'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        y_after_success = list(axis_twist.zy_compensations)
        self.assertEqual(coord.axis_twist_y_state, 'complete')

        # Now a FAILED X calibration.
        self._run(coord, fake.FakeGCmd({'AXIS': 'X', 'SAMPLE_COUNT': '3'}),
                   self._stub_that_fails_at(1, 3))
        self.assertEqual(coord.axis_twist_x_state, 'error')
        self.assertEqual(coord.axis_twist_y_state, 'complete')
        self.assertEqual(axis_twist.zy_compensations, y_after_success)


# ---------------------------------------------------------------------
# Manual method
# ---------------------------------------------------------------------

class ManualMethodTest(unittest.TestCase):
    def test_x_dispatches_to_pristine_upstream_command(self):
        printer, gcode, coord = _build()
        coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'MANUAL'}))
        self.assertEqual(len(gcode.scripts_run), 1)
        self.assertTrue(gcode.scripts_run[0].startswith(
            'AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=X'))

    def test_y_dispatches_to_pristine_upstream_command(self):
        printer, gcode, coord = _build()
        coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'Y', 'METHOD': 'MANUAL'}))
        self.assertTrue(gcode.scripts_run[0].startswith(
            'AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=Y'))

    def test_both_is_rejected_not_silently_chained(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'BOTH', 'METHOD': 'MANUAL'}))
        self.assertIn('not supported', str(ctx.exception))
        self.assertEqual(gcode.scripts_run, [])

    def test_manual_does_not_touch_load_cell_axis_twist_state(self):
        printer, gcode, coord = _build()
        coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'MANUAL'}))
        self.assertEqual(coord.axis_twist_x_state, 'idle')

    def test_manual_requires_no_axis_twist_compensation_object_lookup(self):
        # MANUAL mode must work even if [axis_twist_compensation] object
        # lookup would fail - it never touches it directly, only via the
        # delegated upstream command string.
        printer, gcode, coord = _build(axis_twist=None)
        coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'MANUAL'}))
        self.assertEqual(len(gcode.scripts_run), 1)


# ---------------------------------------------------------------------
# Status
# ---------------------------------------------------------------------

class StatusTest(unittest.TestCase, _StubbedPairMixin):
    def test_status_distinguishes_x_and_y_for_both(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(axis_twist, FakeZOffsetProbe(True))
        # X gets a linear gradient, Y a differently-SHAPED one (not just a
        # constant shift, which the mean-centering formula would make
        # indistinguishable from X's own pattern - see this module's own
        # parity note on why a constant shift alone doesn't change the
        # normalized result).
        self._run_with_stub(
            coord, fake.FakeGCmd({'AXIS': 'BOTH'}),
            _measurement_sequence([0.0, 1.0, 2.0, 0.0, 2.0, 1.0]))
        status = coord.get_status(0.)
        self.assertEqual(status['axis_twist_x_state'], 'complete')
        self.assertEqual(status['axis_twist_y_state'], 'complete')
        self.assertNotEqual(status['axis_twist_x_result'], status['axis_twist_y_result'])
        self.assertIsNone(status['axis_twist_current_axis'])  # nothing running anymore

    def test_status_command_does_not_raise(self):
        printer, gcode, coord = _build()
        coord.cmd_calibration_status(fake.FakeGCmd())


# ---------------------------------------------------------------------
# Layer 2: real pinned upstream object parity
# ---------------------------------------------------------------------

def _find_klipper_src():
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, '..', '..', '..', '_scratch', 'ref-klipper-mainline')
    candidate = os.path.abspath(candidate)
    if os.path.isdir(os.path.join(candidate, 'klippy')):
        return candidate
    return None


_KLIPPER_SRC = os.environ.get('KLIPPER_SRC') or _find_klipper_src()


class MiniFakeGCode:
    def __init__(self):
        self.commands = {}
        self.responses = []

    def register_command(self, name, handler, desc=None):
        self.commands[name] = handler

    def respond_info(self, msg):
        self.responses.append(msg)


class MiniFakePrinter:
    def __init__(self):
        self.objects = {'gcode': MiniFakeGCode(), 'configfile': FakeConfigFile()}
        self.event_handlers = {}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def add_object(self, name, obj):
        self.objects[name] = obj

    def register_event_handler(self, event, cb):
        self.event_handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        for cb in self.event_handlers.get(event, []):
            cb(*args)

    def command_error(self, msg):
        return fake.CommandError(msg)

    def config_error(self, msg):
        return fake.CommandError(msg)


class MiniFakeConfig:
    def __init__(self, values, name='axis_twist_compensation', printer=None):
        self._values = values
        self._name = name
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def getfloat(self, option, default=None, **kw):
        return self._values.get(option, default)

    def getlists(self, option, default=None, parser=float, **kw):
        v = self._values.get(option)
        return v if v is not None else default


@unittest.skipUnless(_KLIPPER_SRC, "no pinned Klipper checkout found (set KLIPPER_SRC)")
class RealUpstreamParityTest(unittest.TestCase):
    """Imports the REAL pinned axis_twist_compensation.py directly and
    proves: (a) this project's own finalize handoff produces IDENTICAL
    results to calling the real object's own method directly, and (b) the
    fakes used everywhere else in this file mirror the real algorithm
    accurately."""

    @classmethod
    def setUpClass(cls):
        # This repo's own top-level package is ALSO called `extras` (this
        # test file is itself extras.test_nebulaos_axis_twist), so
        # sys.modules['extras'] is already bound to THIS repo's package by
        # the time this test runs - a plain `import extras.
        # axis_twist_compensation` would resolve against the WRONG
        # `extras` package's __path__ and fail to find it. Register the
        # real pinned klippy/extras/ directory under a distinctly-named
        # synthetic package instead, so the real module's own
        # `from . import manual_probe, bed_mesh, probe` (relative imports)
        # still resolve correctly, against the real files, with zero
        # collision with this repo's own package.
        import importlib
        import importlib.util
        # probe.py (imported transitively via axis_twist_compensation.py
        # -> bed_mesh.py -> probe.py) does a plain top-level `import pins`,
        # matching how klippy.py itself runs it (klippy/ on sys.path, not
        # as a package) - pins.py itself is dependency-light (stdlib `re`
        # only), so this does not reopen the `extras` collision above.
        klippy_dir = os.path.join(_KLIPPER_SRC, 'klippy')
        if klippy_dir not in sys.path:
            sys.path.insert(0, klippy_dir)
        real_extras_dir = os.path.join(_KLIPPER_SRC, 'klippy', 'extras')
        pkg_name = '_nebulaos_test_real_klipper_extras'
        if pkg_name not in sys.modules:
            pkg_spec = importlib.util.spec_from_loader(pkg_name, loader=None,
                                                         is_package=True)
            pkg = importlib.util.module_from_spec(pkg_spec)
            pkg.__path__ = [real_extras_dir]
            sys.modules[pkg_name] = pkg
        cls.real_module = importlib.import_module(
            pkg_name + '.axis_twist_compensation')

    def _build_real_compensation(self, values=None):
        cfg_values = dict(
            calibrate_start_x=20., calibrate_end_x=200., calibrate_y=117.5,
            calibrate_start_y=40., calibrate_end_y=200., calibrate_x=117.5,
        )
        if values:
            cfg_values.update(values)
        printer = MiniFakePrinter()
        config = MiniFakeConfig(cfg_values, printer=printer)
        compensation = self.real_module.AxisTwistCompensation(config)
        return printer, compensation

    def test_real_object_constructs_and_exposes_calibrater(self):
        printer, compensation = self._build_real_compensation()
        self.assertIsNotNone(compensation.calibrater)
        self.assertEqual(compensation.calibrater.configname, 'axis_twist_compensation')

    def test_direct_finalize_call_matches_this_modules_expected_math(self):
        """Ground truth: call the REAL _finalize_calibration() directly
        (bypassing this project's coordinator entirely) and confirm it
        produces exactly the mean-centered values this module's own
        FakeCalibrater (and therefore every orchestration test above)
        assumes."""
        printer, compensation = self._build_real_compensation()
        calibrater = compensation.calibrater
        calibrater.results = [0.0, 1.0, 2.0]
        calibrater.current_axis = 'X'
        calibrater.gcmd = fake.FakeGCmd()
        calibrater._finalize_calibration()
        self.assertEqual(calibrater.results, [1.0, 0.0, -1.0])
        self.assertEqual(compensation.z_compensations, [1.0, 0.0, -1.0])

    def test_this_projects_coordinator_against_the_real_object_end_to_end(self):
        """The strongest test in this file: runs THIS PROJECT'S OWN
        cmd_axis_twist_calibrate against the real, unmodified upstream
        AxisTwistCompensation/Calibrater object - no fakes standing in for
        upstream at all."""
        printer, compensation = self._build_real_compensation()
        gcode = MiniFakeGCode()
        printer.objects['gcode'] = gcode
        printer.add_object('probe', FakeProbeObj())
        printer.add_object('configfile', FakeConfigFile())
        printer.add_object('axis_twist_compensation', compensation)
        printer.add_object('nebulaos_z_offset_probe', FakeZOffsetProbe(True))

        # nebulaos_calibration.py's coordinator expects the real
        # printer.get_reactor()/lookup_object('gcode') conventions used
        # throughout this test file's other fakes - adapt the mini-fake
        # printer minimally rather than rebuilding NebulaOSCalibration's
        # own config-reading path a third way.
        printer.get_reactor = lambda: fake.FakeReactor()
        config = fake.FakeConfig({}, section='nebulaos_calibration', printer=printer)
        coord = nebulaos_calibration.NebulaOSCalibration(config)

        stub = _measurement_sequence([0.0, 1.0, 2.0])
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = stub
        try:
            coord.cmd_axis_twist_calibrate(
                fake.FakeGCmd({'AXIS': 'X', 'METHOD': 'LOAD_CELL', 'SAMPLE_COUNT': '3'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

        # Identical to the direct-call ground truth above.
        self.assertEqual(compensation.z_compensations, [1.0, 0.0, -1.0])
        self.assertEqual(coord.axis_twist_x_state, 'complete')

    def test_zy_compensations_field_name_matches_real_upstream(self):
        """Guards the exact upstream attribute/option names this project
        depends on for Y - a typo here would silently write to a field
        upstream's own runtime correction never reads."""
        printer, compensation = self._build_real_compensation()
        calibrater = compensation.calibrater
        calibrater.results = [0.0, 1.0, 2.0]
        calibrater.current_axis = 'Y'
        calibrater.gcmd = fake.FakeGCmd()
        calibrater._finalize_calibration()
        self.assertEqual(compensation.zy_compensations, [1.0, 0.0, -1.0])
        self.assertEqual(compensation.z_compensations, [])  # untouched


if __name__ == '__main__':
    unittest.main()
