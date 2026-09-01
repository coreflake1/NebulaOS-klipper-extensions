# Tests for extras/nebulaos_calibration.py (Phase 2 calibration-framework
# mission; contact-safety stabilization, corrected) - the NebulaOS
# calibration coordinator: NEBULAOS_Z_OFFSET_CALIBRATE (LOAD_CELL only -
# manual users call pristine upstream PROBE_CALIBRATE directly) and
# NEBULAOS_CALIBRATION_STATUS. The PID-default-target and bed-mesh-named-
# profile Python wrappers have been removed (upstream-first cleanup -
# see nebulaos_calibration.py's own header for why the bed-mesh one was a
# real duplication: pinned BED_MESH_CALIBRATE PROFILE=<name> already does
# exactly what the wrapper did).
#
# Deliberately stubs nebulaos_probe_pair.measure_probe_nozzle_pair() itself
# (already independently tested for sign/coordinate/envelope correctness
# in test_nebulaos_probe_pair.py) - what THIS file proves is the
# coordinator's OWN logic: preflight gating, magnitude/finite validation,
# the live probe-offset adapter (including forwarding the CURRENT
# probe_z_offset and the two new envelope constants unchanged), configfile
# staging, and command delegation.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
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


class FakeProbeOffsets:
    def __init__(self, x_offset, y_offset, z_offset):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset


class FakeCmdHelper:
    def __init__(self, name):
        self.name = name


class FakeProbeObj:
    """Stands in for the real registered 'probe' object (PrinterBLTouch),
    modeling exactly the three attributes nebulaos_calibration.py's live
    adapter depends on - probe_offsets.z_offset (mutable, the CURRENT
    prior this whole workflow is refining), probe_offsets.x_offset/
    y_offset, and cmd_helper.name (for configfile.set()'s section
    argument) - all confirmed directly against the pinned bltouch.py
    source (see that module's own comment). Defaults to a CREDIBLE
    (nonzero) z_offset, matching the real captured hardware state most
    tests in this file assume - tests that specifically need the
    BOOTSTRAP/virgin case pass z_offset=0. explicitly."""

    def __init__(self, x_offset=0., y_offset=27., z_offset=1.795):
        self.probe_offsets = FakeProbeOffsets(x_offset, y_offset, z_offset)
        self.cmd_helper = FakeCmdHelper('bltouch')

    def get_offsets(self, gcmd=None):
        o = self.probe_offsets
        return o.x_offset, o.y_offset, o.z_offset


class FakeZOffsetProbe:
    def __init__(self, is_calibrated=True):
        self._is_calibrated = is_calibrated

    def get_status(self, eventtime):
        return {'is_calibrated': self._is_calibrated}


class FakeZCompensate:
    def __init__(self, home_x, home_y):
        self.home_x = home_x
        self.home_y = home_y


class FakeConfigFile:
    def __init__(self):
        self.set_calls = []

    def set(self, section, option, value):
        self.set_calls.append((section, option, value))


def _build(z_offset_probe=None, probe_obj=None, z_compensate=None,
           config_overrides=None):
    printer = fake.FakePrinter()
    gcode = fake.FakeGCode()
    printer.add_object('gcode', gcode)
    printer.add_object('probe', probe_obj if probe_obj is not None else FakeProbeObj())
    printer.add_object('configfile', FakeConfigFile())
    if z_offset_probe is not None:
        printer.add_object('nebulaos_z_offset_probe', z_offset_probe)
    if z_compensate is not None:
        printer.add_object('z_compensate', z_compensate)

    values = dict(config_overrides or {})
    config = fake.FakeConfig(values, section='nebulaos_calibration', printer=printer)
    coordinator = nebulaos_calibration.NebulaOSCalibration(config)
    config.assert_all_consumed()
    return printer, gcode, coordinator


def _accepted_measurement(x, y, probe_trigger_z, nozzle_contact_z,
                           contact_mode='established'):
    repeatability = nebulaos_probe_pair.RepeatabilityResult(
        sample_count=1, accepted_count=1, samples=[], mean=nozzle_contact_z,
        minimum=nozzle_contact_z, maximum=nozzle_contact_z, range=0.0,
        stddev=0.0, accepted=True, rejection_reason=None)
    return nebulaos_probe_pair.PairedMeasurement(
        x=x, y=y, contact_id=1.0, contact_mode=contact_mode,
        predicted_nozzle_contact_z=probe_trigger_z - 1.795,
        commanded_floor_z=probe_trigger_z - 1.795 - 1.0,
        raw_probe_trigger_z=probe_trigger_z,
        raw_nozzle_contact_z=nozzle_contact_z,
        repeatability=repeatability,
        probe_z_offset=probe_trigger_z - nozzle_contact_z,
        accepted=True, rejection_reason=None,
        trigger_force=75.0, force_safety_limit=2000.0, contact_speed=2.0)


def _rejected_measurement(x, y, reason='excessive_fit_delta(9.0>1.0)'):
    repeatability = nebulaos_probe_pair.RepeatabilityResult(
        sample_count=1, accepted_count=0, samples=[], mean=None,
        minimum=None, maximum=None, range=None, stddev=None,
        accepted=False, rejection_reason=reason)
    return nebulaos_probe_pair.PairedMeasurement(
        x=x, y=y, contact_id=1.0, contact_mode='established',
        predicted_nozzle_contact_z=0.2, commanded_floor_z=-0.8,
        raw_probe_trigger_z=2.0,
        raw_nozzle_contact_z=None, repeatability=repeatability,
        probe_z_offset=None, accepted=False, rejection_reason=reason,
        trigger_force=75.0, force_safety_limit=2000.0, contact_speed=2.0)


def _stub_pair(probe_trigger_z, nozzle_contact_z):
    def fake_measure(printer, x, y, probe_x_offset, probe_y_offset,
                      probe_z_offset,
                      horizontal_move_z, z_offset_probe, down_min_z,
                      pro_cnt=1, travel_speed=None, probe_lift_speed=None,
                      established_contact_margin_mm=None,
                      bootstrap_contact_envelope_mm=None,
                      max_abs_fit_delta=None, min_accepted_samples=None,
                      max_repeatability_range=None,
                      max_repeatability_stddev=None):
        return _accepted_measurement(x, y, probe_trigger_z, nozzle_contact_z)
    return fake_measure


class LoadCellHappyPathTest(unittest.TestCase):
    def test_measurement_applied_live_and_staged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(x_offset=0., y_offset=27., z_offset=1.795)
        zc = FakeZCompensate(home_x=110., home_y=111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.500, nozzle_contact_z=0.831)
        try:
            gcmd = fake.FakeGCmd({})
            coord.cmd_z_offset_calibrate(gcmd)
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.669, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(len(configfile.set_calls), 1)
        section, option, value = configfile.set_calls[0]
        self.assertEqual(section, 'bltouch')
        self.assertEqual(option, 'z_offset')
        self.assertAlmostEqual(float(value), 1.669, places=2)
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(coord.z_offset_result, 1.669, places=9)
        self.assertIsNone(coord.z_offset_error)

    def test_explicit_xy_override_bypasses_z_compensate(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj()
        printer, gcode, coord = _build(z_probe, probe_obj, z_compensate=None)
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'X': '42', 'Y': '99'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (42.0, 99.0))

    def test_current_probe_z_offset_forwarded_unchanged(self):
        # The coordinator must forward the CURRENT live probe z_offset
        # (the prior this workflow is refining) exactly as read from
        # probe_obj.get_offsets() - not re-derive or clamp it.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        seen = {}

        def fake_measure(printer_, x, y, probe_x_offset, probe_y_offset,
                          probe_z_offset, *a, **kw):
            seen['probe_z_offset'] = probe_z_offset
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(seen['probe_z_offset'], 1.795, places=9)


class LoadCellPreflightTest(unittest.TestCase):
    def test_no_load_cell_configured_raises_and_mentions_probe_calibrate(self):
        printer, gcode, coord = _build(z_offset_probe=None,
                                        z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        self.assertIn('PROBE_CALIBRATE', str(ctx.exception))

    def test_uncalibrated_load_cell_raises_and_mentions_probe_calibrate(self):
        z_probe = FakeZOffsetProbe(is_calibrated=False)
        printer, gcode, coord = _build(z_probe, z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        msg = str(ctx.exception)
        self.assertIn('LOAD_CELL_CALIBRATE', msg)
        self.assertIn('PROBE_CALIBRATE', msg)

    def test_no_reference_point_available_raises(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(z_probe, z_compensate=None)
        with self.assertRaises(fake.CommandError):
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))

    def test_no_method_parameter_accepted_or_needed(self):
        # There is no METHOD= dispatch any more - the command works with
        # a bare gcmd, and passing METHOD= (if a caller still does, out of
        # habit) is simply ignored, not rejected, since gcmd.get() calls
        # for it no longer exist in this handler at all.
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.0, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')


class LoadCellValidationTest(unittest.TestCase):
    def test_implausibly_large_correction_is_rejected_and_not_applied(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        # 5mm apart - exceeds the default 2mm max_offset_correction_mm.
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertIn('max_offset_correction_mm', str(ctx.exception))
        # Must NOT have mutated the live probe offset or staged anything.
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'error')
        self.assertIn('max_offset_correction_mm', coord.z_offset_error)

    def test_non_finite_measurement_is_rejected(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=float('nan'), nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 1.795, places=9)

    def test_custom_max_offset_correction_mm_is_honored(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=1.795)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(
            z_probe, probe_obj, zc,
            config_overrides={'max_offset_correction_mm': '10'})
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 5.0, places=9)


class LoadCellRejectionGatingTest(unittest.TestCase):
    """§9: a rejected measurement must never stage a Z-offset, transition
    to COMPLETE, or call SAVE_CONFIG. Also verifies the coordinator's own
    envelope/quality/repeatability config values are passed through to
    measure_probe_nozzle_pair() unchanged (not re-derived a second way)."""

    def test_rejected_measurement_is_not_applied_or_staged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=0.25)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        def fake_measure(printer_, x, y, *a, **kw):
            return _rejected_measurement(x, y)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig

        self.assertIn('measurement rejected', str(ctx.exception))
        self.assertIn('excessive_fit_delta', str(ctx.exception))
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 0.25, places=9)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'measurement_quality_failure')

    def test_capability_unqualified_error_sets_distinct_state(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)

        def fake_measure(printer_, x, y, *a, **kw):
            raise printer_.command_error(
                "CONTACT_SAFETY_LIMIT_UNQUALIFIED: established_contact_"
                "margin_mm is not configured")
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'capability_unqualified')

    def test_envelope_and_quality_config_passed_through_unchanged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), zc,
            config_overrides={
                'established_contact_margin_mm': '1.5',
                'bootstrap_contact_envelope_mm': '8.0',
                'max_abs_fit_delta': '0.3', 'min_accepted_samples': '2',
                'max_repeatability_range': '0.1', 'max_repeatability_stddev': '0.05'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen.update(kw)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertAlmostEqual(seen['established_contact_margin_mm'], 1.5, places=9)
        self.assertAlmostEqual(seen['bootstrap_contact_envelope_mm'], 8.0, places=9)
        self.assertAlmostEqual(seen['max_abs_fit_delta'], 0.3, places=9)
        self.assertEqual(seen['min_accepted_samples'], 2)
        self.assertAlmostEqual(seen['max_repeatability_range'], 0.1, places=9)
        self.assertAlmostEqual(seen['max_repeatability_stddev'], 0.05, places=9)

    def test_diagnostics_recorded_in_status_after_success(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(2.0, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['z_offset_contact_mode'], 'established')
        self.assertAlmostEqual(
            status['z_offset_predicted_nozzle_contact_z'], 2.0 - 1.795, places=9)
        self.assertEqual(status['z_offset_raw_probe_trigger_z'], 2.0)
        self.assertEqual(status['z_offset_accepted_count'], 1)


class ReferenceXYTest(unittest.TestCase):
    """§11: the canonical reference point now comes from this section's
    own reference_x/reference_y config, not [z_compensate]'s home_x/
    home_y - which stays only as a fallback for an older printer.cfg."""

    def test_configured_reference_xy_used_when_no_explicit_override(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.),
            config_overrides={'reference_x': '20', 'reference_y': '25'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (20.0, 25.0))

    def test_explicit_gcmd_xy_overrides_configured_reference(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.),
            config_overrides={'reference_x': '20', 'reference_y': '25'})
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'X': '42', 'Y': '99'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (42.0, 99.0))

    def test_falls_back_to_z_compensate_home_xy_when_reference_unset(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(
            z_probe, FakeProbeObj(), z_compensate=FakeZCompensate(110., 111.))
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return _accepted_measurement(x, y, 1.0, 0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (110.0, 111.0))


class UpstreamFirstCleanupTest(unittest.TestCase):
    """Confirms the removed Python wrappers are genuinely gone, not just
    unregistered - a future re-addition should not silently reintroduce
    the exact duplication this cleanup removed."""

    def test_no_manual_method_handler_exists(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, '_z_offset_calibrate_manual'))

    def test_no_pid_wrapper_methods_exist(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, 'cmd_pid_calibrate_bed'))
        self.assertFalse(hasattr(coord, 'cmd_pid_calibrate_hotend'))

    def test_no_bed_mesh_wrapper_method_exists(self):
        printer, gcode, coord = _build()
        self.assertFalse(hasattr(coord, 'cmd_bed_mesh_calibrate'))

    def test_no_pid_or_bed_mesh_or_manual_commands_registered(self):
        printer, gcode, coord = _build()
        self.assertEqual(
            sorted(gcode.commands.keys()),
            ['NEBULAOS_AXIS_TWIST_CALIBRATE', 'NEBULAOS_CALIBRATION_STATUS',
             'NEBULAOS_Z_OFFSET_CALIBRATE'])


class StatusTest(unittest.TestCase):
    def test_get_status_reflects_last_result(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.5, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        status = coord.get_status(0.)
        self.assertEqual(status['z_offset_state'], 'complete')
        self.assertEqual(status['z_offset_id'], 1)
        self.assertAlmostEqual(status['z_offset_result'], 1.0, places=9)
        self.assertIsNone(status['z_offset_error'])

    def test_status_command_does_not_raise_when_idle(self):
        printer, gcode, coord = _build()
        coord.cmd_calibration_status(fake.FakeGCmd())  # must not raise


if __name__ == '__main__':
    unittest.main()
