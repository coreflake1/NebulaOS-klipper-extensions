# Tests for extras/nebulaos_calibration.py (Phase 2 calibration-framework
# mission) - the NebulaOS calibration coordinator's first slice:
# NEBULAOS_Z_OFFSET_CALIBRATE (both METHOD=LOAD_CELL and METHOD=MANUAL) and
# the thin PID/bed-mesh delegating wrappers.
#
# Deliberately stubs nebulaos_probe_pair.measure_probe_nozzle_pair() itself
# (already independently tested for sign/coordinate correctness in
# test_nebulaos_probe_pair.py) - what THIS file proves is the coordinator's
# OWN logic: preflight gating, magnitude/finite validation, the live
# probe-offset adapter, configfile staging, and command delegation.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_calibration -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys
import types
import unittest

from . import prtouch_test_support as fake

# nebulaos_calibration.py -> nebulaos_probe_pair.py -> `from . import probe`
# (upstream Klipper's real klippy/extras/probe.py, only present at real
# build/composition time - see test_nebulaos_probe_pair.py's own header for
# why this companion repo does not vendor a copy). Same placeholder-
# injection convention as that file.
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
    modeling exactly the two attributes nebulaos_calibration.py's live
    adapter depends on - probe_offsets.z_offset (mutable) and
    cmd_helper.name (for configfile.set()'s section argument) - both
    confirmed directly against the pinned bltouch.py source (see that
    module's own comment)."""

    def __init__(self, x_offset=0., y_offset=27., z_offset=0.):
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


def _stub_pair(probe_trigger_z, nozzle_contact_z):
    def fake_measure(printer, x, y, probe_x_offset, probe_y_offset,
                      horizontal_move_z, z_offset_probe, down_min_z,
                      pro_cnt=1, travel_speed=None, probe_lift_speed=None):
        return nebulaos_probe_pair.PairedMeasurement(
            x=x, y=y, raw_probe_trigger_z=probe_trigger_z,
            raw_nozzle_contact_z=nozzle_contact_z,
            probe_z_offset=probe_trigger_z - nozzle_contact_z)
    return fake_measure


class LoadCellHappyPathTest(unittest.TestCase):
    def test_measurement_applied_live_and_staged(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(x_offset=0., y_offset=27., z_offset=0.)
        zc = FakeZCompensate(home_x=110., home_y=111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)

        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=2.500, nozzle_contact_z=0.831)
        try:
            gcmd = fake.FakeGCmd({'METHOD': 'LOAD_CELL'})
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

    def test_default_method_is_load_cell(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(home_x=110., home_y=111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.0, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd())  # no METHOD= at all
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')

    def test_explicit_xy_override_bypasses_z_compensate(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj()
        printer, gcode, coord = _build(z_probe, probe_obj, z_compensate=None)
        seen = {}

        def fake_measure(printer_, x, y, *a, **kw):
            seen['xy'] = (x, y)
            return nebulaos_probe_pair.PairedMeasurement(
                x=x, y=y, raw_probe_trigger_z=1.0, raw_nozzle_contact_z=0.5,
                probe_z_offset=0.5)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = fake_measure
        try:
            coord.cmd_z_offset_calibrate(
                fake.FakeGCmd({'METHOD': 'LOAD_CELL', 'X': '42', 'Y': '99'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(seen['xy'], (42.0, 99.0))


class LoadCellPreflightTest(unittest.TestCase):
    def test_no_load_cell_configured_raises_and_mentions_manual(self):
        printer, gcode, coord = _build(z_offset_probe=None,
                                        z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        self.assertIn('METHOD=MANUAL', str(ctx.exception))

    def test_uncalibrated_load_cell_raises_and_mentions_manual(self):
        z_probe = FakeZOffsetProbe(is_calibrated=False)
        printer, gcode, coord = _build(z_probe, z_compensate=FakeZCompensate(110., 111.))
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        msg = str(ctx.exception)
        self.assertIn('LOAD_CELL_CALIBRATE', msg)
        self.assertIn('METHOD=MANUAL', msg)

    def test_no_reference_point_available_raises(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        printer, gcode, coord = _build(z_probe, z_compensate=None)
        with self.assertRaises(fake.CommandError):
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))

    def test_unknown_method_raises(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'BOGUS'}))
        self.assertIn('BOGUS', str(ctx.exception))


class LoadCellValidationTest(unittest.TestCase):
    def test_implausibly_large_correction_is_rejected_and_not_applied(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=0.0)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        # 5mm apart - exceeds the default 2mm max_offset_correction_mm.
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertIn('max_offset_correction_mm', str(ctx.exception))
        # Must NOT have mutated the live probe offset or staged anything.
        self.assertEqual(probe_obj.probe_offsets.z_offset, 0.0)
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])
        self.assertEqual(coord.z_offset_state, 'error')
        self.assertIn('max_offset_correction_mm', coord.z_offset_error)

    def test_non_finite_measurement_is_rejected(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=0.0)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, probe_obj, zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=float('nan'), nozzle_contact_z=0.0)
        try:
            with self.assertRaises(fake.CommandError):
                coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(probe_obj.probe_offsets.z_offset, 0.0)

    def test_custom_max_offset_correction_mm_is_honored(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        probe_obj = FakeProbeObj(z_offset=0.0)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(
            z_probe, probe_obj, zc,
            config_overrides={'max_offset_correction_mm': '10'})
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(probe_trigger_z=5.0, nozzle_contact_z=0.0)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
        finally:
            nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = orig
        self.assertEqual(coord.z_offset_state, 'complete')
        self.assertAlmostEqual(probe_obj.probe_offsets.z_offset, 5.0, places=9)


class ManualMethodTest(unittest.TestCase):
    def test_manual_delegates_to_stock_probe_calibrate(self):
        printer, gcode, coord = _build()
        coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'MANUAL'}))
        self.assertEqual(gcode.scripts_run, ['PROBE_CALIBRATE'])
        # Manual mode must not touch z_offset_state - it is an entirely
        # separate, interactive upstream workflow.
        self.assertEqual(coord.z_offset_state, 'idle')


class PidAndBedMeshWrapperTest(unittest.TestCase):
    def test_pid_calibrate_bed_uses_default_target(self):
        printer, gcode, coord = _build()
        coord.cmd_pid_calibrate_bed(fake.FakeGCmd())
        self.assertEqual(gcode.scripts_run,
                          ['PID_CALIBRATE HEATER=heater_bed TARGET=65.00'])

    def test_pid_calibrate_bed_honors_target_override(self):
        printer, gcode, coord = _build()
        coord.cmd_pid_calibrate_bed(fake.FakeGCmd({'TARGET': '70'}))
        self.assertEqual(gcode.scripts_run,
                          ['PID_CALIBRATE HEATER=heater_bed TARGET=70.00'])

    def test_pid_calibrate_hotend_uses_default_target(self):
        printer, gcode, coord = _build()
        coord.cmd_pid_calibrate_hotend(fake.FakeGCmd())
        self.assertEqual(gcode.scripts_run,
                          ['PID_CALIBRATE HEATER=extruder TARGET=230.00'])

    def test_bed_mesh_calibrate_saves_named_profile_by_default(self):
        printer, gcode, coord = _build()
        coord.cmd_bed_mesh_calibrate(fake.FakeGCmd())
        self.assertEqual(gcode.scripts_run,
                          ['BED_MESH_CALIBRATE',
                           'BED_MESH_PROFILE SAVE="nebulaos_calibration"'])

    def test_bed_mesh_calibrate_honors_profile_override(self):
        printer, gcode, coord = _build()
        coord.cmd_bed_mesh_calibrate(fake.FakeGCmd({'PROFILE': 'my_profile'}))
        self.assertEqual(gcode.scripts_run,
                          ['BED_MESH_CALIBRATE', 'BED_MESH_PROFILE SAVE="my_profile"'])


class StatusTest(unittest.TestCase):
    def test_get_status_reflects_last_result(self):
        z_probe = FakeZOffsetProbe(is_calibrated=True)
        zc = FakeZCompensate(110., 111.)
        printer, gcode, coord = _build(z_probe, FakeProbeObj(), zc)
        orig = nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair
        nebulaos_calibration.nebulaos_probe_pair.measure_probe_nozzle_pair = \
            _stub_pair(1.5, 0.5)
        try:
            coord.cmd_z_offset_calibrate(fake.FakeGCmd({'METHOD': 'LOAD_CELL'}))
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
