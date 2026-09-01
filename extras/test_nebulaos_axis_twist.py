# Tests for NEBULAOS_AXIS_TWIST_CALIBRATE (extras/nebulaos_calibration.py,
# Phase 2 calibration-framework mission; contact-safety stabilization
# rewrite).
#
# Automatic (LOAD_CELL) Axis Twist is now HARD BLOCKED pending hardware
# qualification of remote HX711 nozzle contact (see nebulaos_calibration.py's
# own header comment and the real safety incident this whole mission is
# built on: _evidence/overnight-hx711-investigation-20260831-233518/
# REPORT.md). Manual Axis Twist is no longer wrapped at all - call pristine
# upstream AXIS_TWIST_COMPENSATION_CALIBRATE directly.
#
# Three layers:
#   1. BedPointGenerationTest - axis_twist_bed_points() is pure point-
#      generation math, unaffected by the hard block, still exercised as
#      before.
#   2. GeometryPreflightTest - axis_twist_geometry_preflight(), the
#      CORRECTED (subtraction-based) pure geometry check a future
#      qualification mission will wire back in. NOT reachable from
#      cmd_axis_twist_calibrate() today (see HardBlockTest below) - tested
#      standalone so it is ready and proven correct in advance.
#   3. HardBlockTest/StatusTest - proves the live command performs ZERO
#      motion and ZERO hardware object lookups for every AXIS value.
#   4. RealUpstreamParityTest - imports the REAL pinned
#      axis_twist_compensation.py (58bd67db...) directly and proves the
#      real object's own _finalize_calibration() math matches what a
#      future re-activation of the LOAD_CELL path would rely on - kept as
#      a pinned-compatibility proof even though this project's own
#      coordinator does not call it today.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_axis_twist -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
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


# ---------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------

class FakeCalibrater:
    """Mirrors the REAL upstream Calibrater._finalize_calibration()
    algorithm (58bd67db..., axis_twist_compensation.py) exactly - mean-
    center the raw results, stage per-axis config fields, activate the
    live compensation array. RealUpstreamParityTest below proves this
    mirroring is accurate, not just self-consistent. Kept even though
    nothing in this project's live command path calls it any more - a
    future re-activation of the LOAD_CELL path will."""

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
    """Deliberately allows building a coordinator with NO
    axis_twist_compensation/z_offset_probe/probe objects registered at all
    (all three default to None/absent) - HardBlockTest relies on this to
    prove cmd_axis_twist_calibrate() never looks any of them up."""
    printer = fake.FakePrinter()
    gcode = fake.FakeGCode()
    printer.add_object('gcode', gcode)
    if probe_obj is not None:
        printer.add_object('probe', probe_obj)
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


# ---------------------------------------------------------------------
# Geometry / point generation (unaffected by the hard block)
# ---------------------------------------------------------------------

class BedPointGenerationTest(unittest.TestCase):
    def test_x_axis_three_points_matches_hand_computed_upstream_formula(self):
        comp = FakeAxisTwistCompensation(calibrate_start_x=20., calibrate_end_x=200.,
                                          calibrate_y=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        self.assertEqual(points, [(20.0, 117.5), (110.0, 117.5), (200.0, 117.5)])

    def test_y_axis_three_points_matches_hand_computed_upstream_formula(self):
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
        comp = FakeAxisTwistCompensation(calibrate_start_x=20., calibrate_end_x=200.,
                                          calibrate_y=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        self.assertEqual(points[0], (20.0, 117.5))  # NOT offset by y_offset=27


# ---------------------------------------------------------------------
# Corrected geometry preflight (§15/§16) - subtraction-based transform
# ---------------------------------------------------------------------

class GeometryPreflightTest(unittest.TestCase):
    """The corrected (subtraction-based) preflight math. An earlier
    session's live-patched version of this exact check used ADDITION
    (probe_target = point + offset) and wrongly concluded the real,
    hardware-tested config (calibrate_end_y=200, probe y_offset=+27,
    axis_maximum=223) was unsafe - a negative test built on that wrong
    conclusion then ran a real, unbounded physical touch on the printer
    (see nebulaos-klipper-loadcell-architecture-history.md /
    the overnight investigation report). The correct transform, proven
    directly against nebulaos_probe_pair.py's own real, hardware-
    qualified code, is SUBTRACTION - this file's own required test cases,
    per the mission, include the exact point that earlier mistake flagged
    as unsafe, now proven safe."""

    def test_current_x_geometry_passes(self):
        comp = FakeAxisTwistCompensation(calibrate_start_x=20., calibrate_end_x=200.,
                                          calibrate_y=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'X', 3)
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=0., probe_y_offset=27.,
            axis_minimum=0., axis_maximum=235.)
        self.assertEqual(len(result), 3)

    def test_y_200_passes_the_real_hardware_tested_config(self):
        # The exact point an earlier session's WRONG (addition-based)
        # preflight incorrectly rejected. axis_maximum=223 (real KE Y
        # limit), probe y_offset=+27: nozzle target Y=200 (binding
        # constraint, no offset), probe target Y=173 - both comfortably
        # inside [0, 223].
        comp = FakeAxisTwistCompensation(calibrate_start_y=40., calibrate_end_y=200.,
                                          calibrate_x=117.5)
        points = nebulaos_calibration.axis_twist_bed_points(comp, 'Y', 3)
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'Y', points, probe_x_offset=0., probe_y_offset=27.,
            axis_minimum=0., axis_maximum=223.)
        nozzle_target, probe_target = result[-1]
        self.assertEqual(nozzle_target, (117.5, 200.0))
        self.assertEqual(probe_target, (117.5, 173.0))

    def test_nozzle_only_invalid_point_rejects(self):
        # Nozzle target itself (no offset) exceeds axis_maximum - the
        # binding constraint per the mission's own corrected geometry.
        points = [(117.5, 300.0)]
        with self.assertRaises(ValueError) as ctx:
            nebulaos_calibration.axis_twist_geometry_preflight(
                'Y', points, probe_x_offset=0., probe_y_offset=27.,
                axis_minimum=0., axis_maximum=223.)
        self.assertIn('nozzle carriage target', str(ctx.exception))

    def test_probe_only_invalid_point_rejects(self):
        # Nozzle target (50) is fine, but a LARGE positive offset pushes
        # the probe target negative, out of bounds - proves both targets
        # are independently checked, not just the nozzle one.
        points = [(50.0, 50.0)]
        with self.assertRaises(ValueError) as ctx:
            nebulaos_calibration.axis_twist_geometry_preflight(
                'X', points, probe_x_offset=100., probe_y_offset=0.,
                axis_minimum=0., axis_maximum=235.)
        self.assertIn('probe carriage target', str(ctx.exception))

    def test_positive_offset(self):
        points = [(100.0, 100.0)]
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=27., probe_y_offset=0.,
            axis_minimum=0., axis_maximum=235.)
        self.assertEqual(result[0][1], (73.0, 100.0))

    def test_negative_offset(self):
        points = [(100.0, 100.0)]
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=-27., probe_y_offset=0.,
            axis_minimum=0., axis_maximum=235.)
        self.assertEqual(result[0][1], (127.0, 100.0))

    def test_zero_offset_means_nozzle_and_probe_targets_are_identical(self):
        points = [(100.0, 100.0)]
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=0., probe_y_offset=0.,
            axis_minimum=0., axis_maximum=235.)
        self.assertEqual(result[0][0], result[0][1])

    def test_limit_boundary_exactly_at_maximum_passes(self):
        points = [(235.0, 100.0)]
        result = nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=0., probe_y_offset=0.,
            axis_minimum=0., axis_maximum=235.)
        self.assertEqual(len(result), 1)

    def test_limit_boundary_just_over_maximum_rejects(self):
        points = [(235.001, 100.0)]
        with self.assertRaises(ValueError):
            nebulaos_calibration.axis_twist_geometry_preflight(
                'X', points, probe_x_offset=0., probe_y_offset=0.,
                axis_minimum=0., axis_maximum=235.)

    def test_configurable_safety_margin_shrinks_the_valid_range(self):
        # Passes with no margin, rejects once a 1mm margin is applied.
        points = [(234.5, 100.0)]
        nebulaos_calibration.axis_twist_geometry_preflight(
            'X', points, probe_x_offset=0., probe_y_offset=0.,
            axis_minimum=0., axis_maximum=235., safety_margin_mm=0.0)
        with self.assertRaises(ValueError):
            nebulaos_calibration.axis_twist_geometry_preflight(
                'X', points, probe_x_offset=0., probe_y_offset=0.,
                axis_minimum=0., axis_maximum=235., safety_margin_mm=1.0)

    def test_first_invalid_point_identified_not_a_later_one(self):
        points = [(300.0, 100.0), (100.0, 100.0), (300.0, 100.0)]
        with self.assertRaises(ValueError) as ctx:
            nebulaos_calibration.axis_twist_geometry_preflight(
                'X', points, probe_x_offset=0., probe_y_offset=0.,
                axis_minimum=0., axis_maximum=235.)
        self.assertIn('point 1/3', str(ctx.exception))

    def test_no_toolhead_parameter_at_all_proves_zero_movement(self):
        # The function signature itself has no printer/toolhead argument -
        # it is structurally impossible for this call to move anything.
        import inspect
        sig = inspect.signature(nebulaos_calibration.axis_twist_geometry_preflight)
        for name in sig.parameters:
            self.assertNotIn('printer', name.lower())
            self.assertNotIn('toolhead', name.lower())

    def test_unknown_axis_rejected(self):
        with self.assertRaises(ValueError):
            nebulaos_calibration.axis_twist_geometry_preflight(
                'Z', [(1.0, 1.0)], probe_x_offset=0., probe_y_offset=0.,
                axis_minimum=0., axis_maximum=235.)


# ---------------------------------------------------------------------
# Hard block: zero motion, zero hardware object lookups, for every AXIS
# ---------------------------------------------------------------------

class HardBlockTest(unittest.TestCase):
    def test_axis_required_no_default(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({}))
        self.assertIn('AXIS', str(ctx.exception))

    def test_unknown_axis_rejected(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'Z'}))

    def test_axis_x_raises_remote_load_cell_contact_unqualified(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))

    def test_axis_y_raises_remote_load_cell_contact_unqualified(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'Y'}))
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))

    def test_axis_both_raises_remote_load_cell_contact_unqualified(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'BOTH'}))
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))

    def test_no_axis_twist_compensation_object_needed(self):
        # No [axis_twist_compensation] object registered at all - proves
        # the hard-blocked command never looks it up (a real, unqualified
        # command would raise a DIFFERENT "not configured" error instead
        # of the REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED one).
        printer, gcode, coord = _build(axis_twist=None, z_offset_probe=None)
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))

    def test_no_z_offset_probe_object_needed(self):
        printer, gcode, coord = _build(
            axis_twist=FakeAxisTwistCompensation(), z_offset_probe=None)
        with self.assertRaises(fake.CommandError) as ctx:
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))

    def test_zero_gcode_scripts_run(self):
        printer, gcode, coord = _build(
            axis_twist=FakeAxisTwistCompensation(),
            z_offset_probe=FakeZOffsetProbe(True))
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'BOTH'}))
        self.assertEqual(gcode.scripts_run, [])

    def test_zero_compensation_arrays_touched(self):
        axis_twist = FakeAxisTwistCompensation()
        printer, gcode, coord = _build(
            axis_twist=axis_twist, z_offset_probe=FakeZOffsetProbe(True))
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'BOTH'}))
        self.assertEqual(axis_twist.z_compensations, [])
        self.assertEqual(axis_twist.zy_compensations, [])
        self.assertEqual(axis_twist.clear_calls, [])
        self.assertEqual(axis_twist.calibrater.finalize_calls, 0)

    def test_zero_config_set_calls(self):
        printer, gcode, coord = _build(
            axis_twist=FakeAxisTwistCompensation(),
            z_offset_probe=FakeZOffsetProbe(True))
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        configfile = printer.lookup_object('configfile')
        self.assertEqual(configfile.set_calls, [])

    def test_no_public_unsafe_override(self):
        # There is deliberately no FORCE=/UNSAFE=/OVERRIDE= gcode param
        # that bypasses the block.
        printer, gcode, coord = _build()
        for override_kwargs in ({'AXIS': 'X', 'FORCE': '1'},
                                 {'AXIS': 'X', 'UNSAFE': 'true'},
                                 {'AXIS': 'X', 'OVERRIDE': 'yes'}):
            with self.assertRaises(fake.CommandError) as ctx:
                coord.cmd_axis_twist_calibrate(fake.FakeGCmd(override_kwargs))
            self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', str(ctx.exception))


# ---------------------------------------------------------------------
# Status
# ---------------------------------------------------------------------

class StatusTest(unittest.TestCase):
    def test_axis_x_sets_capability_unqualified_state(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'X'}))
        status = coord.get_status(0.)
        self.assertEqual(status['axis_twist_x_state'], 'capability_unqualified')
        self.assertEqual(status['axis_twist_y_state'], 'idle')
        self.assertIn('REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED', status['axis_twist_x_error'])

    def test_axis_both_sets_capability_unqualified_on_both(self):
        printer, gcode, coord = _build()
        with self.assertRaises(fake.CommandError):
            coord.cmd_axis_twist_calibrate(fake.FakeGCmd({'AXIS': 'BOTH'}))
        status = coord.get_status(0.)
        self.assertEqual(status['axis_twist_x_state'], 'capability_unqualified')
        self.assertEqual(status['axis_twist_y_state'], 'capability_unqualified')

    def test_status_command_does_not_raise(self):
        printer, gcode, coord = _build()
        coord.cmd_calibration_status(fake.FakeGCmd())


# ---------------------------------------------------------------------
# Layer 2: real pinned upstream object parity (kept as a compatibility
# proof for a future re-activation, even though the live command path
# does not call this any more)
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
    proves the real object's own _finalize_calibration() math matches
    what axis_twist_geometry_preflight()/axis_twist_bed_points() and a
    future re-activated LOAD_CELL path would rely on."""

    @classmethod
    def setUpClass(cls):
        import importlib
        import importlib.util
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
        printer, compensation = self._build_real_compensation()
        calibrater = compensation.calibrater
        calibrater.results = [0.0, 1.0, 2.0]
        calibrater.current_axis = 'X'
        calibrater.gcmd = fake.FakeGCmd()
        calibrater._finalize_calibration()
        self.assertEqual(calibrater.results, [1.0, 0.0, -1.0])
        self.assertEqual(compensation.z_compensations, [1.0, 0.0, -1.0])

    def test_zy_compensations_field_name_matches_real_upstream(self):
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
