# Tests for extras/nebulaos_probe_pair.py (Phase 2 calibration-framework
# mission) - the shared paired-measurement primitive behind
# NEBULAOS_Z_OFFSET_CALIBRATE METHOD=LOAD_CELL and
# NEBULAOS_AXIS_TWIST_CALIBRATE METHOD=LOAD_CELL.
#
# Deliberately stubs probe.run_single_probe() and touch_probe() themselves
# (both already independently tested/trusted - upstream pristine and
# test_z_offset_probe_safety.py respectively) rather than reconstructing
# upstream's full probe-session machinery here. What THIS file exists to
# prove, per the mission's own "a sign regression is a crash-risk bug"
# instruction, is that this module's own arithmetic
# (raw_probe_trigger_z - raw_nozzle_contact_z, never the other way around)
# and its own XY-offset/move-ordering logic are correct - independent of
# whatever upstream's probe session internally does.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_probe_pair -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys
import types
import unittest

# nebulaos_probe_pair.py does `from . import probe as probe_module` at
# module level (upstream Klipper's real klippy/extras/probe.py, symlinked
# into place only at real build/composition time - see
# test_z_offset_probe_safety.py's own header comment for why this
# companion repo does not vendor a copy for standalone testing). Inject a
# placeholder BEFORE importing, so the module-level import succeeds; every
# test below then replaces .run_single_probe with its own stub anyway
# (same as monkeypatching any other already-imported dependency), so the
# placeholder's own body is never actually exercised.
if 'extras.probe' not in sys.modules:
    _placeholder = types.ModuleType('extras.probe')
    _placeholder.run_single_probe = lambda probe_obj, gcmd: None
    sys.modules['extras.probe'] = _placeholder

from . import nebulaos_probe_pair as pairmod


class FakeToolhead:
    def __init__(self, position=(110., 111., 5., 0.)):
        self._position = list(position)
        self.moves = []  # [(x_or_None, y_or_None, z_or_None, speed), ...]

    def get_position(self):
        return list(self._position)

    def manual_move(self, coord, speed):
        for i, v in enumerate(coord):
            if v is not None:
                self._position[i] = v
        self.moves.append((tuple(coord), speed))


class FakeGCode:
    def create_gcode_command(self, cmd, commandline, params):
        return ("fake-gcmd", params)


class FakePrinter:
    def __init__(self, toolhead, gcode, probe_obj=None):
        self._objects = {'toolhead': toolhead, 'gcode': gcode,
                          'probe': probe_obj if probe_obj is not None else object()}

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)


class FakeProbeResult:
    def __init__(self, test_z):
        self.test_z = test_z


class FakeZOffsetProbe:
    def __init__(self, contact_z):
        self.contact_z = contact_z
        self.calls = []

    def touch_probe(self, down_min_z, pro_cnt=1):
        self.calls.append({'down_min_z': down_min_z, 'pro_cnt': pro_cnt})
        return self.contact_z


class SignAndArithmeticTest(unittest.TestCase):
    """The single highest-value test class in this file, per the mission's
    own instruction: a sign regression here is a real crash-risk bug, not
    a cosmetic one."""

    def _measure(self, probe_trigger_z, nozzle_contact_z, **kwargs):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(nozzle_contact_z)
        orig_run_single_probe = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = \
            lambda probe_obj, gcmd: FakeProbeResult(probe_trigger_z)
        try:
            return pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.,
                **kwargs)
        finally:
            pairmod.probe_module.run_single_probe = orig_run_single_probe

    def test_probe_higher_than_nozzle_gives_positive_offset(self):
        # Probe triggers at Z=2.500 (in the current frame); the nozzle's
        # real contact is found lower, at Z=0.831. The probe therefore
        # reads 1.669mm higher than reality, and the correction must be
        # POSITIVE by exactly that amount - raw_probe_trigger_z MINUS
        # raw_nozzle_contact_z, in that order, not the reverse.
        result = self._measure(probe_trigger_z=2.500, nozzle_contact_z=0.831)
        self.assertAlmostEqual(result.probe_z_offset, 2.500 - 0.831, places=9)
        self.assertAlmostEqual(result.probe_z_offset, 1.669, places=9)

    def test_probe_lower_than_nozzle_gives_negative_offset(self):
        # The reverse case - proves the formula is not accidentally
        # symmetric/order-independent (e.g. via an abs() slipping in).
        result = self._measure(probe_trigger_z=0.831, nozzle_contact_z=2.500)
        self.assertAlmostEqual(result.probe_z_offset, 0.831 - 2.500, places=9)
        self.assertAlmostEqual(result.probe_z_offset, -1.669, places=9)

    def test_equal_readings_give_exactly_zero(self):
        result = self._measure(probe_trigger_z=1.234, nozzle_contact_z=1.234)
        self.assertEqual(result.probe_z_offset, 0.0)

    def test_raw_fields_are_reported_unmodified(self):
        result = self._measure(probe_trigger_z=-0.500, nozzle_contact_z=0.250)
        self.assertEqual(result.raw_probe_trigger_z, -0.500)
        self.assertEqual(result.raw_nozzle_contact_z, 0.250)

    def test_result_independent_of_starting_toolhead_position(self):
        # The paired-measurement result must depend only on the two raw
        # sensor readings, never on where the toolhead happened to start -
        # this is the same algebraic cancellation property documented in
        # the module docstring (any active gcode offset cancels out).
        for start in [(0., 0., 0., 0.), (200., 200., 30., 0.), (-5., -5., 1., 0.)]:
            toolhead = FakeToolhead(position=start)
            gcode = FakeGCode()
            printer = FakePrinter(toolhead, gcode)
            z_probe = FakeZOffsetProbe(0.831)
            orig = pairmod.probe_module.run_single_probe
            pairmod.probe_module.run_single_probe = \
                lambda probe_obj, gcmd: FakeProbeResult(2.500)
            try:
                result = pairmod.measure_probe_nozzle_pair(
                    printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                    horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.)
            finally:
                pairmod.probe_module.run_single_probe = orig
            self.assertAlmostEqual(result.probe_z_offset, 1.669, places=9)


class XYOffsetAndMoveOrderTest(unittest.TestCase):
    def test_probe_moves_to_offset_position_nozzle_moves_to_bare_xy(self):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(0.5)
        probed_xy_at_call_time = {}

        def fake_run_single_probe(probe_obj, gcmd):
            probed_xy_at_call_time['xy'] = tuple(toolhead.get_position()[:2])
            return FakeProbeResult(2.0)

        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = fake_run_single_probe
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.)
        finally:
            pairmod.probe_module.run_single_probe = orig

        # BLTouch's real y_offset=27: the PROBE tip must be positioned at
        # (100, 73), so that the probe itself (not the toolhead origin) is
        # physically over bed point (100, 100).
        self.assertEqual(probed_xy_at_call_time['xy'], (100.0, 73.0))
        # The nozzle contact must happen with the toolhead origin directly
        # over (100, 100) - no offset subtraction for the nozzle itself.
        nozzle_xy = (toolhead.get_position()[0], toolhead.get_position()[1])
        # (the final hover-off move doesn't change XY, so this still holds
        # after the call returns)
        self.assertEqual(nozzle_xy, (100.0, 100.0))

    def test_zero_probe_offset_means_probe_and_nozzle_targets_are_identical(self):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(0.5)
        seen = []

        def fake_run_single_probe(probe_obj, gcmd):
            seen.append(tuple(toolhead.get_position()[:2]))
            return FakeProbeResult(2.0)

        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = fake_run_single_probe
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=50., y=60., probe_x_offset=0., probe_y_offset=0.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.)
        finally:
            pairmod.probe_module.run_single_probe = orig
        self.assertEqual(seen[0], (50.0, 60.0))

    def test_every_xy_move_is_preceded_by_a_hover_to_horizontal_move_z(self):
        toolhead = FakeToolhead(position=(0., 0., 0., 0.))
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(0.5)
        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = \
            lambda probe_obj, gcmd: FakeProbeResult(2.0)
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.)
        finally:
            pairmod.probe_module.run_single_probe = orig

        # Every move that changes X or Y must be immediately preceded (not
        # necessarily immediately followed) by a pure Z move to
        # horizontal_move_z - never a diagonal XYZ move, and never an XY
        # move at the previous, potentially-much-lower Z.
        moves = toolhead.moves
        for i, (coord, _speed) in enumerate(moves):
            x, y, z = coord[0], coord[1], coord[2] if len(coord) > 2 else None
            changes_xy = x is not None or y is not None
            if changes_xy:
                self.assertGreater(
                    i, 0, "an XY move must never be the very first move")
                prev_coord = moves[i - 1][0]
                prev_z = prev_coord[2] if len(prev_coord) > 2 else None
                self.assertEqual(
                    prev_z, 8.,
                    "XY move at index %d was not preceded by a hover to "
                    "horizontal_move_z" % i)

    def test_down_min_z_and_pro_cnt_are_passed_through(self):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(0.5)
        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = \
            lambda probe_obj, gcmd: FakeProbeResult(2.0)
        try:
            pairmod.measure_probe_nozzle_pair(
                printer, x=100., y=100., probe_x_offset=0., probe_y_offset=27.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=12.5,
                pro_cnt=5)
        finally:
            pairmod.probe_module.run_single_probe = orig
        self.assertEqual(z_probe.calls, [{'down_min_z': 12.5, 'pro_cnt': 5}])

    def test_result_x_y_match_the_requested_point_not_any_offset_target(self):
        toolhead = FakeToolhead()
        gcode = FakeGCode()
        printer = FakePrinter(toolhead, gcode)
        z_probe = FakeZOffsetProbe(0.5)
        orig = pairmod.probe_module.run_single_probe
        pairmod.probe_module.run_single_probe = \
            lambda probe_obj, gcmd: FakeProbeResult(2.0)
        try:
            result = pairmod.measure_probe_nozzle_pair(
                printer, x=42., y=99., probe_x_offset=0., probe_y_offset=27.,
                horizontal_move_z=8., z_offset_probe=z_probe, down_min_z=10.)
        finally:
            pairmod.probe_module.run_single_probe = orig
        self.assertEqual((result.x, result.y), (42., 99.))


if __name__ == '__main__':
    unittest.main()
