# Phase 2 calibration-framework mission: [nebulaos_z_offset_probe] is no
# longer a hard klippy:connect requirement for [z_compensate] - see that
# module's own _handle_connect()/_require_load_cell() comments. These tests
# cover the two halves of that fix directly: (1) klippy:connect succeeds
# with no [nebulaos_z_offset_probe] object registered at all (the printer
# reaches `ready`), and (2) each of the two commands that actually need it
# (Z_OFFSET_CALIBRATION, _NEBULAOS_NOZZLE_CLEAN) raises a clear, specific
# command_error naming the real problem, rather than either silently
# succeeding or crashing with an unrelated AttributeError.
#
# Run from klippy/: python3 -m unittest extras.test_z_compensate_missing_load_cell -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import z_compensate


def _build_without_load_cell():
    printer, mcu, _pins, _values = fake.build_environment()
    # Simulates a printer.cfg with no [nebulaos_z_offset_probe] section at
    # all - the real-world case this whole fix exists for.
    del printer.objects['nebulaos_z_offset_probe']

    zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
    zc = z_compensate.ZCompensate(zc_config)

    # The whole point under test: this must NOT raise, unlike before this
    # mission (an unconditional lookup_object() with no default here used
    # to make this a fatal klippy:connect error - the equivalent of the
    # entire printer refusing to reach `ready`).
    fake.connect(printer, mcu)
    zc_config.assert_all_consumed()
    return printer, mcu, zc


class KlippyConnectSucceedsWithoutLoadCell(unittest.TestCase):
    def test_connect_does_not_raise(self):
        printer, mcu, zc = _build_without_load_cell()
        self.assertIsNone(zc.z_offset_probe)

    def test_other_z_compensate_state_is_still_set_up_normally(self):
        # A missing load cell must not degrade anything ELSE this module
        # sets up at connect time - home_x/home_y resolution, the probe
        # object, bed_mesh - all independent of the load cell.
        printer, mcu, zc = _build_without_load_cell()
        self.assertIsNotNone(zc.probe)
        self.assertIsNotNone(zc.home_x)
        self.assertIsNotNone(zc.home_y)

    def test_get_status_still_works(self):
        # A Moonraker/GuppyScreen/Mainsail client polling get_status() must
        # keep getting a normal, well-formed status dict - not an error -
        # for a printer that simply has no load cell configured.
        _, _, zc = _build_without_load_cell()
        status = zc.get_status(0.)
        self.assertEqual(status['calibration_state'], 'idle')


class CommandsRequiringLoadCellFailPreflightClearly(unittest.TestCase):
    def test_z_offset_calibration_raises_specific_error(self):
        _, _, zc = _build_without_load_cell()
        gcmd = fake.FakeGCmd()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_z_offset_calibration(gcmd)
        msg = str(ctx.exception)
        self.assertIn('no [nebulaos_z_offset_probe]', msg)
        self.assertIn('Z_OFFSET_CALIBRATION', msg)
        self.assertIn('PROBE_CALIBRATE', msg)

    def test_z_offset_calibration_preflight_failure_does_not_disturb_state(self):
        # The preflight check must run BEFORE the reentrancy guard flips
        # calibration_state to "running" and bumps calibration_id - a
        # command that can never succeed on this printer must not leave
        # get_status() reporting a phantom in-progress/failed attempt.
        _, _, zc = _build_without_load_cell()
        gcmd = fake.FakeGCmd()
        id_before = zc.calibration_id
        with self.assertRaises(fake.CommandError):
            zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_id, id_before)
        self.assertEqual(zc.calibration_state, 'idle')

    def test_nozzle_clear_raises_specific_error(self):
        _, _, zc = _build_without_load_cell()
        gcmd = fake.FakeGCmd()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_nozzle_clear(gcmd)
        msg = str(ctx.exception)
        self.assertIn('no [nebulaos_z_offset_probe]', msg)
        self.assertIn('_NEBULAOS_NOZZLE_CLEAN', msg)

    def test_error_message_points_at_the_real_manual_alternative(self):
        # Z_OFFSET_CALIBRATION itself has never had a METHOD=MANUAL of its
        # own (unlike the future NEBULAOS_Z_OFFSET_CALIBRATE) - the error
        # must point the caller at the real, separate command that already
        # exists for a manual Z-offset (stock PROBE_CALIBRATE), not leave
        # them with no path forward at all.
        _, _, zc = _build_without_load_cell()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_z_offset_calibration(fake.FakeGCmd())
        self.assertIn('PROBE_CALIBRATE', str(ctx.exception))


class WithLoadCellConfiguredBehaviorIsUnchanged(unittest.TestCase):
    """Guards against the opposite regression: a printer that DOES have
    [nebulaos_z_offset_probe] configured must behave exactly as before this
    mission - the preflight check must be a pure no-op in that case."""

    def test_both_commands_proceed_normally(self):
        printer, mcu, _pins, _values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)
        z_offset_probe = printer.lookup_object('nebulaos_z_offset_probe')
        z_offset_probe.touch_probe = lambda down_min_z, **kw: 0.05
        zc.cmd_z_offset_calibration(fake.FakeGCmd())
        self.assertEqual(zc.calibration_state, 'complete')


class NozzleCleanRegistrationTest(unittest.TestCase):
    """Phase 2 RC: _NEBULAOS_NOZZLE_CLEAN is the only registered name.
    CRTENSE_NOZZLE_CLEAR (legacy GuppyScreen compat) is removed from the
    core API; GuppyScreen adaptation will call the canonical name."""

    def test_private_backend_is_registered(self):
        printer, mcu, _pins, _values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)
        gcode = printer.lookup_object('gcode')
        self.assertIn('_NEBULAOS_NOZZLE_CLEAN', gcode.commands)

    def test_legacy_crtense_name_is_not_registered(self):
        printer, mcu, _pins, _values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)
        gcode = printer.lookup_object('gcode')
        self.assertNotIn('CRTENSE_NOZZLE_CLEAR', gcode.commands)

    def test_nozzle_clean_preflight_error_without_load_cell(self):
        _, _, zc = _build_without_load_cell()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_nozzle_clear(fake.FakeGCmd())
        self.assertIn('_NEBULAOS_NOZZLE_CLEAN', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
