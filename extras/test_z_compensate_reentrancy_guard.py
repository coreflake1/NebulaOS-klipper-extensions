# Regression tests for the 2026-08-10 Z_OFFSET_CALIBRATION non-reentrancy guard (see
# docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md and z_compensate.py's own guard comment) - a
# second, higher-level guard on top of prtouch_probe.py's own PrtouchProbe._own_raw_operation
# (see test_prtouch_raw_op_guard.py), protecting the whole multi-step calibration sequence
# (positioning move + touch_probe + SET_GCODE_OFFSET) as one logical unit, not just each
# individual raw MCU dispatch within it.
#
# Run from klippy/: python3 -m unittest extras.test_z_compensate_reentrancy_guard -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import z_compensate


def _build(stub_measurement=0.0):
    printer, mcu, _pins, _values = fake.build_environment()

    zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
    zc = z_compensate.ZCompensate(zc_config)

    fake.connect(printer, mcu)
    zc_config.assert_all_consumed()

    calls = []

    def fake_touch_probe(down_min_z, **kwargs):
        calls.append({'down_min_z': down_min_z, 'kwargs': kwargs})
        return stub_measurement

    z_offset_probe = printer.lookup_object('nebulaos_z_offset_probe')
    z_offset_probe.touch_probe = fake_touch_probe
    return printer, mcu, z_offset_probe, zc, calls


class ReentrancyGuardTest(unittest.TestCase):
    def test_second_call_while_running_is_rejected_before_any_motion(self):
        _, _, z_probe, zc, calls = _build(stub_measurement=0.05)
        gcmd = fake.FakeGCmd()

        def reentrant_touch_probe(down_min_z, **kwargs):
            calls.append({'down_min_z': down_min_z, 'kwargs': kwargs})
            with self.assertRaises(fake.CommandError) as ctx:
                zc.cmd_z_offset_calibration(fake.FakeGCmd())
            self.assertIn('already in progress', str(ctx.exception))
            return 0.05

        z_probe.touch_probe = reentrant_touch_probe
        zc.cmd_z_offset_calibration(gcmd)
        # only the outer call's own probe attempt happened - the rejected reentrant call
        # never reached touch_probe() a second time.
        self.assertEqual(len(calls), 1)
        offset_scripts = [s for s in zc.gcode.scripts_run if 'SET_GCODE_OFFSET' in s]
        self.assertEqual(len(offset_scripts), 1)

    def test_rejected_call_does_not_bump_calibration_id_or_touch_status(self):
        _, _, z_probe, zc, calls = _build(stub_measurement=0.05)
        gcmd = fake.FakeGCmd()
        seen = {}

        def reentrant_touch_probe(down_min_z, **kwargs):
            seen['id_before_reentry'] = zc.calibration_id
            try:
                zc.cmd_z_offset_calibration(fake.FakeGCmd())
            except fake.CommandError:
                pass
            seen['id_after_reentry'] = zc.calibration_id
            seen['state_after_reentry'] = zc.calibration_state
            return 0.05

        z_probe.touch_probe = reentrant_touch_probe
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(seen['id_before_reentry'], seen['id_after_reentry'],
                          "a rejected reentrant call must not bump calibration_id")
        self.assertEqual(seen['state_after_reentry'], "running",
                          "a rejected reentrant call must not disturb the in-progress state")

    def test_guard_clears_after_success_allowing_next_sequential_call(self):
        _, _, z_probe, zc, calls = _build(stub_measurement=0.05)
        gcmd = fake.FakeGCmd()
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_state, "complete")
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_state, "complete")
        self.assertEqual(len(calls), 2)

    def test_guard_clears_after_failure_allowing_next_sequential_call(self):
        printer, mcu, _pins, _values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)

        z_offset_probe = printer.lookup_object('nebulaos_z_offset_probe')

        def raising_touch_probe(down_min_z, **kwargs):
            raise fake.CommandError("simulated no-trigger failure")

        z_offset_probe.touch_probe = raising_touch_probe
        gcmd = fake.FakeGCmd()
        with self.assertRaises(fake.CommandError):
            zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_state, "error")

        z_offset_probe.touch_probe = lambda down_min_z, **kw: 0.10
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_state, "complete")


# Final pre-hardware closure (2026-09-06): Z_OFFSET_CALIBRATION's public API
# audit found no [gcode_macro] wrapper and no composed-config caller, which
# looks identical to a stale alias until NebulaOS-guppyscreen's own source is
# checked - its recalibration_wizard_panel.cpp calls this exact name over the
# Moonraker websocket. These tests prove the two commands this mission's
# audit could otherwise confuse are genuinely distinct, independently
# registered implementations, not one aliasing the other.
class PublicAPISurfaceTest(unittest.TestCase):
    def test_z_offset_calibration_is_registered_directly_on_gcode(self):
        printer, mcu, _pins, _values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)

        gcode = printer.lookup_object('gcode')
        self.assertIn('Z_OFFSET_CALIBRATION', gcode.commands)
        self.assertEqual(gcode.commands['Z_OFFSET_CALIBRATION'], zc.cmd_z_offset_calibration)

    def test_z_offset_calibration_help_names_its_real_caller_and_the_canonical_alternative(self):
        # The exact wording is not load-bearing - only that a reader (or a
        # future audit) sees GuppyScreen and NEBULAOS_Z_OFFSET_CALIBRATE
        # named here, so this command is never again misclassified as an
        # orphaned alias.
        help_text = z_compensate.ZCompensate.cmd_z_offset_calibration_help
        self.assertIn('GuppyScreen', help_text)
        self.assertIn('NEBULAOS_Z_OFFSET_CALIBRATE', help_text)

    def test_z_offset_calibration_and_nebulaos_z_offset_calibrate_backend_are_different_modules(self):
        # _NEBULAOS_Z_OFFSET_CALIBRATE (the canonical guided workflow's
        # private backend) lives entirely in nebulaos_calibration.py, not
        # here - confirms these are two independent implementations behind
        # similarly-named commands, not one command aliasing the other.
        # Source-text inspection, not a live import: nebulaos_calibration.py
        # imports upstream Klipper's real probe.py at module scope, which
        # only resolves inside a fully composed Klipper tree (see
        # tests/klipper-config-load-smoke-tests.py for the suite that
        # exercises this module in that real environment instead).
        import os
        this_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(this_dir, 'nebulaos_calibration.py')) as f:
            calibration_source = f.read()
        self.assertIn("'_NEBULAOS_Z_OFFSET_CALIBRATE'", calibration_source)
        self.assertIn('def cmd_z_offset_calibrate(', calibration_source)
        self.assertFalse(hasattr(z_compensate.ZCompensate, 'cmd_z_offset_calibrate'))
        self.assertFalse(hasattr(z_compensate.ZCompensate, 'cmd_nebulaos_z_offset_calibrate'))


if __name__ == '__main__':
    unittest.main()
