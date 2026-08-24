# Phase 1.8 architecture correction: BLTouch + native load-cell coexistence proofs.
#
# Change order section 15, tests A through M. Verifies the corrected architecture where
# BLTouch remains the global Klipper probe (Z homing, bed mesh, probe:z_virtual_endstop)
# and the nozzle load cell is used ONLY for per-print Z-offset calibration via
# z_compensate.py's Z_OFFSET_CALIBRATION command.
#
# Source-inspection tests read the .py file directly to avoid importing
# nebulaos_z_offset_probe at module level — that module's top-level imports pull in upstream
# Klipper extras (hx71x, load_cell, probe, trigger_analog) which are not present in the
# extensions repo's own extras/ package. Runtime tests use FakeZOffsetProbe from
# build_environment() and never need the real module.
#
# Run from klippy/: python3 -m unittest extras.test_z_offset_probe_coexistence -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import unittest

from . import prtouch_test_support as fake
from . import prtouch_v2
from . import z_compensate


def _module_source():
    """Read nebulaos_z_offset_probe.py source from disk without importing it."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, 'nebulaos_z_offset_probe.py')
    with open(path) as f:
        return f.read()


def _module_code_lines():
    """Return only non-comment, non-blank lines of executable source."""
    return '\n'.join(
        line for line in _module_source().splitlines()
        if line.strip() and not line.strip().startswith('#'))


def _build(stub_measurement=0.0):
    printer, mcu, pins, values = fake.build_environment()
    prtouch_config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(prtouch_config)
    printer.add_object('prtouch_v2', pv2)

    zc_config = fake.make_z_compensate_config(printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
    zc = z_compensate.ZCompensate(zc_config)

    fake.connect(printer, mcu)
    prtouch_config.assert_all_consumed()
    zc_config.assert_all_consumed()

    z_offset_probe = printer.lookup_object('nebulaos_z_offset_probe')
    z_offset_probe.touch_probe = lambda down_min_z, **kw: stub_measurement
    return printer, mcu, z_offset_probe, zc


class BLTouchCoexistenceTest(unittest.TestCase):
    """Test A: BLTouch and native load-cell coexist without conflict."""

    def test_both_bltouch_and_z_offset_probe_registered(self):
        printer, _, z_probe, zc = _build()
        bltouch = printer.lookup_object('probe')
        self.assertIsNotNone(bltouch)
        self.assertIsNotNone(z_probe)
        self.assertIsNot(bltouch, z_probe)


class BLTouchRemainsGlobalProbeTest(unittest.TestCase):
    """Test B: BLTouch remains the global Klipper probe object."""

    def test_global_probe_is_bltouch_not_load_cell(self):
        printer, _, _, _ = _build()
        probe = printer.lookup_object('probe')
        self.assertIsInstance(probe, fake.FakeBLTouchProbe)

    def test_z_compensate_references_bltouch_as_probe(self):
        _, _, _, zc = _build()
        self.assertIsInstance(zc.probe, fake.FakeBLTouchProbe)


class NoDuplicateProbeObjectTest(unittest.TestCase):
    """Test C: nebulaos_z_offset_probe does NOT register as a global probe."""

    def test_module_does_not_call_add_object_probe(self):
        code = _module_code_lines()
        self.assertNotIn("add_object('probe'", code)
        self.assertNotIn('add_object("probe"', code)


class NoDuplicateProbeCommandsTest(unittest.TestCase):
    """Test D: No ProbeCommandHelper registration from the load cell module."""

    def test_module_does_not_use_probe_command_helper(self):
        code = _module_code_lines()
        self.assertNotIn('ProbeCommandHelper', code)


class ProbeVirtualEndstopNotRegisteredTest(unittest.TestCase):
    """Test E: probe:z_virtual_endstop is NOT registered by the load cell module."""

    def test_module_does_not_use_homing_via_probe_helper(self):
        code = _module_code_lines()
        self.assertNotIn('HomingViaProbeHelper', code)


class BedMeshBLTouchBackedTest(unittest.TestCase):
    """Test F: bed_mesh remains independent of the load cell module."""

    def test_z_offset_probe_does_not_reference_bed_mesh(self):
        source = _module_source()
        self.assertNotIn('bed_mesh', source)


class HX711ConfigurationTest(unittest.TestCase):
    """Test G: nebulaos_z_offset_probe configures HX711 via sensor_type."""

    def test_module_uses_hx71x_sensor_types(self):
        source = _module_source()
        self.assertIn('HX71X_SENSOR_TYPES', source)

    def test_module_imports_hx71x(self):
        source = _module_source()
        self.assertIn('from . import hx71x', source)


class OffsetCeilingEnforcedTest(unittest.TestCase):
    """Test I: max_offset_correction_mm is still enforced through the new backend."""

    def test_large_measurement_rejected(self):
        _, _, z_probe, zc = _build()
        z_probe.touch_probe = lambda down_min_z, **kw: 5.0
        gcmd = fake.FakeGCmd()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_z_offset_calibration(gcmd)
        self.assertIn("max_offset_correction_mm", str(ctx.exception))

    def test_normal_measurement_accepted(self):
        _, _, z_probe, zc = _build(stub_measurement=0.1)
        gcmd = fake.FakeGCmd()
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(zc.calibration_state, "complete")


class NoPRTouchMCUDependencyTest(unittest.TestCase):
    """Test J: nebulaos_z_offset_probe does NOT require PRTouch MCU commands."""

    def test_module_does_not_import_prtouch_mcu(self):
        source = _module_source()
        self.assertNotIn('prtouch_mcu', source)

    def test_module_does_not_import_prtouch_probe(self):
        source = _module_source()
        self.assertNotIn('prtouch_probe', source)

    def test_module_does_not_import_prtouch_v2(self):
        source = _module_source()
        self.assertNotIn('prtouch_v2', source)


class HostKlipperPristineTest(unittest.TestCase):
    """Test K: nebulaos_z_offset_probe uses only upstream Klipper imports."""

    def test_upstream_imports_only(self):
        source = _module_source()
        allowed_package = ('hx71x', 'load_cell', 'probe', 'trigger_analog')
        allowed_from = {'load_cell_probe'}
        for line in source.splitlines():
            stripped = line.strip()
            if stripped.startswith('from . import'):
                modules = [m.strip() for m in
                           stripped.split('from . import')[1].split(',')]
                for mod in modules:
                    self.assertIn(mod, allowed_package,
                                  "unexpected import: %s" % mod)
            elif stripped.startswith('from .') and ' import ' in stripped:
                mod_name = stripped.split(' import ', 1)[0] \
                    .replace('from .', '').strip()
                self.assertIn(mod_name, allowed_from,
                              "unexpected from-import: %s" % mod_name)


class ZCompensateUsesNewBackendTest(unittest.TestCase):
    """Test L: z_compensate calls nebulaos_z_offset_probe, not prtouch_v2."""

    def test_z_compensate_resolves_z_offset_probe(self):
        _, _, z_probe, zc = _build()
        self.assertIs(zc.z_offset_probe, z_probe)

    def test_prtouch_is_optional(self):
        printer, mcu, pins, values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(
            printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)
        self.assertIsNone(zc.prtouch)

    def test_calibration_calls_z_offset_probe_touch_probe(self):
        _, _, z_probe, zc = _build(stub_measurement=0.05)
        calls = []

        def tracking_touch_probe(down_min_z, **kwargs):
            calls.append({'down_min_z': down_min_z, 'kwargs': kwargs})
            return 0.05

        z_probe.touch_probe = tracking_touch_probe
        gcmd = fake.FakeGCmd()
        zc.cmd_z_offset_calibration(gcmd)
        self.assertEqual(len(calls), 1)
        self.assertEqual(zc.calibration_state, "complete")


class NoGlobalProbeRegistrationTest(unittest.TestCase):
    """Test M: the load cell module never registers as the global probe."""

    def test_load_config_does_not_return_probe_interface(self):
        code = _module_code_lines()
        self.assertNotIn("add_object('probe'", code)
        self.assertNotIn('ProbeOffsetsHelper', code)
        self.assertNotIn('start_probe_session', code)


class NozzleClearRequiresPRTouchTest(unittest.TestCase):
    """CRTENSE_NOZZLE_CLEAR fails cleanly when prtouch_v2 is not configured."""

    def test_nozzle_clear_without_prtouch_raises_clear_error(self):
        printer, mcu, pins, values = fake.build_environment()
        zc_config = fake.make_z_compensate_config(
            printer, dict(fake.REAL_Z_COMPENSATE_CONFIG))
        zc = z_compensate.ZCompensate(zc_config)
        fake.connect(printer, mcu)
        gcmd = fake.FakeGCmd()
        with self.assertRaises(fake.CommandError) as ctx:
            zc.cmd_nozzle_clear(gcmd)
        self.assertIn("prtouch_v2", str(ctx.exception))

    def test_nozzle_clear_with_prtouch_does_not_raise_prtouch_missing(self):
        printer, _, _, zc = _build()
        self.assertIsNotNone(zc.prtouch)


if __name__ == '__main__':
    unittest.main()
