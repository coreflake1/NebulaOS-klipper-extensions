# Configuration parity tests - instantiates the REAL PRTouchV2/ZCompensate classes against
# fixtures copied verbatim from this printer's own live [prtouch_v2]/[z_compensate]
# sections (pulled via SSH, see printer.cfg and prtouch_test_support.REAL_*_CONFIG), and
# fails loud if any real stock value would be rejected, silently defaulted over, or read
# from the wrong section - the exact three failure classes already found live on
# 2026-08-05/06 (clr_noz_start_x's minval, wrong-section reads, deferred reads).
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_config -v (this fork's own layout - klippy/extras/
# is a real Python package named 'extras', not 'klippy_extras' - see NebulaOS-firmware's
# klippy_extras/ mirror of this same file for that repo's own invocation form)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import prtouch_v2
from . import z_compensate


def _build_full_environment(prtouch_overrides=None, zcompensate_overrides=None):
    """Mirrors real printer.cfg load order: [prtouch_v2] then [z_compensate], both
    connected via the same klippy:connect event, exactly like klippy.py's real _read_config
    + _connect sequence."""
    printer, mcu, pins, prtouch_values = fake.build_environment(prtouch_overrides)
    prtouch_config = fake.make_prtouch_v2_config(printer, pins, prtouch_values)
    pv2 = prtouch_v2.PRTouchV2(prtouch_config)
    printer.add_object('prtouch_v2', pv2)

    zc_values = dict(fake.REAL_Z_COMPENSATE_CONFIG)
    if zcompensate_overrides:
        zc_values.update(zcompensate_overrides)
    zc_config = fake.make_z_compensate_config(printer, zc_values)
    zc = z_compensate.ZCompensate(zc_config)

    fake.connect(printer, mcu)
    prtouch_config.assert_all_consumed()
    zc_config.assert_all_consumed()
    return printer, mcu, pv2, zc


class RealFixtureLoadsCleanTest(unittest.TestCase):
    """The headline live failure (2026-08-06): pasting the real config section in would
    make Klipper refuse to start. This is the offline equivalent of that live restart
    test - if this test ever fails, a real restart against this exact fixture would too."""

    def test_real_prtouch_v2_and_z_compensate_sections_load_without_error(self):
        printer, mcu, pv2, zc = _build_full_environment()
        self.assertIsNotNone(pv2)
        self.assertIsNotNone(zc)

    def test_every_gcode_command_registers(self):
        printer, mcu, pv2, zc = _build_full_environment()
        gcode = printer.objects['gcode']
        for name in ('NOZZLE_CLEAR', 'SAFE_MOVE_Z', 'READ_PRES',
                     'CRTENSE_NOZZLE_CLEAR', 'Z_OFFSET_CALIBRATION'):
            self.assertIn(name, gcode.commands, "%s must be registered" % name)


class RealValuesParsedCorrectlyTest(unittest.TestCase):
    """Per-key checks: the real literal from printer.cfg must come back out unchanged, not
    silently replaced by a fallback default - this is the "default overrides a supplied
    stock value" failure class from the task brief."""

    def setUp(self):
        self.printer, self.mcu, self.pv2, self.zc = _build_full_environment()

    def test_prtouch_v2_speed_key_drives_tri_z_down_spd(self):
        # real [prtouch_v2] speed: 1 - was completely unconsumed before the 2026-08-05 fix.
        self.assertEqual(self.pv2.probe.tri_z_down_spd, 1.0)

    def test_prtouch_v2_tri_min_max_hold(self):
        self.assertEqual(self.pv2.probe.tri_min_hold, 1000)
        self.assertEqual(self.pv2.probe.tri_max_hold, 1500)

    def test_z_compensate_tri_min_max_hold_distinct_from_prtouch_v2(self):
        # separately-tuned per real config (1400/2000 vs prtouch_v2's 1000/1500) - not a
        # duplicate, not defaulted to prtouch_v2's own values.
        self.assertEqual(self.zc.tri_min_hold, 1400)
        self.assertEqual(self.zc.tri_max_hold, 2000)
        self.assertNotEqual(self.zc.tri_min_hold, self.pv2.probe.tri_min_hold)

    def test_z_compensate_speed_distinct_from_prtouch_v2_speed(self):
        self.assertEqual(self.zc.probe_speed, 5.0)
        self.assertNotEqual(self.zc.probe_speed, self.pv2.probe.tri_z_down_spd)

    def test_tri_expand_mm_is_this_printers_live_tuned_value_not_factory(self):
        # 0.10 (live-tuned), not factory's 0.13 - see printer.cfg's own inline comment.
        self.assertAlmostEqual(self.zc.tri_expand_mm, 0.10)

    def test_bl_offset_matches_bltouch_y_offset(self):
        # real evidence tying Z_OFFSET_CALIBRATION's target point to BLTouch's own
        # calibrated probe location - see z_compensate.py's own module docstring.
        self.assertEqual(self.zc.bl_offset_x, 0.0)
        self.assertEqual(self.zc.bl_offset_y, fake.REAL_BLTOUCH_Y_OFFSET)

    def test_clr_noz_start_x_accepts_the_real_negative_value(self):
        # real value is -3 - a stricter minval=0 (the pre-2026-08-05 bound) would reject
        # this outright the moment it was actually read.
        self.assertEqual(self.zc.clear_nozzle_config.clr_noz_start_x, -3.0)

    def test_wipe_pad_is_y_oriented_not_x(self):
        # pa_clr_dis_mm_x: 0, pa_clr_dis_mm_y: 30 - confirms the real wipe geometry read
        # correctly (see prtouch_nozzle.py's 2D-drag-vector generalization).
        self.assertEqual(self.zc.clear_nozzle_config.pa_clr_dis_mm_x, 0.0)
        self.assertEqual(self.zc.clear_nozzle_config.pa_clr_dis_mm_y, 30.0)

    def test_hot_end_temp_present_and_distinct_from_hot_start_and_rub(self):
        self.assertEqual(self.zc.hot_end_temp, 140.0)
        self.assertEqual(self.zc.hot_start_temp, 180.0)
        self.assertEqual(self.zc.hot_rub_temp, 200.0)

    def test_vs_start_z_pos_drives_hover_height_not_the_old_default(self):
        # real value 3, not the old Klipper-only z_offset_hover_height default of 5.
        self.assertEqual(self.zc.hover_height, 3.0)

    def test_pr_probe_cnt_and_pr_clear_probe_cnt_are_distinct_reads(self):
        self.assertEqual(self.zc.pr_probe_cnt, 3)
        self.assertEqual(self.zc.clear_nozzle_config.pr_clear_probe_cnt, 3)


class InertKeysAreReadButDocumentedUnwiredTest(unittest.TestCase):
    """type_nozz/noz_pos_center/noz_pos_offset/pumpback_mm: no reference exists for these
    anywhere (see z_compensate.py's own comment) - this test proves they're at least
    ACCEPTED (real Klipper would reject the whole section otherwise), and pins down that
    they are stored as plain read values, never silently promoted into wired behavior
    without a corresponding code change (which would need this test updated deliberately,
    not accidentally)."""

    def setUp(self):
        self.printer, self.mcu, self.pv2, self.zc = _build_full_environment()

    def test_inert_keys_are_stored_verbatim(self):
        self.assertEqual(self.zc.type_nozz, 0)
        self.assertEqual(self.zc.noz_pos_center, (20.0, 25.0))
        self.assertEqual(self.zc.noz_pos_offset, (3.0, 7.0))
        self.assertEqual(self.zc.pumpback_mm, 10.0)


class BoundsRegressionTest(unittest.TestCase):
    """Bound-by-bound regression coverage: a real value sitting exactly at, or just past,
    a bound that has been tightened in the past must still be accepted (or a genuinely
    invalid value must still be rejected) - guards against either direction of mistake."""

    def test_clr_noz_start_x_minus_three_is_accepted(self):
        printer, mcu, pins, values = fake.build_environment()
        config = fake.make_prtouch_v2_config(printer, pins, values)
        pv2 = prtouch_v2.PRTouchV2(config)
        printer.add_object('prtouch_v2', pv2)
        zc_values = dict(fake.REAL_Z_COMPENSATE_CONFIG)
        zc_values['clr_noz_start_x'] = '-3'
        zc_config = fake.make_z_compensate_config(printer, zc_values)
        z_compensate.ZCompensate(zc_config)  # must not raise

    def test_clr_noz_start_x_below_minus_fifty_is_still_rejected(self):
        # sanity check the bound is real, not accidentally removed entirely.
        printer, mcu, pins, values = fake.build_environment()
        prtouch_config = fake.make_prtouch_v2_config(printer, pins, values)
        pv2 = prtouch_v2.PRTouchV2(prtouch_config)
        printer.add_object('prtouch_v2', pv2)
        zc_values = dict(fake.REAL_Z_COMPENSATE_CONFIG)
        zc_values['clr_noz_start_x'] = '-999'
        zc_config = fake.make_z_compensate_config(printer, zc_values)
        with self.assertRaises(fake.ConfigError):
            z_compensate.ZCompensate(zc_config)

    def test_bed_add_temp_sixty_is_accepted(self):
        # real value 60 - an earlier maxval=20 would have rejected this.
        printer, mcu, pv2, zc = _build_full_environment()
        self.assertEqual(zc.bed_add_temp, 60.0)


class UnusedOptionIsCaughtTest(unittest.TestCase):
    """Proves assert_all_consumed() itself actually catches the real failure mode - a
    meta-test for the test harness: inject one genuinely unread key and confirm it's
    flagged, so a silent regression in FakeConfig's own tracking can't hide a real one."""

    def test_deliberately_unread_key_is_flagged(self):
        printer, mcu, pins, values = fake.build_environment()
        values['this_key_is_never_read'] = '1'
        config = fake.make_prtouch_v2_config(printer, pins, values)
        prtouch_v2.PRTouchV2(config)
        with self.assertRaises(fake.ConfigError):
            config.assert_all_consumed()


if __name__ == '__main__':
    unittest.main()
