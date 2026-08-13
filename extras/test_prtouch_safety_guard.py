# Proves the movement guard actually blocks every motion-capable path and leaves the
# zero-motion diagnostic path (READ_PRES/deal_avgs_prtouch) untouched, and that it always
# restores the original, unguarded methods afterward - including when the guarded block
# itself raises.
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_safety_guard -v (this fork's own layout - klippy/extras/
# is a real Python package named 'extras', not 'klippy_extras' - see NebulaOS-firmware's
# klippy_extras/ mirror of this same file for that repo's own invocation form)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import prtouch_safety_guard as guard_mod
from . import prtouch_v2


def _build():
    printer, mcu, pins, values = fake.build_environment()
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    mcu.set_query_response('deal_avgs_prtouch',
                            {'oid': pv2.mcu.pres_oid, 'ch0': -251471, 'ch1': 0, 'ch2': 0, 'ch3': 0})
    # 2026-08-12 root-cause mission: touch_probe()'s baseline guard now requires an explicitly
    # confirmed TRUSTED_REFERENCE (see prtouch_probe.py's three-state model) - prime one here
    # so tests unrelated to that guard itself can still reach real motion.
    pv2.probe.check_sensor_consistency()
    pv2.probe.confirm_bootstrap_baseline()
    return printer, mcu, pv2


class BlocksRealMotionTest(unittest.TestCase):
    def test_blocks_nonzero_start_step_prtouch(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            with self.assertRaises(guard_mod.MovementBlockedError):
                pv2.mcu.start_step(0, 100, 2000, 4)

    def test_allows_zero_step_cnt_stop_idiom(self):
        # step_cnt=0 is the documented "stop/disarm" call (PrtouchMCU.stop_step(), and every
        # real raw-op disarm site in prtouch_probe.py) - it moves nothing and must stay usable
        # even under the guard, or cleanup code itself would be unable to run. 2026-08-14
        # disarm-protocol mission: start_step() itself now rejects step_cnt=0 outright (it is
        # not a valid disarm on the real wire protocol without send_ms=0 too - see
        # prtouch_mcu.py's start_step()/stop_step() docstrings), so this guard-level test now
        # exercises stop_step() directly, same as every real caller does.
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            pv2.mcu.stop_step()  # must not raise
        self.assertTrue(mcu.last_call('start_step_prtouch'))

    def test_blocks_touch_probe_end_to_end(self):
        # the real orchestration path (touch_probe -> get_step_counts -> mcu.start_step)
        # must be blocked at its first real motion attempt, not just the raw mcu call.
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            with self.assertRaises(guard_mod.MovementBlockedError):
                pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)

    def test_blocks_safe_move_z(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            with self.assertRaises(guard_mod.MovementBlockedError):
                pv2.probe.safe_move_z(1, 5.0, 10.0)


class BlocksMovementGcodeTest(unittest.TestCase):
    def test_blocks_g28_g29_bed_mesh_save_config(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            for script in ('G28', 'G29', 'G1 X10 Y10 Z5 F3000', 'BED_MESH_CALIBRATE',
                           'SAVE_CONFIG', 'Z_OFFSET_APPLY_PROBE', 'Z_OFFSET_CALIBRATION',
                           'NOZZLE_CLEAR', 'CRTENSE_NOZZLE_CLEAR', 'SAFE_MOVE_Z DIR=1'):
                with self.assertRaises(guard_mod.MovementBlockedError, msg=script):
                    pv2.gcode.run_script_from_command(script)

    def test_blocks_unrecognized_command_fail_closed(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            with self.assertRaises(guard_mod.MovementBlockedError):
                pv2.gcode.run_script_from_command('SOME_FUTURE_MACRO_NOT_YET_CLASSIFIED')

    def test_allows_read_pres(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            pv2.gcode.run_script_from_command('READ_PRES')  # must not raise


class ZeroMotionDiagnosticsRemainUsableTest(unittest.TestCase):
    def test_deal_avgs_prtouch_unaffected_by_guard(self):
        # a genuinely different query-command object from start_step_prtouch_cmd - never
        # touched by the guard at all, proven by exercising the real READ_PRES command.
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            gcmd = fake.FakeGCmd()
            pv2.cmd_READ_PRES(gcmd)
        self.assertIn('ch0=-251471', gcmd.responses[0])


class RestoresOriginalMethodsTest(unittest.TestCase):
    def test_restored_after_normal_exit(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            pass
        pv2.mcu.start_step(0, 50, 1000, 2)  # must not raise post-guard
        self.assertTrue(mcu.last_call('start_step_prtouch').by_field['step_cnt'] == 50)

    def test_restored_even_when_guarded_block_raises(self):
        _, mcu, pv2 = _build()
        try:
            with guard_mod.guard(pv2):
                pv2.mcu.start_step(0, 100, 2000, 4)  # raises MovementBlockedError
        except guard_mod.MovementBlockedError:
            pass
        # guard must be gone now, even though the with-block exited via exception
        pv2.mcu.start_step(0, 50, 1000, 2)  # must not raise post-guard
        self.assertTrue(mcu.last_call('start_step_prtouch').by_field['step_cnt'] == 50)

    def test_restored_gcode_after_block(self):
        _, mcu, pv2 = _build()
        with guard_mod.guard(pv2):
            pass
        pv2.gcode.run_script_from_command('G28')  # must not raise post-guard
        self.assertIn('G28', pv2.gcode.scripts_run)


if __name__ == '__main__':
    unittest.main()
