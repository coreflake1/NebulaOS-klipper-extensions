# Regression tests for the 2026-08-10 raw-operation ownership guard + disarm-then-rearm
# settle fix (see docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md) - models the exact live
# incident sequence (no-trigger retries producing repeated disarm/rearm transitions) against
# the fake MCU harness, and proves the two new safety properties hold:
#   1. only one of touch_probe()/safe_move_z() may be active at a time, across BOTH public
#      entry points, however a second call is triggered - without breaking _fail()'s own
#      internal safety lift, which is legitimately nested inside an already-held operation.
#   2. every disarm this module issues is followed by a settle yield before the next arm.
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_raw_op_guard -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_probe
from . import prtouch_test_support as fake
from . import prtouch_v2


def _build(prtouch_values=None):
    printer, mcu, pins, values = fake.build_environment(prtouch_values)
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    mcu.set_query_response('deal_avgs_prtouch',
                            {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0})

    def _step_repair(call):
        i = call.args[1]
        return {'oid': pv2.mcu.step_oid, 'index': i, 'tri_time': 0,
                'tick0': i * 100, 'tick1': (i + 1) * 100, 'tick2': (i + 2) * 100,
                'tick3': (i + 3) * 100, 'step0': i, 'step1': i + 1, 'step2': i + 2,
                'step3': i + 3}

    def _pres_repair(call):
        i = call.args[1]
        return {'oid': pv2.mcu.pres_oid, 'index': i, 'tri_time': 0, 'tri_chs': 0, 'buf_cnt': 32,
                'tick_0': i * 100, 'ch0_0': 0, 'ch1_0': 0, 'ch2_0': 0, 'ch3_0': 0,
                'tick_1': (i + 1) * 100, 'ch0_1': 0, 'ch1_1': 0, 'ch2_1': 0, 'ch3_1': 0}

    mcu.set_query_response('manual_get_steps', _step_repair)
    mcu.set_query_response('manual_get_pres', _pres_repair)
    return printer, mcu, pv2


class SharedOwnershipGuardTest(unittest.TestCase):
    """Section 9: only one raw PRTouch operation (touch_probe or safe_move_z) may exist at a
    time, across both public entry points, with no queueing - the guard the live incident's
    own unexplained second Z_OFFSET_CALIBRATION request (proven not to be the actual cause,
    but closed regardless) motivated."""

    def test_second_touch_probe_call_rejected_while_first_active(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        seen = {}

        def reentrant_call(call):
            with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
                probe.touch_probe(0.5, retries=1, pro_cnt=1)
            seen['message'] = str(ctx.exception)

        mcu.on_send_hook('start_step_prtouch', reentrant_call)
        with self.assertRaises(Exception):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertIn('already in progress', seen.get('message', ''))

    def test_safe_move_z_rejected_while_touch_probe_active(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        seen = {}

        def reentrant_call(call):
            with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
                probe.safe_move_z(1, 1.0, 1.0)
            seen['message'] = str(ctx.exception)

        mcu.on_send_hook('start_step_prtouch', reentrant_call)
        with self.assertRaises(Exception):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertIn('already in progress', seen.get('message', ''))

    def test_touch_probe_rejected_while_safe_move_z_active(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        seen = {}

        def reentrant_call(call):
            with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
                probe.touch_probe(0.5, retries=1, pro_cnt=1)
            seen['message'] = str(ctx.exception)

        mcu.on_send_hook('start_step_prtouch', reentrant_call)
        probe.safe_move_z(1, 1.0, 1.0)
        self.assertIn('already in progress', seen.get('message', ''))

    def test_guard_releases_after_success_allowing_next_call(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        probe.safe_move_z(1, 1.0, 1.0)
        self.assertFalse(probe._raw_op_active)
        # must not be rejected - the guard only blocks genuine overlap, not sequential reuse.
        probe.safe_move_z(1, 1.0, 1.0)

    def test_guard_releases_after_exception_allowing_next_call(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        with self.assertRaises(Exception):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertFalse(probe._raw_op_active)
        with self.assertRaises(Exception):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)

    def test_fail_internal_safety_lift_not_blocked_by_its_own_guard(self):
        # Regression proof for the _raw_move refactor: _fail()'s own internal safety lift
        # must still actually run (and arm an upward move) even though it executes while
        # touch_probe()'s own guard is held - it must never raise PrtouchProbeSafetyError
        # against itself, which would silently swallow the real command_error and skip the
        # safety lift entirely.
        _, mcu, pv2 = _build()
        probe = pv2.probe
        with self.assertRaises(Exception) as ctx:
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertNotIsInstance(ctx.exception, prtouch_probe.PrtouchProbeSafetyError)
        up_calls = [c for c in mcu.all_calls('start_step_prtouch')
                    if c.by_field['dir'] == 1 and c.by_field['step_cnt'] > 0]
        self.assertTrue(up_calls,
                         "expected _fail()'s own safety lift to have armed an upward move")


class SettleAfterDisarmTest(unittest.TestCase):
    """Section 7: every disarm-then-rearm transition observed in the live incident preceded a
    'Timer too close' MCU warning. _settle_after_disarm() must run after every single disarm
    this module issues, before the next arm."""

    def test_settle_called_once_per_disarm_in_no_trigger_retry_sequence(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        calls = []
        original = probe._settle_after_disarm

        def spy():
            calls.append(pv2.printer.get_reactor().monotonic())
            original()

        probe._settle_after_disarm = spy
        retries = 3
        with self.assertRaises(Exception):
            probe.touch_probe(1.0, retries=retries, pro_cnt=1)
        # Per attempt: one settle after the down-arm's disarm, one after the recovery lift's
        # own disarm - exactly the two transitions the live incident's "Timer too close"
        # warnings clustered around - plus one more for _fail()'s own final safety-lift disarm.
        self.assertEqual(len(calls), retries * 2 + 1)

    def test_settle_duration_defaults_to_tri_send_ms(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        self.assertAlmostEqual(probe._raw_op_settle_s, probe.tri_send_ms / 1000.0)

    def test_settle_duration_overridable(self):
        _, mcu, pv2 = _build(prtouch_values={'raw_op_settle_s': '0.25'})
        probe = pv2.probe
        self.assertAlmostEqual(probe._raw_op_settle_s, 0.25)

    def test_settle_actually_advances_the_reactor_clock(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        reactor = pv2.printer.get_reactor()
        before = reactor.monotonic()
        probe.safe_move_z(1, 1.0, 1.0)
        after = reactor.monotonic()
        self.assertGreaterEqual(after - before, probe._raw_op_settle_s)


if __name__ == '__main__':
    unittest.main()
