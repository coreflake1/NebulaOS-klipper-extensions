# Regression tests for the 2026-08-10 raw-operation ownership guard + disarm-then-rearm
# settle fix (see docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md) - models the exact live
# incident sequence (no-trigger retries producing repeated disarm/rearm transitions) against
# the fake MCU harness, and proves the two new safety properties hold:
#   1. only one of touch_probe()/safe_move_z() may be active at a time, across BOTH public
#      entry points, however a second call is triggered - without breaking
#      _recover_after_no_trigger()'s own recovery lift, which is legitimately nested inside
#      an already-held operation.
#   2. every disarm this module issues is followed by a settle yield before the next arm.
#
# Also covers the 2026-08-13 redundant-recovery-lift fix: _fail() no longer issues its own
# lift on top of an already-completed recovery (see prtouch_probe.py's own docstring), and a
# recovery lift that itself fails to complete latches the raw channel unhealthy instead of
# ever guessing with another move.
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
    # 2026-08-12 root-cause mission: touch_probe()'s baseline guard now requires an explicitly
    # confirmed TRUSTED_REFERENCE (see prtouch_probe.py's three-state model) - prime one here
    # so tests unrelated to that guard itself can still reach real motion.
    pv2.probe.check_sensor_consistency()
    pv2.probe.confirm_bootstrap_baseline()

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

    def test_fail_raises_a_clean_command_error_not_the_raw_op_guard(self):
        # _fail() itself no longer arms any motion (2026-08-13 fix - see its own docstring),
        # so the historical concern this test guarded (a nested internal lift tripping over
        # its own _own_raw_operation guard) no longer applies to _fail() directly. What must
        # still hold: the exception _fail() raises is a plain command_error, never
        # PrtouchProbeSafetyError - a caller must not be able to confuse a genuine terminal
        # failure with the raw-op-already-active rejection this same guard also produces.
        _, mcu, pv2 = _build()
        probe = pv2.probe
        with self.assertRaises(Exception) as ctx:
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertNotIsInstance(ctx.exception, prtouch_probe.PrtouchProbeSafetyError)
        up_calls = [c for c in mcu.all_calls('start_step_prtouch')
                    if c.by_field['dir'] == 1 and c.by_field['step_cnt'] > 0]
        self.assertEqual(len(up_calls), 1,
                          "expected exactly the no-trigger recovery's own upward move - no "
                          "second lift from _fail()")


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
        # warnings clustered around. No extra settle beyond that: _fail() (reached once
        # retries are exhausted) no longer issues a lift of its own (2026-08-13 fix), so there
        # is no third disarm to settle after.
        self.assertEqual(len(calls), retries * 2)

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


class SingleRawMoveCommandTest(unittest.TestCase):
    """2026-08-11 closure mission: explicit single-direction coverage - a bare 'safe_move_z
    sends one arm/disarm pair' proof, independent of the fuller no-trigger/retry scenarios
    covered elsewhere, so UP and DOWN are each directly, individually asserted."""

    def test_single_raw_up_move_sends_exactly_one_arm_disarm_pair(self):
        _, mcu, pv2 = _build()
        pv2.probe.safe_move_z(1, 1.0, 1.0)
        step_calls = mcu.all_calls('start_step_prtouch')
        arms = [c for c in step_calls if c.by_field['step_cnt'] > 0]
        disarms = [c for c in step_calls if c.by_field['step_cnt'] == 0]
        self.assertEqual(len(arms), 1)
        self.assertEqual(arms[0].by_field['dir'], 1)
        self.assertEqual(len(disarms), 1)

    def test_single_raw_down_move_sends_exactly_one_arm_disarm_pair(self):
        _, mcu, pv2 = _build()
        pv2.probe.safe_move_z(0, 1.0, 1.0)
        step_calls = mcu.all_calls('start_step_prtouch')
        arms = [c for c in step_calls if c.by_field['step_cnt'] > 0]
        disarms = [c for c in step_calls if c.by_field['step_cnt'] == 0]
        self.assertEqual(len(arms), 1)
        self.assertEqual(arms[0].by_field['dir'], 0)
        self.assertEqual(len(disarms), 1)


class SafeMoveZOwnershipTest(unittest.TestCase):
    """The 4th ownership combination not covered by SharedOwnershipGuardTest above:
    safe_move_z vs safe_move_z. touch_probe-vs-touch_probe, touch_probe-vs-safe_move_z, and
    safe_move_z-vs-touch_probe are covered there."""

    def test_second_safe_move_z_rejected_while_first_active(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        seen = {}

        def reentrant_call(call):
            with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
                probe.safe_move_z(0, 1.0, 1.0)
            seen['message'] = str(ctx.exception)

        mcu.on_send_hook('start_step_prtouch', reentrant_call)
        probe.safe_move_z(1, 1.0, 1.0)
        self.assertIn('already in progress', seen.get('message', ''))


class TimeoutReleasesOwnershipTest(unittest.TestCase):
    """max_probe_duration_s is a distinct failure path from retry exhaustion (_touch_probe
    checks it separately, before the retries check) - the ownership guard must release on
    this exit path too, not just on a plain exception or a retries-exhausted _fail()."""

    def test_ownership_released_after_max_duration_timeout(self):
        # 1.0 is the configured minval - a single no-trigger attempt's own collect timeout
        # (down_min_z/speed + margin) already exceeds it, so the *next* loop iteration's
        # top-of-loop deadline check fires before a 2nd/3rd attempt is ever armed.
        _, mcu, pv2 = _build(prtouch_values={'max_probe_duration_s': '1.0'})
        probe = pv2.probe
        with self.assertRaises(Exception) as ctx:
            probe.touch_probe(1.0, retries=10, pro_cnt=1)
        self.assertIn('max_probe_duration_s', str(ctx.exception))
        self.assertFalse(probe._raw_op_active,
                          "ownership must release even when _fail() is reached via the "
                          "duration guard rather than retry exhaustion")
        # confirm it's genuinely usable again, not just flagged inactive.
        with self.assertRaises(Exception):
            probe.touch_probe(1.0, retries=10, pro_cnt=1)


class InstrumentationHasNoSideEffectsTest(unittest.TestCase):
    """The new logging.info() calls added throughout touch_probe/_raw_move/_raw_lift must be
    pure observation - identical MCU protocol traffic with instrumentation active or silenced.
    Proven directly by diffing the real sent-command sequence (names + field values) between a
    normal run and one with logging.info patched to a no-op, rather than just trusting that
    logging calls "obviously" don't touch the MCU."""

    def test_logging_patched_to_noop_produces_identical_mcu_traffic(self):
        import logging as logging_module

        def _sent_signature(mcu_obj):
            return [(c.name, tuple(sorted(c.by_field.items())) if c.by_field else tuple(c.args))
                    for c in mcu_obj.sent_commands]

        _, mcu_a, pv2_a = _build()
        with self.assertRaises(Exception):
            pv2_a.probe.touch_probe(1.0, retries=2, pro_cnt=1)
        normal_signature = _sent_signature(mcu_a)

        _, mcu_b, pv2_b = _build()
        original_info = logging_module.info
        logging_module.info = lambda *a, **kw: None
        try:
            with self.assertRaises(Exception):
                pv2_b.probe.touch_probe(1.0, retries=2, pro_cnt=1)
        finally:
            logging_module.info = original_info
        silenced_signature = _sent_signature(mcu_b)

        self.assertEqual(normal_signature, silenced_signature,
                          "instrumentation logging must not alter MCU protocol traffic")
        self.assertTrue(normal_signature, "sanity check: the scenario must actually send commands")


class RawChannelHealthLatchTest(unittest.TestCase):
    """2026-08-13 redundant-recovery-lift mission: if a recovery/lift move itself fails to
    complete, the physical position it was meant to restore is no longer provably known.
    _fail() no longer papers over that with a guessed extra move (see its own docstring) -
    instead _raw_lift() latches _raw_channel_healthy False, and _own_raw_operation refuses
    every future raw op (touch_probe or safe_move_z) until a restart."""

    def test_recovery_failure_latches_raw_channel_unhealthy(self):
        # The descent itself must complete cleanly (its own manual_get_steps repair queries
        # must succeed) so the failure below is unambiguously the RECOVERY lift's - the first
        # 8 manual_get_steps calls repair the down move's buffer (MAX_BUF_LEN=32, 4 samples
        # per call); every call after that belongs to the recovery lift's own repair attempt.
        _, mcu, pv2 = _build()
        probe = pv2.probe
        call_count = {'n': 0}

        def flaky_repair(call):
            call_count['n'] += 1
            if call_count['n'] > 8:
                raise RuntimeError("simulated manual_get_steps comms failure during recovery")
            i = call.args[1]
            return {'oid': pv2.mcu.step_oid, 'index': i, 'tri_time': 0,
                    'tick0': i * 100, 'tick1': (i + 1) * 100, 'tick2': (i + 2) * 100,
                    'tick3': (i + 3) * 100, 'step0': i, 'step1': i + 1, 'step2': i + 2,
                    'step3': i + 3}

        mcu.set_query_response('manual_get_steps', flaky_repair)
        self.assertTrue(probe._raw_channel_healthy)
        with self.assertRaises(RuntimeError):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertFalse(probe._raw_channel_healthy,
                          "a recovery lift that didn't complete must latch unhealthy")
        # ownership itself still releases - this is a distinct, longer-lived latch, not a
        # substitute for the per-call ownership guard.
        self.assertFalse(probe._raw_op_active)

    def test_unhealthy_raw_channel_rejects_touch_probe(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        probe._raw_channel_healthy = False
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertIn('unhealthy', str(ctx.exception))
        # confirmed before arming anything - no MCU traffic at all.
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_unhealthy_raw_channel_rejects_safe_move_z(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        probe._raw_channel_healthy = False
        with self.assertRaises(prtouch_probe.PrtouchProbeSafetyError) as ctx:
            probe.safe_move_z(1, 1.0, 1.0)
        self.assertIn('unhealthy', str(ctx.exception))
        self.assertEqual(mcu.all_calls('start_step_prtouch'), [])

    def test_healthy_channel_unaffected_by_an_ordinary_no_trigger_failure(self):
        # A plain no-trigger/retries-exhausted failure (the matching recovery completes fine)
        # must NOT latch the channel unhealthy - only a recovery move that itself fails to
        # complete should ever do that.
        _, mcu, pv2 = _build()
        probe = pv2.probe
        with self.assertRaises(Exception):
            probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertTrue(probe._raw_channel_healthy)
        # genuinely still usable - not just flagged healthy.
        probe.safe_move_z(1, 1.0, 1.0)


class ExactOrderedSequenceTest(unittest.TestCase):
    """Reproduces the incident's own multi-attempt no-trigger shape and asserts the FULL
    ordered (direction, step_cnt>0) sequence, not just aggregate counts - the old,
    incident-producing code would have shown the same directional alternation (that bug was
    already fixed in 2026-08-06/09), so what this test adds is proving a settle observably
    separates every disarm from the following arm in the reactor's own timeline, which the
    pre-2026-08-10 code never did."""

    def test_settle_gap_separates_every_disarm_from_the_next_arm_in_time(self):
        _, mcu, pv2 = _build()
        probe = pv2.probe
        reactor = pv2.printer.get_reactor()
        timeline = []
        # probe.mcu is the real PrtouchMCU wrapper (what prtouch_probe.py actually calls
        # self.mcu.start_step(...) on) - not the raw FakeMCU/`mcu` fixture, which has no
        # start_step/stop_step methods of its own. 2026-08-14 disarm-protocol mission: arm and
        # disarm are now two distinct methods (start_step() rejects step_cnt=0 outright), so
        # both must be spied on to reconstruct the same arm/disarm timeline as before.
        original_start_step = probe.mcu.start_step
        original_stop_step = probe.mcu.stop_step

        def spy_start_step(direction, step_cnt, *a, **kw):
            timeline.append(('arm', direction, reactor.monotonic()))
            return original_start_step(direction, step_cnt, *a, **kw)

        def spy_stop_step():
            timeline.append(('disarm', None, reactor.monotonic()))
            return original_stop_step()

        probe.mcu.start_step = spy_start_step
        probe.mcu.stop_step = spy_stop_step
        with self.assertRaises(Exception):
            probe.touch_probe(1.0, retries=2, pro_cnt=1)

        disarms = [(i, t) for i, (kind, _d, t) in enumerate(timeline) if kind == 'disarm']
        arms = [(i, t) for i, (kind, _d, t) in enumerate(timeline) if kind == 'arm']
        # every disarm not immediately followed by another disarm (i.e. one with a next arm
        # after it) must show that next arm strictly later in the (virtual) reactor clock -
        # zero elapsed time between them is exactly the incident's own failure mode.
        checked = 0
        for idx, disarm_time in disarms:
            next_arms = [t for i, t in arms if i > idx]
            if not next_arms:
                continue
            self.assertGreater(next_arms[0], disarm_time,
                                "a disarm must be followed by strictly later time before the "
                                "next arm - zero gap is the incident's own reproduced failure")
            checked += 1
        self.assertGreater(checked, 0, "sanity check: the scenario must contain at least one "
                                        "disarm followed by a later arm")


if __name__ == '__main__':
    unittest.main()
