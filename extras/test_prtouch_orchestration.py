# Runtime state-machine tests for PrtouchProbe.touch_probe() - success, no-trigger, retry,
# malformed/partial buffers, stale responses, and exception/cleanup paths - driven entirely
# through the real production PrtouchProbe/PrtouchMCU classes against the fake MCU harness.
# No physical motion, no real time elapsed (FakeReactor's clock is a plain float).
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_orchestration -v (this fork's own layout - klippy/extras/
# is a real Python package named 'extras', not 'klippy_extras' - see NebulaOS-firmware's
# klippy_extras/ mirror of this same file for that repo's own invocation form)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_mcu
from . import prtouch_test_support as fake
from . import prtouch_v2
from . import prtouch_probe


MM_PER_STEP = 0.01  # step_base(2) * FakeStepper default step_dist(0.005)


def _build(prtouch_values=None):
    printer, mcu, pins, values = fake.build_environment(prtouch_values)
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    # deal_avgs_prtouch is called once per attempt purely as a baseline sensor read (its
    # result isn't used by touch_probe itself, matching production - see prtouch_probe.py's
    # _touch_probe) - every scenario needs a response for it regardless of what's under test.
    mcu.set_query_response('deal_avgs_prtouch',
                            {'oid': pv2.mcu.pres_oid, 'ch0': -250000, 'ch1': 0, 'ch2': 0, 'ch3': 0})
    # 2026-08-12 root-cause mission: touch_probe()'s baseline guard now requires an explicitly
    # confirmed TRUSTED_REFERENCE (see prtouch_probe.py's three-state model) - prime one here,
    # against the default response above, before any test-specific override is applied, so
    # tests unrelated to that guard itself can still reach real motion.
    pv2.probe.check_sensor_consistency()
    pv2.probe.confirm_bootstrap_baseline()
    # manual_get_steps/manual_get_pres back the _repair_*_samples path any time a collect_*
    # poll times out without a full 32-sample buffer (an "up" recovery/lift move never gets
    # an async response scripted in most scenarios below, since the test is exercising the
    # *down* attempt's behavior) - generic zero-filled repair data unless a test overrides it.
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


def _full_step_trace(step_cnt):
    """32-sample step trace (tick, step) pairs, ramping linearly from step_cnt down to 0
    across the buffer - the step side keeps reporting its own position regardless of where the
    pressure side's trigger dip lands (they're two independently-sampled channels, tied
    together only by shared tick units - see interpolate_trigger_step).

    2026-08-09 (load-cell safety hardening mission): direction corrected. 'step' is the
    firmware's own REMAINING-pulse countdown, not a traveled-so-far count - confirmed directly
    from reference/prtouch_v2.c: step_cfg.now_steps is initialized to the commanded total
    (fix_steps) and decrements (now_steps--) every pulse, and it's now_steps/2 that gets
    pushed to the FIFO the host reads back as 'step'. An earlier version of this fixture ramped
    the wrong way (0 up to step_cnt) - self-consistent against this test file's own circular
    assertions, so it never failed, but did not model what the real firmware actually sends.
    See prtouch_probe.py's _lift_after_down docstring for the same finding applied to
    production code (which was already correct - only this fixture had the direction backward).
    """
    return [(i * 100, int(step_cnt * (31 - i) / 31)) for i in range(32)]


def _full_pres_trace(dip_at=None, baseline=0.0, dip=-500.0):
    trace = []
    for i in range(32):
        ch0 = dip if (dip_at is not None and i == dip_at) else baseline
        trace.append((i * 100, ch0, 0, 0, 0))
    return trace


def _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200):
    """Registers on_send hooks so that the NEXT start_pres/start_step pair triggers a full,
    successful 32-sample response pair with a pressure dip at `dip_at`, delivered
    synchronously (as soon as both arms have been sent) - models a well-behaved MCU."""
    state = {'armed_step': False, 'armed_pres': False}

    def maybe_fire():
        if state['armed_step'] and state['armed_pres']:
            for chunk in fake.make_step_result(
                    pv2.mcu.step_oid, 0, _full_step_trace(step_cnt_hint)):
                mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, chunk)
            for chunk in fake.make_pres_result(
                    pv2.mcu.pres_oid, 0, 0x1, 32, _full_pres_trace(dip_at=dip_at)):
                mcu.push_response('result_run_pres_prtouch', pv2.mcu.pres_oid, chunk)
            state['armed_step'] = state['armed_pres'] = False

    def on_step(call):
        if call.by_field and call.by_field.get('step_cnt', 0) > 0:
            state['armed_step'] = True
            maybe_fire()

    def on_pres(call):
        if call.by_field and call.by_field.get('acq_ms', 0) > 0:
            state['armed_pres'] = True
            maybe_fire()

    mcu.on_send_hook('start_step_prtouch', on_step)
    mcu.on_send_hook('start_pres_prtouch', on_pres)


class SuccessfulTriggerTest(unittest.TestCase):
    def test_single_attempt_success_with_agreeing_second_sample(self):
        _, mcu, pv2 = _build()
        _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200)
        z = pv2.probe.touch_probe(2.0, retries=3, pro_cnt=2, tolerance=1000.0)
        self.assertIsInstance(z, float)
        # both downward probe arms happened (one per successful attempt) - excludes the
        # lift-back-up arm each successful attempt also sends (dir=1).
        down_arms = [c for c in mcu.all_calls('start_step_prtouch')
                     if c.by_field['step_cnt'] > 0 and c.by_field['dir'] == 0]
        self.assertEqual(len(down_arms), 2)  # pro_cnt=2, both attempts succeed on first try
        # every arm (down or up) was followed by a stop (all-zero) call somewhere
        stop_calls = [c for c in mcu.all_calls('start_step_prtouch') if c.by_field['step_cnt'] == 0]
        self.assertTrue(len(stop_calls) >= 2)

    def test_mesh_suspended_during_probe_and_restored_after(self):
        printer, mcu, pv2 = _build()
        bed_mesh = printer.objects['bed_mesh']
        bed_mesh.set_mesh('a-real-mesh-object')  # test setup - this call is also recorded
        setup_calls = len(bed_mesh.set_mesh_calls)
        _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200)
        pv2.probe.touch_probe(2.0, retries=3, pro_cnt=2, tolerance=1000.0)
        # set_mesh(None) then set_mesh(the original) - suspend, then restore - the first
        # touch_probe()-internal call (right after setup) must be the suspend to None.
        self.assertEqual(bed_mesh.set_mesh_calls[setup_calls], None)
        self.assertEqual(bed_mesh.set_mesh_calls[-1], 'a-real-mesh-object')

    def test_trigger_z_uses_expected_arrays(self):
        # cross-check against prtouch_calibration directly with the same inputs the
        # orchestration layer actually collected, proving the handoff is faithful.
        from . import prtouch_calibration as cal
        _, mcu, pv2 = _build()
        down_min_z = 2.0
        step_cnt = int(down_min_z / MM_PER_STEP)  # matches PrtouchProbe.get_step_counts exactly
        _arm_trigger_response(mcu, pv2, dip_at=16, step_cnt_hint=step_cnt)
        z = pv2.probe.touch_probe(down_min_z, retries=1, pro_cnt=1)
        step_samples = fake.make_step_result(0, 0, _full_step_trace(step_cnt))
        pres_samples = fake.make_pres_result(0, 0, 0x1, 32, _full_pres_trace(dip_at=16))
        # flatten the chunked wire format back into the same shape compute_trigger_z expects
        steps = []
        for c in step_samples:
            for j in range(4):
                steps.append({'tick': c['tick%d' % j] / 10000., 'step': c['step%d' % j]})
        press = []
        for c in pres_samples:
            for j in range(2):
                press.append({'tick': c['tick_%d' % j] / 10000., 'ch0': c['ch0_%d' % j],
                               'ch1': 0, 'ch2': 0, 'ch3': 0})
        expected = cal.compute_trigger_z(
            steps, press, 0.0, 0.0, 0x1, step_cnt, 0.0, MM_PER_STEP,
            pv2.mcu.use_adc, pv2.probe.tri_acq_ms, pv2.probe.cal_hftr_cut,
            pv2.probe.cal_lftr_k1, pres_cnt=pv2.mcu.pres_cnt)
        self.assertAlmostEqual(z, expected, places=6)


class StockFidelityOrderingTest(unittest.TestCase):
    """2026-08-12 stock-vs-NebulaOS behavioral fidelity mission: locks in, as a regression, the
    one finding that mattered from a full line-by-line comparison against the real stock
    reference/prtouch_v2_wrapper.py (run_step_prtouch, lines 1157-1307 of that file). Stock
    always arms start_pres_prtouch BEFORE start_step_prtouch for a given attempt, and always
    disarms start_step_prtouch BEFORE start_pres_prtouch - this exact ordering was independently
    already present in _touch_probe() before this mission started, which is why no production
    behavior change came out of the mission. If this ordering ever regresses, it silently
    reopens the question this whole investigation spent multiple sessions closing."""

    def test_pres_armed_before_step_armed_each_attempt(self):
        _, mcu, pv2 = _build()
        _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200)
        pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        arm_names = [c.name for c in mcu.sent_commands
                     if c.name in ('start_pres_prtouch', 'start_step_prtouch')
                     and c.by_field and c.by_field.get(
                         'acq_ms' if c.name == 'start_pres_prtouch' else 'step_cnt', 0) > 0]
        self.assertEqual(arm_names[0], 'start_pres_prtouch')
        self.assertEqual(arm_names[1], 'start_step_prtouch')

    def test_step_disarmed_before_pres_disarmed_each_attempt(self):
        _, mcu, pv2 = _build()
        _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200)
        pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        # the very first disarm pair after the down-attempt's arms - a stop is step_cnt==0 /
        # acq_ms==0 - excludes the later lift-back-up arm/disarm, which is step-only (no pres
        # channel involved at all - matches stock's own recovery lift, also step-only).
        disarm_names = [c.name for c in mcu.sent_commands
                         if c.name in ('start_pres_prtouch', 'start_step_prtouch')
                         and c.by_field and c.by_field.get(
                             'acq_ms' if c.name == 'start_pres_prtouch' else 'step_cnt', 0) == 0]
        self.assertEqual(disarm_names[0], 'start_step_prtouch')
        self.assertEqual(disarm_names[1], 'start_pres_prtouch')


class NoTriggerTest(unittest.TestCase):
    def test_no_trigger_exhausts_retries_and_fails_loud(self):
        _, mcu, pv2 = _build()
        # never arm a response - every attempt times out with empty buffers
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(0.5, retries=2, pro_cnt=1)

    def test_no_trigger_never_fabricates_a_z_result(self):
        _, mcu, pv2 = _build()
        try:
            pv2.probe.touch_probe(0.5, retries=1, pro_cnt=1)
            self.fail("expected command_error on exhausted no-trigger retries")
        except Exception as e:
            self.assertIn("did not converge", str(e))

    def test_no_trigger_lifts_toolhead_back_before_each_retry(self):
        # regression test for the 2026-08-06 fix: a no-trigger attempt must issue a
        # compensating upward step move (using the known COMMANDED distance, since
        # step_samples is empty) before the next attempt's start_step_prtouch is armed -
        # otherwise each retry stacks another full blind descent on top of the last.
        _, mcu, pv2 = _build()
        try:
            pv2.probe.touch_probe(1.0, retries=3, pro_cnt=1)
        except Exception:
            pass
        step_calls = mcu.all_calls('start_step_prtouch')
        # sequence per attempt: arm-down, stop(0), recover-lift-up, stop(0) - so directions
        # must alternate 0 (down), -, 1 (up), - for every attempt, never two downs in a row
        # with no intervening up.
        directions_of_nonzero_arms = [c.by_field['dir'] for c in step_calls
                                       if c.by_field['step_cnt'] > 0]
        self.assertIn(1, directions_of_nonzero_arms,
                      "expected at least one upward (dir=1) recovery move after a no-trigger")
        # no two consecutive non-zero arms in the same (downward) direction without an
        # intervening upward arm - this is exactly the cumulative-descent failure mode.
        for i in range(len(directions_of_nonzero_arms) - 1):
            if directions_of_nonzero_arms[i] == 0:
                self.assertEqual(
                    directions_of_nonzero_arms[i + 1], 1,
                    "a downward arm must always be immediately followed by an upward "
                    "recovery arm, never by another downward arm - repeated blind descents "
                    "with no compensating lift is the exact bug this test guards against")

    def test_recovery_lift_uses_full_commanded_distance_not_zero(self):
        _, mcu, pv2 = _build()
        try:
            pv2.probe.touch_probe(3.0, retries=1, pro_cnt=1)
        except Exception:
            pass
        up_calls = [c for c in mcu.all_calls('start_step_prtouch')
                    if c.by_field['dir'] == 1 and c.by_field['step_cnt'] > 0]
        self.assertTrue(up_calls, "expected a nonzero upward recovery move")
        # 3.0mm commanded at MM_PER_STEP=0.01 -> 300 steps commanded; recovery must lift
        # (approximately) that many steps back, not some other/zero amount.
        self.assertGreater(up_calls[0].by_field['step_cnt'], 250)


class TimeoutTest(unittest.TestCase):
    def test_timeout_uses_down_min_z_over_speed_plus_margin(self):
        _, mcu, pv2 = _build()
        reactor = pv2.printer.get_reactor()
        start = reactor.monotonic()
        try:
            pv2.probe.touch_probe(1.0, retries=1, pro_cnt=1)
        except Exception:
            pass
        elapsed = reactor.monotonic() - start
        # down_min_z / tri_z_down_spd + 2.0, x2 (down attempt + the recovery lift's own
        # timeout), loose bound - proves the FakeReactor clock genuinely advanced via the
        # real polling code, not skipped/short-circuited.
        expected_probe_timeout = 1.0 / pv2.probe.tri_z_down_spd + 2.0
        self.assertGreaterEqual(elapsed, expected_probe_timeout)

    def test_late_callback_after_timeout_cannot_resurrect_a_failed_attempt(self):
        # a response that arrives after collect_*_samples has already given up (timed out)
        # must not silently get treated as if it belonged to a fresh, still-in-progress
        # attempt - the buffers were freshly reset_buffers()'d before the NEXT attempt starts,
        # so a late push from attempt N landing during attempt N+1's window would
        # contaminate it. This proves reset_buffers() truly empties the list each attempt.
        _, mcu, pv2 = _build()
        pv2.mcu.step_res.append({'tick': 999., 'step': 999, 'index': 0})  # stale leftover
        pv2.mcu.reset_buffers()
        self.assertEqual(pv2.mcu.step_res, [])
        self.assertEqual(pv2.mcu.pres_res, [])


class RetryIsolationTest(unittest.TestCase):
    def test_first_attempt_fails_second_succeeds_result_is_pure(self):
        _, mcu, pv2 = _build()
        attempts = {'n': 0}

        def on_step(call):
            if call.by_field and call.by_field.get('step_cnt', 0) > 0 and call.by_field['dir'] == 0:
                attempts['n'] += 1
                if attempts['n'] == 2:  # succeed only on the 2nd downward arm
                    for chunk in fake.make_step_result(
                            pv2.mcu.step_oid, 0, _full_step_trace(200)):
                        mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, chunk)
                    for chunk in fake.make_pres_result(
                            pv2.mcu.pres_oid, 0, 0x1, 32, _full_pres_trace(dip_at=16)):
                        mcu.push_response('result_run_pres_prtouch', pv2.mcu.pres_oid, chunk)

        mcu.on_send_hook('start_step_prtouch', on_step)
        z = pv2.probe.touch_probe(2.0, retries=3, pro_cnt=1)
        self.assertIsInstance(z, float)
        self.assertEqual(attempts['n'], 2, "expected exactly one failed + one successful attempt")

    def test_retry_count_is_bounded(self):
        _, mcu, pv2 = _build()
        calls = {'n': 0}

        def on_step(call):
            if call.by_field and call.by_field.get('step_cnt', 0) > 0 and call.by_field['dir'] == 0:
                calls['n'] += 1

        mcu.on_send_hook('start_step_prtouch', on_step)
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(0.5, retries=4, pro_cnt=1)
        self.assertEqual(calls['n'], 4, "must attempt exactly `retries` downward arms, no more")


class ExceptionCleanupTest(unittest.TestCase):
    def test_mesh_restored_even_when_probe_raises(self):
        printer, mcu, pv2 = _build()
        bed_mesh = printer.objects['bed_mesh']
        bed_mesh.set_mesh('a-real-mesh-object')
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(0.5, retries=1, pro_cnt=1)
        self.assertEqual(bed_mesh.set_mesh_calls[-1], 'a-real-mesh-object',
                          "mesh must be restored via try/finally even on a raised error")

    def test_fail_path_issues_final_safety_lift(self):
        _, mcu, pv2 = _build()
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(0.5, retries=1, pro_cnt=1)
        # _fail() calls safe_move_z(1, 5.0, 10.0) in addition to the per-attempt recovery -
        # confirm at least one upward safe_move_z-shaped arm was sent after the last
        # recovery, i.e. cleanup ran even on the terminal failure path.
        up_arms = [c for c in mcu.all_calls('start_step_prtouch')
                   if c.by_field['dir'] == 1 and c.by_field['step_cnt'] > 0]
        self.assertTrue(up_arms)


class MalformedResponseTest(unittest.TestCase):
    def test_no_valid_channel_bit_set_is_rejected_not_silently_accepted(self):
        # pres samples arrive (buffer fills) but tri_chs=0 - no channel ever crossed
        # threshold. compute_trigger_z must raise ValueError (see prtouch_calibration.py),
        # and the orchestration layer must treat this as a failed attempt, not a Z result.
        _, mcu, pv2 = _build()

        def on_step(call):
            if call.by_field and call.by_field.get('step_cnt', 0) > 0 and call.by_field['dir'] == 0:
                for chunk in fake.make_step_result(
                        pv2.mcu.step_oid, 0, _full_step_trace(200)):
                    mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, chunk)
                for chunk in fake.make_pres_result(
                        pv2.mcu.pres_oid, 0, 0x0, 32, _full_pres_trace(dip_at=None)):
                    mcu.push_response('result_run_pres_prtouch', pv2.mcu.pres_oid, chunk)

        mcu.on_send_hook('start_step_prtouch', on_step)
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(2.0, retries=2, pro_cnt=1)

    def test_malformed_channel_bitmask_still_recovers_toolhead(self):
        _, mcu, pv2 = _build()

        def on_step(call):
            if call.by_field and call.by_field.get('step_cnt', 0) > 0 and call.by_field['dir'] == 0:
                for chunk in fake.make_step_result(pv2.mcu.step_oid, 0, _full_step_trace(200)):
                    mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, chunk)
                for chunk in fake.make_pres_result(pv2.mcu.pres_oid, 0, 0x0, 32,
                                                    _full_pres_trace(dip_at=None)):
                    mcu.push_response('result_run_pres_prtouch', pv2.mcu.pres_oid, chunk)

        mcu.on_send_hook('start_step_prtouch', on_step)
        with self.assertRaises(Exception):
            pv2.probe.touch_probe(2.0, retries=1, pro_cnt=1)
        up_arms = [c for c in mcu.all_calls('start_step_prtouch')
                   if c.by_field['dir'] == 1 and c.by_field['step_cnt'] > 0]
        self.assertTrue(up_arms, "even a ValueError (bad channel data) must still lift back")


class SafeMoveZCleanupTest(unittest.TestCase):
    """Regression test for a real gap found 2026-08-06: safe_move_z's final disarm
    (start_step with step_cnt=0) previously ran only if collect_step_samples() completed
    without raising - a genuine buffer-repair failure (PrtouchProtocolError, or any other
    exception from the manual_get_steps query path) would skip it entirely. safe_move_z is
    _fail()'s own last-resort safety lift, so this mattered specifically on the one path
    meant to make failures safe."""

    def test_disarm_still_sent_when_repair_query_raises(self):
        _, mcu, pv2 = _build()

        def raising_repair(call):
            raise RuntimeError("simulated manual_get_steps comms failure")

        mcu.set_query_response('manual_get_steps', raising_repair)
        with self.assertRaises(RuntimeError):
            pv2.probe.safe_move_z(1, 5.0, 10.0)
        # the disarm (step_cnt=0) call must still have been sent despite the raise above
        disarm_calls = [c for c in mcu.all_calls('start_step_prtouch')
                         if c.by_field['step_cnt'] == 0]
        self.assertTrue(disarm_calls, "safe_move_z must disarm even when repair raises")


class SafetyGuardRejectionIsACommandErrorTest(unittest.TestCase):
    """Live incident, 2026-08-12: see prtouch_v2.py's _guarded() docstring for the full story -
    PrtouchProbeSafetyError/PrtouchProtocolError reaching Klipper's real gcode dispatcher
    unconverted triggers a full printer emergency_stop instead of a clean command rejection.
    Confirmed live: the fail-closed no-trusted-reference guard doing exactly its documented job
    took the whole printer down on the very first real touch attempt. Proves every prtouch_v2.py
    gcode command that can reach probe/mcu code now converts both known exception types into
    printer.command_error before they can escape to the dispatcher."""

    def test_safe_move_z_converts_probe_safety_error(self):
        _, mcu, pv2 = _build()
        pv2.probe.safe_move_z = lambda *a, **k: (_ for _ in ()).throw(
            prtouch_probe.PrtouchProbeSafetyError("raw op already active"))
        with self.assertRaises(fake.CommandError) as ctx:
            pv2.cmd_SAFE_MOVE_Z(fake.FakeGCmd())
        self.assertIn("raw op already active", str(ctx.exception))

    def test_safe_move_z_converts_protocol_error(self):
        _, mcu, pv2 = _build()
        pv2.probe.safe_move_z = lambda *a, **k: (_ for _ in ()).throw(
            prtouch_mcu.PrtouchProtocolError("stale buffer"))
        with self.assertRaises(fake.CommandError):
            pv2.cmd_SAFE_MOVE_Z(fake.FakeGCmd())

    def test_prtouch_test_touch_converts_probe_safety_error(self):
        _, mcu, pv2 = _build()
        pv2.probe.touch_probe = lambda *a, **k: (_ for _ in ()).throw(
            prtouch_probe.PrtouchProbeSafetyError("no trusted reference yet"))
        with self.assertRaises(fake.CommandError) as ctx:
            pv2.cmd_PRTOUCH_TEST_TOUCH(fake.FakeGCmd())
        self.assertIn("no trusted reference yet", str(ctx.exception))
        self.assertNotIsInstance(ctx.exception, prtouch_probe.PrtouchProbeSafetyError)

    def test_confirm_baseline_converts_probe_safety_error(self):
        _, mcu, pv2 = _build()
        pv2.probe.confirm_bootstrap_baseline = lambda *a, **k: (_ for _ in ()).throw(
            prtouch_probe.PrtouchProbeSafetyError("no candidate exists"))
        with self.assertRaises(fake.CommandError) as ctx:
            pv2.cmd_PRTOUCH_CONFIRM_BASELINE(fake.FakeGCmd())
        self.assertIn("no candidate exists", str(ctx.exception))

    def test_a_genuinely_unexpected_error_still_propagates_unconverted(self):
        # deliberately NOT one of the two known prtouch exception types - the whole-printer
        # shutdown fail-safe should still apply to failure modes this fix doesn't specifically
        # recognize, rather than silently downgrading every exception a probe method can raise.
        _, mcu, pv2 = _build()
        pv2.probe.touch_probe = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("something genuinely unexpected"))
        with self.assertRaises(RuntimeError):
            pv2.cmd_PRTOUCH_TEST_TOUCH(fake.FakeGCmd())


class PrtouchTestTouchCommandTest(unittest.TestCase):
    """2026-08-12 physical-qualification prep mission: PRTOUCH_TEST_TOUCH is the smallest
    production-compatible single-touch entry point (see prtouch_v2.py's cmd_PRTOUCH_TEST_TOUCH).
    Proves it: requires Z homed, performs exactly one real pressure-armed descent (reuses
    touch_probe(retries=1, pro_cnt=1) - already proven by StockFidelityOrderingTest to match
    stock's arm/disarm ordering), and never applies an offset or persists a calibration result."""

    def test_blocked_when_z_not_homed(self):
        printer, mcu, pv2 = _build()
        printer.objects['toolhead'].homed_axes = 'xy'
        with self.assertRaises(fake.CommandError):
            pv2.cmd_PRTOUCH_TEST_TOUCH(fake.FakeGCmd())

    def test_single_touch_reaches_touch_probe_with_one_attempt_one_sample(self):
        printer, mcu, pv2 = _build()
        _arm_trigger_response(mcu, pv2, dip_at=20, step_cnt_hint=200)
        gcmd = fake.FakeGCmd()
        pv2.cmd_PRTOUCH_TEST_TOUCH(gcmd)
        down_arms = [c for c in mcu.all_calls('start_step_prtouch')
                     if c.by_field['step_cnt'] > 0 and c.by_field['dir'] == 0]
        self.assertEqual(len(down_arms), 1, "PRTOUCH_TEST_TOUCH must send exactly one descent")
        self.assertIn('z=', gcmd.responses[0])
        self.assertIn('no offset applied', gcmd.responses[0])

    def test_single_touch_default_travel_is_small_and_capped(self):
        _, mcu, pv2 = _build()
        gcmd = fake.FakeGCmd(params={'DOWN_MIN_Z': '50'})
        with self.assertRaises(fake.CommandError):
            pv2.cmd_PRTOUCH_TEST_TOUCH(gcmd)

    def test_no_trigger_fails_after_exactly_one_attempt_no_retry(self):
        _, mcu, pv2 = _build()  # no _arm_trigger_response - every attempt is a genuine no-trigger
        gcmd = fake.FakeGCmd()
        with self.assertRaises(Exception):
            pv2.cmd_PRTOUCH_TEST_TOUCH(gcmd)
        down_arms = [c for c in mcu.all_calls('start_step_prtouch')
                     if c.by_field['step_cnt'] > 0 and c.by_field['dir'] == 0]
        self.assertEqual(len(down_arms), 1,
                          "a no-trigger result must not cause a second real descent attempt")


if __name__ == '__main__':
    unittest.main()
