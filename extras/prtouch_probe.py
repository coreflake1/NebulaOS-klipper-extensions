# prtouch_v2 touch-probe orchestration - send/poll/retry, delegates math to prtouch_calibration
#
# Clean-room rewrite of Creality's run_step_prtouch()/safe_move_z()/ck_and_raise_error()
# (prtouch_v2_wrapper.py, GPLv3, see reference/), read completely and traced in ../ANALYSIS.md
# secs 3-4/6. Deliberately not a verbatim port (ANALYSIS.md sec 6): the Z_RefreshFlag
# re-home-on-no-trigger branch, fast_probe's lost_min_cnt bookkeeping, and the re_g28
# auto-rehome path all exist in the original to serve run_G28_Z/bed_mesh_post_proc, which are
# confirmed dead code in real production (ANALYSIS.md sec 7) - this only needs to serve
# clear_nozzle() and the new z_compensate Z_OFFSET_CALIBRATION path, both of which probe with
# BLTouch already homed and a known-good Z reference, so a no-trigger here is a real failure to
# surface, not a homing state to silently repair.
#
# 2026-08-09 (load-cell safety hardening mission): three real safety gaps closed, all found
# offline via source tracing + the fake-MCU test harness, none requiring hardware access:
#
#   1. No-trigger retries never lifted the toolhead back up. On a genuine no-trigger, the
#      MCU's own step callback (reference/prtouch_v2.c's prtouch_event(), confirmed at the
#      now_steps-- / now_steps==0 check) only stops early on a real trigger or when the full
#      commanded pulse train completes - so an empty/no-trigger buffer means the *full*
#      commanded descent was physically executed. Klipper's own toolhead position tracking is
#      never told about this raw, MCU-driven motion, so without an explicit compensating lift,
#      every retry would recompute another full-depth descent from the same stale start
#      height - i.e. `retries` consecutive full blind descents stacked in the same direction,
#      bounded by nothing but the stepper stalling against the bed. Fixed by
#      _recover_after_no_trigger(), which undoes the full *commanded* step_cnt (not
#      sample-derived - a no-trigger response is empty by definition) before the next attempt
#      or before _fail() raises (see _fail()'s own docstring - as of 2026-08-13 it no longer
#      issues its own lift on top of this one). Deliberately does NOT replicate the reference's
#      toolhead.set_position() re-homing (which redefines Z=0 at the failure point) - this
#      module's whole design principle is that BLTouch's Z=0 stays the one authoritative
#      reference; silently redefining it from a failed touch would undermine
#      Z_OFFSET_CALIBRATION's own correction, not protect it.
#   2. Every raw lift's final disarm (the trailing start_step(..., 0, ...) that stops the step
#      channel) only ran if collect_step_samples() completed without raising. A genuine
#      buffer-repair failure (PrtouchProtocolError, or any comms exception on the
#      manual_get_steps query path) would skip it - on the ONE path meant to make a failure
#      safe, that's exactly backwards. Every raw lift (safe_move_z, and both callers of the new
#      shared _raw_lift helper below) now disarms in a finally block, unconditionally.
#   3. The MCU's own baseline sensor read (deal_avgs_prtouch, already sent once per attempt
#      purely as a "is the sensor alive" check - see prtouch_v2.py's READ_PRES) was collected
#      but its result was discarded. Now checked, before ever arming a real descent, against
#      two conservative, configurable guards: the reading must be finite and within a wide
#      sanity envelope (catches a disconnected/stuck/saturated sensor), and its magnitude must
#      not already be at or past this attempt's own trigger sensitivity (tri_min_hold) - a
#      deliberately conservative proxy for "the sensor already reads as loaded/triggered before
#      any motion has happened," using the SAME threshold real trigger detection already relies
#      on rather than inventing a new one. This is a raw-signal heuristic, not a replica of the
#      MCU's own filtered, hold-count-based trigger logic - see PRE_MOTION_SENSOR_CHECK's own
#      docstring for exactly what it does and does not prove; the real threshold needs hardware
#      qualification (NEEDS_HARDWARE_DATA), the STRUCTURE of refusing to move on a bad reading
#      does not.
#
# Also new this mission: an explicit maximum-travel bound (max_probe_travel_mm) checked once,
# up front, before touch_probe() ever arms a single command - independent of whatever
# down_min_z a caller passes in (z_compensate.py's own config bounds its own down_min_z, but
# this is a shared API other callers use too, like clear_nozzle()'s g29_down_min_z) - and an
# explicit maximum total wall-clock duration (max_probe_duration_s) across the whole retry
# loop, so a pathological run of legitimately-timing-out attempts can't continue indefinitely.
# Neither is a new physical threshold requiring hardware data - both are conservative ceilings
# on top of the existing, already-configured per-call values.
#
# 2026-08-10 (raw-op timer-incident mission, see docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md):
# a live no-trigger test of touch_probe() ended in a real MCU firmware shutdown
# ("sentinel timer called"), preceded by five "Timer too close" MCU warnings. Source-level
# tracing of the running firmware's exact scheduler was proven infeasible (the running
# 38d96adc-dirty-20231016_135251 build does not match any source in this repo's history -
# see the forensics doc), so this fix is evidence-based on observed host/MCU protocol
# behavior, not on proprietary firmware internals: every single "Timer too close" in that
# incident occurred immediately after a disarm-then-immediate-rearm transition on the raw
# step channel, with zero host-side yield in between (the transition either between a probe
# attempt's own down-then-recovery-lift, or between one retry attempt's recovery lift and the
# next attempt's own down arm). Two changes address this, independent of whatever the exact
# firmware-level trigger turns out to be:
#   - _own_raw_operation(): only one of this module's two public raw-motion entry points
#     (touch_probe, safe_move_z) may be active at a time - closes the class of risk where a
#     second, independent invocation could overlap raw MCU traffic with an in-flight one,
#     regardless of what triggers it (a second gcode/script request, a stray macro, etc.).
#     The live incident's own second, unexplained Z_OFFSET_CALIBRATION request was proven NOT
#     to be this incident's actual cause (it arrived 1.6s after the first "Timer too close"),
#     but there is no legitimate reason to permit the overlap regardless.
#   - _settle_after_disarm(): a minimum yield after every disarm before the next arm, using
#     the protocol's own declared pacing granularity (tri_send_ms) as the most evidence-
#     grounded default available rather than an invented constant - see its own docstring.
#
# 2026-08-13 (redundant-recovery-lift mission): a real, non-retried PRTOUCH_TEST_TOUCH attempt
# (docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md sec 16) showed _fail() always issuing its own
# 5mm safety lift even when the no-trigger recovery immediately before it had already restored
# the full commanded descent - 3 raw disarms for a sequence that only ever needed 2. That
# specific attempt did not crash the MCU (checked against the actual 2026-08-10 shutdown
# incident's own timeline in sec 2 of the same doc, which stalled one level earlier, during a
# recovery lift, with retries still remaining - _fail() was never reached), so this is
# independent hardening, not a fix for that still-open incident. _fail() no longer moves at
# all; _raw_lift() (shared by _recover_after_no_trigger/_lift_after_down) now latches
# _raw_channel_healthy False and refuses all further raw ops (via _own_raw_operation) if a
# recovery move itself fails to complete, instead of ever guessing with another move.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import contextlib
import json
import logging
import math
import os

from . import prtouch_calibration
from . import prtouch_units as units


class PrtouchProbeSafetyError(Exception):
    """Raised by a pre-motion guard (max travel/duration, invalid/already-triggered sensor
    reading) that refuses to arm any real movement at all - distinct from a command_error
    raised after a real, physical attempt (via _fail(), once whatever motion that attempt made
    has already been recovered - see _fail()'s own docstring) since these guards trip BEFORE
    any motion this call has commanded, so there is nothing of this call's own to recover
    from."""


class PrtouchProbe:
    def __init__(self, config, mcu):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu
        self.toolhead = None

        use_adc = mcu.use_adc
        self.tri_acq_ms = config.getint('tri_acq_ms', default=(1 if use_adc else 12), minval=1)
        self.tri_send_ms = config.getint('tri_send_ms', default=10, minval=1)
        self.tri_need_cnt = config.getint('tri_need_cnt', default=1, minval=1)
        self.cal_hftr_cut = config.getfloat('cal_hftr_cut', default=10., minval=0.01)
        self.cal_lftr_k1 = config.getfloat('cal_lftr_k1', default=(0.65 if use_adc else 0.85))
        self.tri_min_hold = config.getint('tri_min_hold', default=(3 if use_adc else 2000))
        self.tri_max_hold = config.getint('tri_max_hold', default=(3072 if use_adc else 6000))
        self.tri_hftr_cut = config.getfloat('tri_hftr_cut', default=2.0)
        self.tri_lftr_k1 = config.getfloat('tri_lftr_k1', default=(0.50 if use_adc else 0.70))
        # 'speed' is this printer's own real [prtouch_v2] key for probe descent speed - neither
        # our own prior 'tri_z_down_spd' name nor the reference wrapper's 'speeds' (plural,
        # float-list) match it; confirmed via SSH 2026-08-05 that the real section has a bare
        # 'speed: 1' with no 'tri_z_down_spd'/'speeds' key at all. 'tri_z_down_spd' is kept as a
        # fallback name, not a real key on this printer, so it stays fully optional.
        self.tri_z_down_spd = config.getfloat(
            'speed',
            config.getfloat('tri_z_down_spd', default=(10. if use_adc else 2.5), minval=0.1),
            minval=0.1)
        self.tri_z_up_spd = config.getfloat(
            'lift_speed', default=self.tri_z_down_spd * (1.0 if use_adc else 2.0), minval=0.1)
        self.acc_ctl_mm = config.getfloat('acc_ctl_mm', default=(0.5 if use_adc else 0.25),
                                           minval=0)
        self.low_spd_nul = config.getint('low_spd_nul', default=5, minval=1, maxval=10)
        self.send_step_duty = config.getint('send_step_duty', default=16, minval=0, maxval=10)
        self.probe_min_3err = config.getfloat('probe_min_3err', default=0.1, minval=0.01)
        self.step_base = config.getint('step_base', default=1, minval=1)

        # Safety ceilings (2026-08-09 hardening mission) - conservative, configurable, deliberately
        # NOT tuned to this specific sensor's real physical limits (that needs hardware
        # qualification - see module docstring). max_probe_travel_mm's default (50mm) matches
        # z_compensate.py's own long-standing z_offset_down_min_z maxval, so it never rejects
        # that call site's own already-validated config; it exists to bound OTHER callers of
        # this shared API too (e.g. clear_nozzle()'s g29_down_min_z).
        self.max_probe_travel_mm = config.getfloat('max_probe_travel_mm', default=50.,
                                                     minval=1., maxval=100.)
        self.max_probe_duration_s = config.getfloat('max_probe_duration_s', default=120.,
                                                      minval=1.)
        # max_baseline_abs: a sanity envelope for the raw, unfiltered deal_avgs_prtouch
        # reading, not a trigger threshold - live hardware's own documented at-rest baseline is
        # roughly -251,500 (NON_MOTION_VALIDATION.md, DESIGN.md 2026-08-05 entry); this default
        # gives roughly 20x headroom above that magnitude before treating a reading as
        # implausible/saturated/disconnected, deliberately wide since the real dynamic range
        # under an actual approaching/contacting nozzle is unmeasured (NEEDS_HARDWARE_DATA).
        self.max_baseline_abs = config.getfloat('max_baseline_abs', default=5000000.,
                                                  minval=1.)
        # baseline_reference/baseline_deviation_max: OPTIONAL, OFF BY DEFAULT "already
        # triggered before any motion" guard. Deliberately NOT derived from tri_min_hold/
        # tri_max_hold - those threshold the FILTERED (high-pass + low-pass) delta signal
        # touch_probe()'s own real trigger detection uses (see prtouch_calibration.py's
        # filter_pressure_series), not the raw deal_avgs_prtouch magnitude, which carries a
        # large sensor-specific DC offset (this printer's own real documented at-rest reading
        # is roughly -251,500 - already proven, directly from real hardware data captured in
        # this codebase's own test fixtures, to be two-plus orders of magnitude past
        # tri_min_hold's own 1000-2000 range; an earlier version of this guard compared raw
        # magnitude straight against tri_min_hold and would have rejected every single real
        # probe attempt outright - caught before ever reaching hardware, by checking this
        # exact scenario against the existing real-baseline fixture value). With no reference
        # configured (the default), this guard is a documented no-op - genuinely
        # NEEDS_HARDWARE_DATA, not a fabricated threshold. Once real at-rest values are known
        # (see read_diagnostics()/READ_PRES for how to capture them), set baseline_reference to
        # those 4 values and baseline_deviation_max to a real, qualified tolerance to activate
        # it - see docs/prtouch_diagnostics.md.
        self.baseline_reference = config.getfloatlist('baseline_reference', default=None,
                                                        count=4)
        self.baseline_deviation_max = config.getfloat('baseline_deviation_max', default=None,
                                                        minval=0.)
        if self.baseline_reference is not None and self.baseline_deviation_max is None:
            raise config.error(
                "prtouch_probe: baseline_reference is set but baseline_deviation_max is not - "
                "both are required together")

        # Shared raw-operation ownership + settle timing (2026-08-10, see module docstring).
        # Scoped to this class's two PUBLIC raw-motion entry points only (touch_probe,
        # safe_move_z) - every other method that touches the raw step/pres channel is a
        # private helper only ever reached from one of those two, so no reentrancy handling
        # is needed beyond that (see _own_raw_operation's own docstring).
        self._raw_op_active = False
        self._raw_op_name = None
        self._raw_op_id = 0
        # Raw-channel health latch (2026-08-13, redundant-recovery-lift mission). Set False only
        # when a recovery/lift move (_raw_lift) itself fails to complete - i.e. the physical
        # position is no longer provably known. Deliberately NOT auto-cleared: the only recovery
        # from "position uncertain" is a human re-homing/restart, not another guess-and-move.
        # See _raw_lift's own comment for why this is the sole place that ever sets it False.
        self._raw_channel_healthy = True
        # raw_op_settle_s: minimum yield after a disarm before the next arm. The real minimum
        # safe gap is NOT known - that needs hardware qualification - so this defaults to
        # tri_send_ms (the same value the MCU firmware itself uses, via check_delay(), to pace
        # its own buffered-sample sends on this exact channel - confirmed against
        # reference/prtouch_v2.c), converted to seconds, rather than an invented constant.
        # None (the default) means "derive from tri_send_ms"; set explicitly once real
        # hardware timing margins are measured.
        self._raw_op_settle_s_override = config.getfloat('raw_op_settle_s', default=None,
                                                           minval=0.)

        # Sensor-consistency guard (2026-08-11, physical-qualification closure mission - see
        # docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md's follow-up): live testing found that a
        # SINGLE deal_avgs_prtouch read (the guard above) is not sufficient. After a raw step
        # operation (safe_move_z, or touch_probe's own down/lift phases), READ_PRES was
        # observed to intermittently return near-zero/partial-magnitude garbage (e.g. -1,
        # -63923, -127864 against a real ~-256000 baseline) that is individually finite and
        # well under max_baseline_abs - it passes _evaluate_baseline outright, so the existing
        # single-read guard cannot see it. check_sensor_consistency() takes several independent
        # reads and requires them to agree with each other and with this session's own
        # auto-learned healthy baseline (see check_sensor_consistency's own docstring) -
        # defense in depth, not a fix for whatever is corrupting the underlying MCU read.
        self.sensor_consistency_reads = config.getint(
            'sensor_consistency_reads', default=3, minval=2, maxval=10)
        self.sensor_consistency_settle_s = config.getfloat(
            'sensor_consistency_settle_s', default=0.05, minval=0.)
        # NEEDS_HARDWARE_DATA-style defaults, but grounded in real data from this session, not
        # invented: four genuine healthy back-to-back reads spread across ~300 counts
        # (-255752..-256031); the corrupted reads observed were tens of thousands to the full
        # baseline magnitude off. 5000/10000 give >15x headroom over real observed noise while
        # staying two-plus orders of magnitude below the real corruption seen.
        self.sensor_consistency_max_spread = config.getfloat(
            'sensor_consistency_max_spread', default=5000., minval=1.)
        self.sensor_baseline_max_drift = config.getfloat(
            'sensor_baseline_max_drift', default=10000., minval=1.)
        # Persisted per-channel reference (2026-08-12, root-cause mission - see
        # docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md's disassembly-grounded root-cause
        # section): a SESSION-LOCAL auto-learned baseline (the original 2026-08-11 version of
        # this guard) has a real gap - if the sensor is already corrupted before the first
        # healthy-looking read this session (e.g. Klipper restarted while the MCU/HX711 was
        # still in a corrupted state from an earlier raw step op), a corrupted-but-internally-
        # consistent reading could get learned as "healthy" with nothing to compare it against.
        # Persisting the reference to disk and gating every future update against the
        # PERSISTED value (not just this session's own view) closes that gap: once a real
        # baseline is established, only readings already close to it can ever update it again -
        # see check_sensor_consistency()'s own docstring for exactly how. Distinct from the
        # user-configurable baseline_reference above (which stays opt-in/unset by default);
        # this one self-populates so the drift check works even with no manual config, and
        # survives Klipper restarts / Linux reboots (loaded in _handle_connect below) - the one
        # case it cannot help with is a config's very first-ever boot with no persisted file
        # yet - see the three-state model below for how that case is handled without
        # silently trusting an unverified reading.
        self.baseline_persist_path = config.get(
            'baseline_persist_path', default='/opt/printer_data/prtouch_baseline.json')
        # Three explicit states (2026-08-12 root-cause mission, closing a real gap found in
        # the 2026-08-11 version above: an internally-consistent-but-unverified first reading
        # is NOT the same as a verified-healthy one - a sensor stuck at a stable-but-corrupted
        # value would pass every self-consistency check on every single restart, forever, so
        # trusting it on first sight was never actually safe):
        #   NO_REFERENCE        - self._auto_baseline is None and self._bootstrap_candidate is
        #                          None. Nothing to compare against, nothing to confirm yet.
        #   BOOTSTRAP_CANDIDATE - self._auto_baseline is None, self._bootstrap_candidate holds
        #                          the latest internally-consistent-but-unverified reading.
        #                          Visible for diagnostics; check_sensor_consistency() reports
        #                          it as ok=False, state='bootstrap_candidate' - it can NEVER
        #                          authorize touch_probe()/Z_OFFSET_CALIBRATION on its own.
        #   TRUSTED_REFERENCE   - self._auto_baseline is set (persisted, survives restarts).
        #                          Only reachable by matching an EXISTING trusted reference
        #                          within tolerance, or by an explicit human confirmation (see
        #                          confirm_bootstrap_baseline()/PRTOUCH_CONFIRM_BASELINE) -
        #                          never automatically from a bootstrap candidate alone.
        self._auto_baseline = None
        self._bootstrap_candidate = None

        self.mm_per_step = None
        self.bed_mesh = None
        # Diagnostics-only, zero-motion state (see read_diagnostics()) - last_error is
        # deliberately not cleared by a successful read; it is cleared only by
        # touch_probe()/safe_move_z() actually running without raising, matching
        # z_compensate.py's own calibration_error contract (a status field describes the last
        # real outcome, not just "no error was observed since the module last checked").
        self.last_error = None
        # get_status()-facing cache (see prtouch_v2.py's own get_status()) - Klipper's status/
        # webhooks layer polls get_status() on its own schedule (potentially frequently, e.g.
        # from a live GuppyScreen screen), and that call must stay cheap and never itself touch
        # the MCU (a synchronous serial round-trip on every status poll would be both wasteful
        # and a layering violation - get_status() implementations elsewhere in this codebase,
        # see z_compensate.py, only ever return already-computed state). read_diagnostics()
        # updates this cache every time it actually reads the sensor (explicitly via READ_PRES,
        # or implicitly via touch_probe()'s own pre-motion/per-attempt baseline checks), so a
        # status subscriber always sees the most recent REAL observation, whether that came
        # from a deliberate diagnostic request or an actual probe attempt - never a live read
        # triggered by the act of checking status itself.
        self.last_diagnostic = {
            'ok': None, 'raw': None, 'reason': 'no reading taken yet', 'state': None,
            'tri_min_hold': self.tri_min_hold, 'tri_max_hold': self.tri_max_hold,
            'max_baseline_abs': self.max_baseline_abs,
        }
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.bed_mesh = self.printer.lookup_object('bed_mesh', None)
        for stepper in self.toolhead.get_kinematics().get_steppers():
            if stepper.is_active_axis('z'):
                self.mm_per_step = self.step_base * stepper.get_step_dist()
                break
        if self.mm_per_step is None:
            raise self.printer.config_error("prtouch_probe: no active Z stepper found")
        self._load_persisted_baseline()

    def _load_persisted_baseline(self):
        """Loads self._auto_baseline from baseline_persist_path, if a prior session left one -
        see __init__'s own comment on why this needs to survive Klipper restarts/Linux reboots,
        not just live in memory for the current session. Never raises: a missing/corrupt/
        unreadable file just means no trusted reference yet (the next healthy read bootstraps
        one, same as this guard's original in-memory-only behavior) - a persistence failure
        must never itself block probing, only the sensor-health checks that already exist for
        that purpose."""
        self._auto_baseline = None
        try:
            with open(self.baseline_persist_path) as f:
                data = json.load(f)
            values = data.get('baseline')
            if (isinstance(values, list) and len(values) == 4
                    and all(isinstance(v, (int, float)) for v in values)):
                self._auto_baseline = [float(v) for v in values]
                logging.info("prtouch_probe: loaded persisted sensor baseline %s from %s",
                             self._auto_baseline, self.baseline_persist_path)
            else:
                logging.warning(
                    "prtouch_probe: %s exists but its 'baseline' field isn't 4 numbers (%r) - "
                    "ignoring it, next healthy read re-bootstraps", self.baseline_persist_path,
                    values)
        except FileNotFoundError:
            pass
        except Exception as e:
            logging.warning(
                "prtouch_probe: could not load persisted baseline from %s: %s - starting with "
                "no trusted reference (next healthy read re-bootstraps)",
                self.baseline_persist_path, e)

    def _save_persisted_baseline(self, values):
        """Writes self._auto_baseline to baseline_persist_path - see check_sensor_consistency()
        for the policy governing WHEN this is called (only from an already-accepted 'healthy'
        verdict, which by construction is already within tolerance of whatever was persisted
        before, if anything was). Write-then-rename so a crash/power-loss mid-write can never
        leave a truncated/corrupt file behind for _load_persisted_baseline() to trip over.
        Never raises: a persistence failure must not block the probe attempt that triggered
        it - it only means this particular update won't survive a restart."""
        try:
            tmp_path = self.baseline_persist_path + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump({'baseline': list(values)}, f)
            os.replace(tmp_path, self.baseline_persist_path)
        except Exception as e:
            logging.warning(
                "prtouch_probe: failed to persist sensor baseline to %s: %s - in-memory "
                "reference still updated, but won't survive a restart",
                self.baseline_persist_path, e)

    def get_step_counts(self, distance, speed):
        """get_step_cnts-equivalent: distance/speed -> (step_cnt, step_us, acc_ctl_cnt) for the
        MCU's start_step_prtouch pulse-train command."""
        step_cnt = units.distance_mm_to_step_count(distance, self.mm_per_step)
        if step_cnt <= 0:
            return 0, 0, 0
        step_us = units.step_count_to_step_us(distance, speed, step_cnt)
        acc_ctl_cnt = units.distance_mm_to_acc_ctl_cnt(self.acc_ctl_mm, self.mm_per_step)
        return step_cnt, step_us, acc_ctl_cnt

    @contextlib.contextmanager
    def _own_raw_operation(self, op_name):
        """Reject a second raw PRTouch operation that tries to start while one (touch_probe or
        safe_move_z) is already active, instead of letting it queue behind/interleave with the
        first - see module docstring and docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md secs 3/9.
        Checked and set with no yield in between, so this is race-free under Klipper's single-
        threaded/cooperative reactor without needing a lock: whichever call reaches this line
        first always sets the flag before it can ever yield (via reactor.pause(), which only
        happens once collect_step_samples()/_settle_after_disarm() run below), so a second call
        - however it was triggered, from any client - is guaranteed to observe it already set.
        Deliberately wraps only the two PUBLIC entry points (touch_probe, safe_move_z), not the
        private helpers they call internally (_fail/_raw_lift/_raw_move/_lift_after_down/
        _recover_after_no_trigger) - those are only ever reached from within an already-held
        operation, so re-entering this guard for them would be both unnecessary and wrong
        (it would make _recover_after_no_trigger's own recovery lift raise instead of
        running)."""
        if not self._raw_channel_healthy:
            raise PrtouchProbeSafetyError(
                "prtouch: raw PRTouch channel is latched unhealthy after a recovery move did "
                "not complete cleanly - physical position is no longer provably known; refusing "
                "to start %s until a Klipper restart (FIRMWARE_RESTART) and re-homing" % op_name)
        if self._raw_op_active:
            raise PrtouchProbeSafetyError(
                "prtouch: a raw PRTouch operation (%s) is already in progress - refusing to "
                "start %s" % (self._raw_op_name, op_name))
        self._raw_op_active = True
        self._raw_op_name = op_name
        self._raw_op_id += 1
        op_id = self._raw_op_id
        logging.info("prtouch_probe: raw op #%d start (%s)", op_id, op_name)
        try:
            yield op_id
        finally:
            logging.info("prtouch_probe: raw op #%d end (%s)", op_id, op_name)
            self._raw_op_active = False
            self._raw_op_name = None

    @property
    def _raw_op_settle_s(self):
        if self._raw_op_settle_s_override is not None:
            return self._raw_op_settle_s_override
        return self.tri_send_ms / 1000.0

    def _settle_after_disarm(self):
        """Yield at least one raw_op_settle_s tick after a disarm before the next raw MCU arm -
        see this module's own header comment for the incident evidence this responds to, and
        _own_raw_operation's docstring for why this cannot itself reopen the race that guard
        closes (the operation-active flag stays set for the whole duration of this yield)."""
        eventtime = self.reactor.monotonic()
        self.reactor.pause(eventtime + self._raw_op_settle_s)

    def safe_move_z(self, direction, distance, speed):
        """Non-probing raw Z move via the same MCU step command (safe_move_z-equivalent) - used
        for general manual moves outside a probe cycle. direction: 1 = up, 0 = down. This is the
        PUBLIC entry point (guarded by _own_raw_operation); _raw_move() is its own private
        helper, kept separate from touch_probe()'s down-arm (which arms/disarms the step
        channel directly - see _touch_probe) since a probe descent must run concurrently with
        start_pres_prtouch, which safe_move_z deliberately does not (see below).

        2026-08-12 stock-vs-NebulaOS fidelity mission: the one confirmed remaining deviation
        from reference/prtouch_v2_wrapper.py's own safe_move_z() (lines 1122-1151) is that stock
        arms start_pres_prtouch concurrently with every real move (so a manual jog can itself
        register a trigger and early-stop), while this stays step-only/blind on purpose - this
        tool exists to isolate raw step behavior from pressure-channel complexity, and a
        diagnostic that can silently early-stop on a spurious trigger would defeat that purpose.
        This is unrelated to the read_pres_prtouch/prtouch_event ISR-collision corruption
        mechanism (see NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md sec 12) - that lives entirely
        inside the raw pressure read, not in whether start_pres happens to be armed during a
        step move - and both this path and _touch_probe()'s share the same _settle_after_disarm
        gap and fail-closed sensor-consistency guard, so this simplification carries no cost
        against that mechanism. Classified NEBULAOS SAFETY IMPROVEMENT / DELIBERATE
        SIMPLIFICATION, not a bug - see that same doc's sec 15."""
        with self._own_raw_operation('safe_move_z') as op_id:
            self._raw_move(direction, distance, speed, op_id)

    def _raw_move(self, direction, distance, speed, op_id=None):
        step_cnt, step_us, acc_ctl_cnt = self.get_step_counts(distance, speed)
        if step_cnt == 0:
            return
        logging.info(
            "prtouch_probe: raw op #%s arm dir=%d step_cnt=%d step_us=%d acc_ctl_cnt=%d "
            "send_ms=%d", op_id, direction, step_cnt, step_us, acc_ctl_cnt, self.tri_send_ms)
        self.mcu.reset_buffers()
        self.mcu.start_step(direction, step_cnt, step_us, acc_ctl_cnt,
                             send_ms=self.tri_send_ms, low_spd_nul=self.low_spd_nul,
                             send_step_duty=self.send_step_duty)
        try:
            self.mcu.collect_step_samples(
                units.probe_timeout_seconds(distance, speed, margin_s=5.0))
        finally:
            # Found 2026-08-06: previously unguarded - a genuine buffer-repair failure
            # (PrtouchProtocolError) here would skip the disarm below entirely. This is
            # safe_move_z's own raw-move path, so letting a repair failure mask the real
            # command_error (and leave the step channel armed) would be exactly the wrong
            # failure mode to introduce into a path a user can invoke directly.
            self.mcu.stop_step()
            logging.info("prtouch_probe: raw op #%s disarm dir=%d", op_id, direction)
            self._settle_after_disarm()

    def _fail(self, message, op_id=None):
        """ck_and_raise_error-equivalent, minus its own courtesy safety lift (removed
        2026-08-13, redundant-recovery-lift mission - see module docstring's incident log
        entry). Every call site in _touch_probe reaches _fail() only after either (a) the
        descent that led to this failure was already fully recovered via
        _recover_after_no_trigger/_lift_after_down (both of which restore the complete
        commanded travel before returning, and latch _raw_channel_healthy False + raise
        instead of returning if they can't - see _raw_lift), or (b) no descent has been armed
        yet this attempt (e.g. the per-attempt baseline guard). In both cases physical position
        is already correct/unchanged, so an unconditional extra lift here was pure redundant
        motion stacked on top of an already-completed recovery - confirmed live: a single
        non-retried PRTOUCH_TEST_TOUCH attempt produced 3 raw disarms (descent, recovery lift,
        this lift) instead of the 2 the sequence actually needed, each logging a MCU-side
        `Timer too close` (docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md sec 16 - that same
        sequence did NOT crash the MCU; this is a redundant-motion cleanup, not a fix for the
        separate, still-unresolved 2026-08-10 MCU-shutdown incident documented in sec 2)."""
        logging.info("prtouch_probe: %s", message)
        self.last_error = message
        raise self.printer.command_error("prtouch: " + message)

    def read_diagnostics(self, base_cnt=8):
        """Zero-motion sensor diagnostic - a pure deal_avgs_prtouch read (see prtouch_mcu.py's
        deal_avgs()) with no start_step_prtouch involved anywhere in this call, ever. Returns
        raw channel values, the configured trigger thresholds those values will eventually be
        compared against, and a plausibility verdict from the exact same guard touch_probe()
        itself runs before arming any real descent - so a caller can see, before ever risking
        motion, whether that guard would currently allow a probe attempt to proceed at all.
        Never raises - a caller surfacing this in a UI/status field wants a description of a
        bad reading, not an exception. Updates self.last_diagnostic (see its own comment in
        __init__ for why get_status() reads that cache instead of calling this directly)."""
        try:
            raw = self.mcu.deal_avgs(base_cnt=base_cnt)
        except Exception as e:
            self.last_diagnostic = {
                'ok': False, 'raw': None, 'reason': 'mcu_read_failed: %s' % e,
                'state': 'corrupted',
                'tri_min_hold': self.tri_min_hold, 'tri_max_hold': self.tri_max_hold,
                'max_baseline_abs': self.max_baseline_abs,
            }
            return self.last_diagnostic
        ok, reason = self._evaluate_baseline(raw)
        self.last_diagnostic = {
            'ok': ok, 'raw': raw, 'reason': reason,
            'state': 'healthy' if ok else 'corrupted',
            'tri_min_hold': self.tri_min_hold, 'tri_max_hold': self.tri_max_hold,
            'max_baseline_abs': self.max_baseline_abs,
        }
        return self.last_diagnostic

    def _evaluate_baseline(self, raw):
        """Shared verdict logic for read_diagnostics() and the real pre-motion guard
        (_check_baseline_safe) - kept as one function so the diagnostic a user inspects before
        ever running a real calibration is provably the exact same check that calibration
        itself relies on, not a lookalike that could silently drift out of sync.

        Two failure classes, both fail closed (refuse to proceed) rather than guessing:
          - non-finite or |value| > max_baseline_abs on any channel: "invalid sensor data" -
            a disconnected, saturated, or otherwise implausible reading. Always active.
          - |value - baseline_reference[i]| > baseline_deviation_max on any channel: "already
            triggered"/"trigger before movement" - only active once baseline_reference is
            explicitly configured (see __init__'s own comment for why this can't default to
            anything derived from tri_min_hold, and why it stays off, not guessed, until real
            hardware data sets it).
        """
        channels = [raw.get('ch%d' % i) for i in range(4) if 'ch%d' % i in raw]
        if not channels:
            return False, "no channel values in MCU response"
        for i, value in enumerate(channels):
            if value is None or not math.isfinite(value):
                return False, "non-finite channel value %r" % (value,)
            if abs(value) > self.max_baseline_abs:
                return False, ("channel magnitude %r exceeds max_baseline_abs=%r - invalid/"
                                "saturated/disconnected sensor" % (value, self.max_baseline_abs))
            if self.baseline_reference is not None and i < len(self.baseline_reference):
                deviation = abs(value - self.baseline_reference[i])
                if deviation > self.baseline_deviation_max:
                    return False, (
                        "channel %d value %r deviates %r from baseline_reference %r (max "
                        "allowed %r) before any motion - refusing to start a probe on an "
                        "already-loaded/triggered reading" % (
                            i, value, deviation, self.baseline_reference[i],
                            self.baseline_deviation_max))
        return True, None

    def check_sensor_consistency(self, base_cnt=8):
        """Fail-closed pre-motion sensor-health gate (2026-08-11, physical-qualification
        closure mission). A single deal_avgs_prtouch read (read_diagnostics()) was proven live
        insufficient: after a raw step operation (safe_move_z, or touch_probe's own down/lift
        phases), READ_PRES was observed to intermittently return near-zero/partial-magnitude
        garbage (e.g. -1, -63923, -127864, against a real ~-256000 baseline) that is
        individually finite and well under max_baseline_abs, so it sailed through
        _evaluate_baseline outright - no single read can distinguish that from genuine data.

        Takes sensor_consistency_reads independent deal_avgs_prtouch reads, spaced
        sensor_consistency_settle_s apart, and only returns a 'healthy' verdict if ALL of:
          - every individual read passes _evaluate_baseline (finite, within max_baseline_abs,
            any configured baseline_reference deviation) - otherwise 'corrupted', the clearest
            failure class.
          - the reads agree with EACH OTHER within sensor_consistency_max_spread per channel -
            catches exactly the flickering behavior actually observed live, which needs at
            least two reads to see at all.
          - if a trusted baseline is already established (see self._auto_baseline, loaded from
            baseline_persist_path on klippy:connect if a prior session already wrote one - see
            __init__'s own comment on why this must survive restarts, not just live in memory),
            this batch's mean must not drift from it by more than sensor_baseline_max_drift -
            catches a reading that is internally consistent but consistently WRONG (e.g. stuck
            at a stable-but-false value), which the spread check alone cannot catch.
        Both failure classes are reported as diag['state'] in {'unstable', 'corrupted'} (vs.
        'healthy') so a caller/status subscriber can distinguish "sensor looks actively broken"
        from "sensor readings disagree with each other or with the trusted baseline" - both
        refuse to proceed (diag['ok'] is False for either), matching this method's fail-closed
        contract; only the label differs.

        Updates self.last_diagnostic like read_diagnostics(), plus a 'bootstrap' bool (True only
        when this call established the very first trusted baseline this printer has ever
        persisted - worth surfacing since that one case is trusted on first sight, not verified
        against anything). Only ever refreshes self._auto_baseline AND its on-disk copy on a
        'healthy' verdict - a rejected batch can never poison the reference future checks (this
        session's or a future restarted one) compare against.

        NEEDS_HARDWARE_DATA: sensor_consistency_max_spread/sensor_baseline_max_drift are sized
        against this session's own real observed idle noise floor (~300 counts across genuine
        healthy reads), not a fabricated number - see this class's __init__ comment for the
        exact live data point."""
        def _reject(raw, reason, state):
            self.last_diagnostic = {
                'ok': False, 'raw': raw, 'reason': reason, 'state': state,
                'tri_min_hold': self.tri_min_hold, 'tri_max_hold': self.tri_max_hold,
                'max_baseline_abs': self.max_baseline_abs,
            }
            return self.last_diagnostic

        reads = []
        for i in range(self.sensor_consistency_reads):
            if i > 0:
                eventtime = self.reactor.monotonic()
                self.reactor.pause(eventtime + self.sensor_consistency_settle_s)
            try:
                raw = self.mcu.deal_avgs(base_cnt=base_cnt)
            except Exception as e:
                return _reject(None, 'mcu_read_failed: %s' % e, 'corrupted')
            ok, reason = self._evaluate_baseline(raw)
            if not ok:
                return _reject(raw, reason, 'corrupted')
            reads.append(raw)

        keys = ['ch%d' % i for i in range(4)]
        spreads = {}
        for key in keys:
            vals = [r[key] for r in reads if key in r]
            if vals:
                spreads[key] = max(vals) - min(vals)
        max_spread = max(spreads.values()) if spreads else 0.
        if max_spread > self.sensor_consistency_max_spread:
            return _reject(reads[-1], (
                "repeated reads disagree with each other (max spread %.0f counts across %d "
                "channel(s) sampled %d times, allowed %.0f) - sensor reading is unstable, not "
                "trustworthy for a real probe attempt right now" % (
                    max_spread, len(spreads), len(reads),
                    self.sensor_consistency_max_spread)), 'unstable')

        latest = reads[-1]
        batch_mean = [
            (sum(r[key] for r in reads if key in r) / len(reads)) if any(key in r for r in reads)
            else 0. for key in keys]

        if self._auto_baseline is None:
            # NO_REFERENCE -> BOOTSTRAP_CANDIDATE (2026-08-12 root-cause mission - see
            # docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md's final synthesis): an internally-
            # consistent reading with nothing trusted to compare it against is NOT the same as
            # a verified-healthy reading - a sensor stuck at a stable-but-corrupted value would
            # look identical to this check on every single restart, forever. Deliberately does
            # NOT authorize probing (ok=False) and does NOT persist - only an explicit human
            # confirmation (confirm_bootstrap_baseline()/PRTOUCH_CONFIRM_BASELINE) can promote
            # this to a real TRUSTED_REFERENCE. Overwrites any earlier candidate with the
            # latest consistent reading, so a human confirming always trusts fresh data.
            self._bootstrap_candidate = batch_mean
            return _reject(latest, (
                "reading is internally consistent, but no trusted reference exists yet to "
                "verify it against (first boot, or persisted reference file missing) - "
                "refusing to authorize a real probe attempt on an unconfirmed reading. Run "
                "PRTOUCH_CONFIRM_BASELINE after independently checking this is a genuine "
                "healthy reading to establish it as the trusted reference"), 'bootstrap_candidate')

        drift = 0.
        for i, key in enumerate(keys):
            if key in latest and i < len(self._auto_baseline):
                drift = max(drift, abs(latest[key] - self._auto_baseline[i]))
        if drift > self.sensor_baseline_max_drift:
            # Deliberately does NOT update self._auto_baseline or the persisted file below
            # this point - a stable-but-wrong reading must never overwrite a trusted
            # reference just because it agrees with itself (see __init__'s own comment on
            # this exact failure mode). Only readings that are ALREADY close to the
            # existing trusted baseline ever reach the update below - that's what makes it
            # safe to persist unconditionally once reached.
            return _reject(latest, (
                "reading is internally consistent but has drifted %.0f counts from the "
                "established TRUSTED_REFERENCE (allowed %.0f) - refusing to trust a "
                "stable-but-implausible reading, and not overwriting the trusted "
                "reference with it" % (drift, self.sensor_baseline_max_drift)), 'unstable')

        # Reached only for a batch that matches the existing TRUSTED_REFERENCE within
        # tolerance - safe to persist (this is normal drift tracking, not a new promotion).
        self._auto_baseline = batch_mean
        self._save_persisted_baseline(self._auto_baseline)
        self.last_diagnostic = {
            'ok': True, 'raw': latest, 'reason': None, 'state': 'healthy',
            'bootstrap': False,
            'tri_min_hold': self.tri_min_hold, 'tri_max_hold': self.tri_max_hold,
            'max_baseline_abs': self.max_baseline_abs,
        }
        return self.last_diagnostic

    def confirm_bootstrap_baseline(self):
        """BOOTSTRAP_CANDIDATE -> TRUSTED_REFERENCE (2026-08-12 root-cause mission). The one
        explicit, human-driven promotion path - see check_sensor_consistency()'s own comment
        on why this can't happen automatically: a sensor stuck at a stable-but-corrupted value
        would pass every automatic self-consistency check on every single restart, forever, so
        establishing genuine trust for the very first reference requires a human who has
        independently verified (e.g. inspected READ_PRES, knows the printer is genuinely idle/
        untouched) that the candidate is real. Raises PrtouchProbeSafetyError if there is no
        current candidate (never checked, or the last check was itself corrupted/unstable -
        checked_sensor_consistency() only ever sets a candidate from an internally-consistent
        batch). Returns the newly-trusted baseline values."""
        if self._bootstrap_candidate is None:
            raise PrtouchProbeSafetyError(
                "prtouch: no bootstrap candidate to confirm - run READ_PRES or "
                "check_sensor_consistency() first and confirm it reports "
                "state=bootstrap_candidate, not corrupted/unstable")
        self._auto_baseline = self._bootstrap_candidate
        self._bootstrap_candidate = None
        self._save_persisted_baseline(self._auto_baseline)
        logging.info("prtouch_probe: bootstrap candidate %s confirmed as TRUSTED_REFERENCE "
                     "by explicit operator command", self._auto_baseline)
        return self._auto_baseline

    def _check_baseline_safe(self):
        """Pre-motion guard - see check_sensor_consistency()'s own docstring for exactly what
        this does and does not prove. Raises PrtouchProbeSafetyError (never arms anything)
        rather than the ordinary _fail() path: nothing of THIS call has moved yet, so there is
        no motion of its own to recover from - unlike a real failed attempt, a fixed safety
        lift here would itself be an unrequested, unexplained move."""
        diag = self.check_sensor_consistency(base_cnt=8)
        if not diag['ok']:
            self.last_error = diag['reason']
            raise PrtouchProbeSafetyError(
                "prtouch: refusing to probe - %s (raw=%s)" % (diag['reason'], diag['raw']))
        return diag['raw']

    def touch_probe(self, down_min_z, retries=10, pro_cnt=3, tolerance=None):
        """run_step_prtouch-equivalent (ANALYSIS.md secs 3-4): send start_step+start_pres
        concurrently, collect both buffers, compute one Z sample via
        prtouch_calibration.compute_trigger_z(), lift back to the start height, and repeat
        until either two samples agree within tolerance or pro_cnt samples have been collected.
        Raises command_error (via _fail, with the safety-lift courtesy) after `retries` attempts
        without a usable sample - mirrors PR_NOT_TRIGGER/STEP_LOST/PRES_LOST (ANALYSIS.md sec 1)
        at a reduced surface, using plain command_error rather than Creality's PR_ERR_CODE_*
        catalog (DESIGN.md open question 3, resolved this way for the clean rewrite).

        Raises PrtouchProbeSafetyError, before ever arming a single command, if another raw
        PRTouch operation is already active (see _own_raw_operation), down_min_z exceeds
        max_probe_travel_mm, or the pre-motion baseline guard rejects the current sensor
        reading (see _check_baseline_safe).

        Suspends any active bed mesh for the duration of the probe cycle (matches the original):
        a loaded mesh applies a Z-compensation transform to every toolhead move, which would
        skew a raw touch-probe reading taken at a specific mesh-relative point.
        """
        with self._own_raw_operation('touch_probe') as op_id:
            if down_min_z > self.max_probe_travel_mm:
                self.last_error = (
                    "requested down_min_z=%.3fmm exceeds max_probe_travel_mm=%.3fmm"
                    % (down_min_z, self.max_probe_travel_mm))
                raise PrtouchProbeSafetyError("prtouch: " + self.last_error)
            self._check_baseline_safe()

            saved_mesh = self.bed_mesh.get_mesh() if self.bed_mesh is not None else None
            if saved_mesh is not None:
                self.bed_mesh.set_mesh(None)
            try:
                z = self._touch_probe(down_min_z, retries, pro_cnt, tolerance, op_id)
            finally:
                if saved_mesh is not None:
                    self.bed_mesh.set_mesh(saved_mesh)
            self.last_error = None
            return z

    def _touch_probe(self, down_min_z, retries, pro_cnt, tolerance, op_id=None):
        if tolerance is None:
            tolerance = self.probe_min_3err
        results = []
        attempt = 0
        deadline = self.reactor.monotonic() + self.max_probe_duration_s
        while len(results) < pro_cnt:
            if attempt >= retries:
                self._fail("touch_probe did not converge after %d attempts (results=%s)"
                            % (attempt, results), op_id)
            if self.reactor.monotonic() >= deadline:
                self._fail(
                    "touch_probe exceeded max_probe_duration_s=%.1f after %d attempts "
                    "(results=%s)" % (self.max_probe_duration_s, attempt, results), op_id)
            attempt += 1

            self.mcu.reset_buffers()
            diag = self.check_sensor_consistency(base_cnt=8)
            if not diag['ok']:
                reason = diag['reason']
                # A sensor that looked fine before touch_probe() started (the guard in
                # touch_probe() itself) but goes bad mid-sequence (e.g. a connector working
                # loose between attempts, or the PREVIOUS attempt's own raw step ops leaving
                # the pressure read degraded - see check_sensor_consistency's own docstring)
                # must not be allowed to keep probing on later attempts either - re-checked
                # every attempt, not just once up front.
                self._fail("baseline check failed on attempt %d/%d: %s"
                            % (attempt, retries, reason), op_id)
            step_cnt, step_us, acc_ctl_cnt = self.get_step_counts(down_min_z, self.tri_z_down_spd)
            start_pos_z = self.toolhead.get_position()[2]

            logging.info(
                "prtouch_probe: raw op #%s attempt %d/%d arm dir=0 step_cnt=%d step_us=%d "
                "acc_ctl_cnt=%d send_ms=%d", op_id, attempt, retries, step_cnt, step_us,
                acc_ctl_cnt, self.tri_send_ms)
            self.mcu.start_pres(0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                 self.tri_hftr_cut, self.tri_lftr_k1,
                                 self.tri_min_hold, self.tri_max_hold)
            self.mcu.start_step(0, step_cnt, step_us, acc_ctl_cnt, send_ms=self.tri_send_ms,
                                 low_spd_nul=self.low_spd_nul, send_step_duty=self.send_step_duty)
            timeout = units.probe_timeout_seconds(down_min_z, self.tri_z_down_spd)
            step_samples = self.mcu.collect_step_samples(timeout)
            pres_samples = self.mcu.collect_pres_samples(timeout)
            self.mcu.stop_step()
            self.mcu.start_pres(0, 0, 0, 0, 0, 0, 0, 0)
            logging.info("prtouch_probe: raw op #%s attempt %d/%d disarm dir=0", op_id, attempt,
                         retries)
            self._settle_after_disarm()

            if not step_samples or not pres_samples:
                logging.info("prtouch_probe: no trigger on attempt %d/%d", attempt, retries)
                # SAFETY (found 2026-08-06 via line-by-line comparison against the reference's
                # run_step_prtouch): on a genuine no-trigger, the MCU's own step callback
                # (prtouch_v2.c prtouch_event(), confirmed by now_steps counting DOWN from the
                # commanded total to 0) only stops early on a real trigger or when the full
                # commanded pulse train completes - so an empty/no-trigger result means the
                # *full* commanded step_cnt was actually executed, physically. Klipper's own
                # toolhead position tracking is never told about this raw MCU-driven motion (it
                # bypasses the normal trapq entirely), so self.toolhead.get_position()[2] still
                # reports the *pre-descent* height. Without lifting back here, the next loop
                # iteration recomputes another full down_min_z descent from that same stale
                # start_pos_z - i.e. `retries` consecutive full-depth blind descents stacked in
                # the same direction with nothing to stop them but the stepper stalling against
                # the bed. Always undo the full commanded descent via a raw step move using the
                # known-commanded step_cnt (not sample-derived, since step_samples is empty
                # here) before the next attempt, or before _fail() raises if retries are
                # exhausted (as of 2026-08-13, _fail() no longer lifts on its own - this
                # recovery is the only lift that runs, see _fail()'s own docstring).
                self._recover_after_no_trigger(step_cnt, op_id)
                continue

            try:
                z = prtouch_calibration.compute_trigger_z(
                    step_samples, pres_samples, self.mcu.step_tri_time, self.mcu.pres_tri_time,
                    self.mcu.pres_tri_chs, step_cnt, start_pos_z, self.mm_per_step,
                    self.mcu.use_adc, self.tri_acq_ms, self.cal_hftr_cut, self.cal_lftr_k1,
                    pres_cnt=self.mcu.pres_cnt)
            except ValueError as e:
                logging.info("prtouch_probe: %s on attempt %d/%d", e, attempt, retries)
                self._lift_after_down(step_cnt, step_samples, op_id)
                continue

            results.append(z)
            self._lift_after_down(step_cnt, step_samples, op_id)

            if len(results) >= 2 and (max(results) - min(results)) <= tolerance:
                break

        results.sort()
        n = len(results)
        return results[n // 2] if n % 2 == 1 else (results[n // 2 - 1] + results[n // 2]) / 2.

    def _lift_after_down(self, step_cnt, step_samples, op_id=None):
        """'step' is the MCU's own remaining-pulse countdown (reference/prtouch_v2.c: now_steps
        starts at the commanded total and decrements to 0 - confirmed directly from firmware
        source, not inferred from the host wrapper alone), so step_cnt - last_reported_step is
        the distance actually traveled by the time sampling stopped."""
        traveled = units.step_count_to_distance_mm(
            step_cnt - step_samples[-1]['step'], self.mm_per_step)
        if traveled < 0:
            # Should not happen with genuine firmware data (the last sample can never report
            # MORE remaining steps than were commanded) - if it does, something upstream is
            # corrupted/malformed, and silently treating it as "nothing to lift" (the pre-
            # existing `<= 0: return` behavior) would hide that. Still returns without lifting
            # (there is no sane "traveled" distance to act on from garbage data), but now says
            # so loudly instead of silently.
            logging.info(
                "prtouch_probe: _lift_after_down computed negative traveled=%.4fmm "
                "(step_cnt=%d, last reported step=%r) - treating as no-op, but this indicates "
                "malformed sample data, not a genuinely shallow probe", traveled, step_cnt,
                step_samples[-1]['step'])
            return
        if traveled == 0:
            return
        self._raw_lift(traveled, op_id)

    def _recover_after_no_trigger(self, step_cnt, op_id=None):
        """Undo a full, non-triggered commanded descent (see the safety comment at this
        method's call site in _touch_probe). Unlike _lift_after_down, there are no step_samples
        to derive an exact traveled distance from - a no-trigger response is empty by
        definition - so this uses the full commanded step_cnt directly, which is the correct,
        known distance given the firmware only stops early on a real trigger."""
        traveled = units.step_count_to_distance_mm(step_cnt, self.mm_per_step)
        if traveled <= 0:
            return
        self._raw_lift(traveled, op_id)

    def _raw_lift(self, traveled, op_id=None):
        """Shared by _lift_after_down/_recover_after_no_trigger. 2026-08-09: the trailing
        disarm now runs in a finally block, matching safe_move_z's own 2026-08-06 fix - a
        genuine buffer-repair failure (PrtouchProtocolError) during collect_step_samples()
        previously would have skipped the disarm entirely, leaving the step channel armed on
        exactly the recovery path meant to make a no-trigger/malformed-response failure safe.
        2026-08-10: disarm is now followed by _settle_after_disarm() - see module docstring.

        2026-08-13 (redundant-recovery-lift mission): if collect_step_samples() itself raises
        here, this recovery move did not provably complete - the physical position it was
        meant to restore is no longer known. _fail() no longer issues a courtesy lift of its
        own in that case (see its own docstring), so this is the one place that must react:
        latch _raw_channel_healthy False (refuses every future raw op via _own_raw_operation
        until a restart/re-home) instead of silently letting the caller treat this as an
        ordinary, recovered failure. The trailing disarm attempt still runs regardless (finally,
        unchanged) - even a channel we no longer trust should still get one disarm attempt
        rather than none."""
        up_cnt, up_us, up_acc = self.get_step_counts(traveled, self.tri_z_up_spd)
        if up_cnt == 0:
            return
        logging.info(
            "prtouch_probe: raw op #%s recovery arm dir=1 step_cnt=%d step_us=%d "
            "acc_ctl_cnt=%d send_ms=%d", op_id, up_cnt, up_us, up_acc, self.tri_send_ms)
        self.mcu.reset_buffers()
        self.mcu.start_step(1, up_cnt, up_us, up_acc, send_ms=self.tri_send_ms,
                             low_spd_nul=self.low_spd_nul, send_step_duty=self.send_step_duty)
        try:
            self.mcu.collect_step_samples(
                units.probe_timeout_seconds(traveled, self.tri_z_up_spd))
        except Exception:
            self._raw_channel_healthy = False
            logging.error(
                "prtouch_probe: raw op #%s recovery lift of %.4fmm did not complete cleanly - "
                "latching raw channel unhealthy (position no longer provably known)",
                op_id, traveled)
            raise
        finally:
            self.mcu.stop_step()
            logging.info("prtouch_probe: raw op #%s recovery disarm dir=1", op_id)
            self._settle_after_disarm()
