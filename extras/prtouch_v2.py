# prtouch_v2 - load-cell/pressure-probe touch detection, host-side Klipper extra
#
# Drop-in rewrite of Creality's compiled prtouch_v2_wrapper.so, using entirely standard Klipper
# host APIs (the same pattern hx711s.py/dirzctl.py already prove work on this exact device).
# Named/config-section-compatible with the existing [prtouch_v2] printer.cfg section on purpose -
# see ../DESIGN.md's "one real design decision" section. See ../ANALYSIS.md for the full protocol
# and algorithm this replaces.
#
# Deliberately NOT porting: run_G28_Z/run_G29_Z/bed_mesh_post_proc/run_re_g29s/
# correct_bed_mesh_data and their gcode entry points (CHECK_BED_MESH, ACCURATE_HOME_Z,
# PRTOUCH_READY) - confirmed dead code in real production, BLTouch owns homing/bed-mesh
# (ANALYSIS.md sec 7). Also not porting env_self_check/SELF_CHECK_PRTOUCH (only ever called from
# the dead run_G28_Z path, ANALYSIS.md sec 7/8) or most of the debug/diagnostic command set
# (TEST_PRTH, TRIG_TEST, TRIG_BED_TEST, TEST_SWAP) - cheap to add later for bring-up, not
# blocking the real feature.
#
# READ_PRES (2026-08-05, first real hardware bring-up pass): one diagnostic pulled forward early,
# specifically because it's the only command in this whole module that touches zero motion -
# deal_avgs_prtouch is a pure MCU sensor-channel read (see prtouch_mcu.py's deal_avgs()), no
# start_step_prtouch involved. Exists to let a first live check confirm the load-cell hardware is
# actually alive and producing plausible numbers relative to [prtouch_v2]/[z_compensate]'s real
# tri_min_hold/tri_max_hold thresholds, before ever risking a motion-based touch_probe() call.
#
# 2026-08-09 (load-cell safety hardening mission): READ_PRES now goes through
# PrtouchProbe.read_diagnostics() (same zero-motion deal_avgs_prtouch read as before) instead of
# calling deal_avgs directly, so its response includes the same plausibility verdict
# touch_probe() itself would apply before ever arming a real descent - a user checking this
# before running a real calibration sees exactly what the safety guard sees, not a lookalike.
# get_status() (below) exposes the same information through Klipper's own object-status/
# webhooks mechanism (automatically queryable/subscribable under this module's config section
# name, same pattern as z_compensate.py's own structured status - see its module comment for
# how that machinery works) for GuppyScreen/Moonraker to poll without needing to send gcode.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import prtouch_mcu
from . import prtouch_nozzle
from . import prtouch_probe


class PRTouchV2:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config

        self.mcu = prtouch_mcu.PrtouchMCU(config)
        self.probe = prtouch_probe.PrtouchProbe(config, self.mcu)
        self.heaters = None
        # Must be built here, not lazily inside clear_nozzle() - see prtouch_nozzle.py's
        # ClearNozzleConfig docstring.
        self.clear_nozzle_config = prtouch_nozzle.ClearNozzleConfig(config)

        self.hot_min_temp = config.getfloat('hot_min_temp', default=140, minval=80, maxval=200)
        self.hot_max_temp = config.getfloat('hot_max_temp', default=200, minval=180, maxval=300)
        self.bed_max_temp = config.getfloat('bed_max_temp', default=60, minval=45, maxval=100)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        self.gcode.register_command('NOZZLE_CLEAR', self.cmd_NOZZLE_CLEAR,
                                     desc=self.cmd_NOZZLE_CLEAR_help)
        self.gcode.register_command('SAFE_MOVE_Z', self.cmd_SAFE_MOVE_Z,
                                     desc=self.cmd_SAFE_MOVE_Z_help)
        self.gcode.register_command('READ_PRES', self.cmd_READ_PRES,
                                     desc=self.cmd_READ_PRES_help)
        self.gcode.register_command('PRTOUCH_CONFIRM_BASELINE',
                                     self.cmd_PRTOUCH_CONFIRM_BASELINE,
                                     desc=self.cmd_PRTOUCH_CONFIRM_BASELINE_help)
        self.gcode.register_command('PRTOUCH_TEST_TOUCH', self.cmd_PRTOUCH_TEST_TOUCH,
                                     desc=self.cmd_PRTOUCH_TEST_TOUCH_help)

    def _handle_connect(self):
        self.heaters = prtouch_nozzle.NozzleHeaters(self.printer)

    def _guarded(self, fn, *args, **kwargs):
        """2026-08-12 (live incident): PrtouchProbeSafetyError/PrtouchProtocolError are plain
        Exception subclasses, not self.printer.command_error - Klipper's own gcode.py dispatch
        loop (`except self.error as e:` / bare `except:`) only treats command_error specially as
        a clean, user-facing rejection; anything else falls into its generic "Internal error on
        command" handler, which calls printer.invoke_shutdown() - a FULL emergency_stop of every
        MCU, not a refusal of just this command. Confirmed live: the very first real
        PRTOUCH_TEST_TOUCH call on fresh hardware hit exactly this path (the fail-closed
        no-trusted-reference guard correctly refused to probe, via PrtouchProbeSafetyError) and
        took the whole printer down instead of just rejecting the command - no motion was ever
        armed (the guard raises before arming anything), but the safety guard's own success case
        was, ironically, indistinguishable from a genuine internal fault to Klipper's dispatcher.
        Every gcode entry point that can reach probe/mcu code goes through this wrapper so a
        correctly-functioning safety refusal is a clean command_error, not a shutdown."""
        try:
            return fn(*args, **kwargs)
        except (prtouch_probe.PrtouchProbeSafetyError, prtouch_mcu.PrtouchProtocolError) as e:
            raise self.printer.command_error(str(e))

    cmd_NOZZLE_CLEAR_help = "Wipe the nozzle using the load-cell touch probe"

    def cmd_NOZZLE_CLEAR(self, gcmd):
        hot_min_temp = gcmd.get_float('HOT_MIN_TEMP', self.hot_min_temp)
        hot_max_temp = gcmd.get_float('HOT_MAX_TEMP', self.hot_max_temp)
        bed_max_temp = gcmd.get_float('BED_MAX_TEMP', self.bed_max_temp)
        self._guarded(self.clear_nozzle, hot_min_temp, hot_max_temp, bed_max_temp)

    cmd_SAFE_MOVE_Z_help = "Raw non-probing Z move via the prtouch MCU step channel"

    def cmd_SAFE_MOVE_Z(self, gcmd):
        direction = gcmd.get_int('DIR', 1, minval=0, maxval=1)
        distance = gcmd.get_float('DIS', 10., above=0.)
        speed = gcmd.get_float('SPD', 5., above=0.)
        self._guarded(self.probe.safe_move_z, direction, distance, speed)

    cmd_READ_PRES_help = "Read raw load-cell sensor channels - no motion, diagnostic only"

    def cmd_READ_PRES(self, gcmd):
        base_cnt = gcmd.get_int('BASE_CNT', 8, minval=1, maxval=32)
        diag = self.probe.read_diagnostics(base_cnt=base_cnt)
        raw = diag['raw'] or {}
        gcmd.respond_info(
            "READ_PRES: ch0=%s ch1=%s ch2=%s ch3=%s (tri_min_hold=%d tri_max_hold=%d) "
            "ok=%s%s"
            % (raw.get('ch0'), raw.get('ch1'), raw.get('ch2'), raw.get('ch3'),
               self.probe.tri_min_hold, self.probe.tri_max_hold, diag['ok'],
               '' if diag['ok'] else (' reason=%s' % diag['reason'])))

    cmd_PRTOUCH_CONFIRM_BASELINE_help = (
        "Promote the current BOOTSTRAP_CANDIDATE sensor reading to a TRUSTED_REFERENCE - "
        "run this only after independently confirming (e.g. via READ_PRES) that the printer "
        "is genuinely idle and the reading is real, not corrupted")

    def cmd_PRTOUCH_CONFIRM_BASELINE(self, gcmd):
        values = self._guarded(self.probe.confirm_bootstrap_baseline)
        gcmd.respond_info(
            "PRTOUCH_CONFIRM_BASELINE: confirmed ch0=%.0f ch1=%.0f ch2=%.0f ch3=%.0f as the "
            "new TRUSTED_REFERENCE (persisted, survives restarts)" % tuple(values))

    cmd_PRTOUCH_TEST_TOUCH_help = (
        "Single bounded real touch_probe() attempt on the production lifecycle - retries=1, "
        "pro_cnt=1 (one attempt, no retry loop). Never applies a Z offset and never runs any "
        "calibration-result persistence - it only calls PrtouchProbe.touch_probe() and reports "
        "the resulting Z sample. Requires Z already homed. 2026-08-12 physical-qualification "
        "prep: the smallest existing production-compatible path that exercises one real, "
        "pressure-armed descent - see NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md sec 15/16.")

    def cmd_PRTOUCH_TEST_TOUCH(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        homed_axes = toolhead.get_status(self.printer.get_reactor().monotonic())['homed_axes']
        if 'z' not in homed_axes:
            raise gcmd.error("PRTOUCH_TEST_TOUCH: Z axis must already be homed")
        # DOWN_MIN_Z's maxval is a small, hard-coded test ceiling - independent of and well
        # below [prtouch_v2]'s own configured max_probe_travel_mm (default 50mm) - this command
        # exists specifically to keep the first real touch small, not to expose the full
        # configured travel range.
        down_min_z = gcmd.get_float('DOWN_MIN_Z', 3.0, above=0., maxval=5.0)
        z = self._guarded(self.probe.touch_probe, down_min_z, retries=1, pro_cnt=1)
        gcmd.respond_info(
            "PRTOUCH_TEST_TOUCH: single-attempt touch_probe result z=%.4fmm "
            "(down_min_z=%.2fmm) - no offset applied, nothing persisted beyond the normal "
            "sensor-baseline guard" % (z, down_min_z))

    def get_status(self, eventtime):
        """Zero-motion diagnostic status - see prtouch_probe.py's own last_diagnostic comment
        for why this returns a cached value rather than triggering a fresh MCU read on every
        poll. Safe to call at any time, including before klippy:connect (self.probe always
        exists once __init__ has run) and before any real reading has ever been taken (the
        cache starts with ok=None/reason='no reading taken yet', not a fabricated value)."""
        diag = self.probe.last_diagnostic
        return {
            'sensor_ok': diag['ok'],
            'sensor_reason': diag['reason'],
            'sensor_state': diag.get('state'),
            'sensor_has_trusted_reference': self.probe._auto_baseline is not None,
            'sensor_bootstrap_candidate_pending': self.probe._bootstrap_candidate is not None,
            'raw_channel_healthy': self.probe._raw_channel_healthy,
            'raw_op_active': self.probe._raw_op_active,
            'raw': diag['raw'],
            'tri_min_hold': diag['tri_min_hold'],
            'tri_max_hold': diag['tri_max_hold'],
            'max_baseline_abs': diag['max_baseline_abs'],
            'max_probe_travel_mm': self.probe.max_probe_travel_mm,
            'max_probe_duration_s': self.probe.max_probe_duration_s,
            'last_error': self.probe.last_error,
        }

    def touch_probe(self, down_min_z, **kwargs):
        """Public API for z_compensate.py (and anything else) to call into - thin passthrough
        to self.probe.touch_probe()."""
        return self.probe.touch_probe(down_min_z, **kwargs)

    def clear_nozzle(self, hot_min_temp, hot_max_temp, bed_max_temp):
        """Public API passthrough to prtouch_nozzle.clear_nozzle()."""
        toolhead = self.printer.lookup_object('toolhead')
        prtouch_nozzle.clear_nozzle(self.probe, toolhead, self.gcode, self.heaters,
                                     self.clear_nozzle_config, hot_min_temp, hot_max_temp,
                                     bed_max_temp)


def load_config(config):
    return PRTouchV2(config)
