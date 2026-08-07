# z_compensate - per-print auto-Z-offset orchestration, host-side Klipper extra
#
# NEW code, not a port - Creality's real z_compensate_wrapper.so has no published source anywhere
# (confirmed via GitHub org-wide search, ANALYSIS.md sec 5). Design is inferred from strong
# evidence (ANALYSIS.md sec 7): the real z_compensate_wrapper.so registers no MCU commands of its
# own, it does lookup_object('prtouch_v2') and calls straight into its primitives, and its
# bl_offset config value matches [bltouch]'s own y_offset exactly - consistent with probing at
# the physical point BLTouch already homed, offset by the nozzle-to-probe-tip distance, then
# reconciling the two readings. Named/config-section-compatible with the existing [z_compensate]
# printer.cfg section on purpose - see ../DESIGN.md.
#
# Real production calls this once per print, before BED_MESH_CALIBRATE (custom_macro.py's
# CX_PRINT_LEVELING_CALIBRATION, ANALYSIS.md sec 7) - a per-print thermal/wear fine-tune, not a
# one-time factory calibration. That matters for how the correction gets applied: baking it
# straight into the saved probe z_offset via stock Klipper's Z_OFFSET_APPLY_PROBE + SAVE_CONFIG
# would trigger a klippy restart in the middle of a print-start sequence, which is clearly wrong.
# So by default this only applies the correction as a live SET_GCODE_OFFSET for the current
# session/print - permanent persistence is opt-in (persist_offset) and needs a real, non-
# restarting save command for whichever environment this ends up running under (Creality's own
# stock image has a restart-free CXSAVE_CONFIG for exactly this reason; nothing has confirmed yet
# whether pellcorp's SimpleAF environment has an equivalent - flagged as a real open question,
# not silently assumed either way).
#
# Config-key reconciliation against the real device's live [z_compensate] section (pulled via
# SSH 2026-08-05, see project memory): this section has its OWN tri_min_hold/tri_max_hold
# (1400/2000, distinct from [prtouch_v2]'s own 1000/1500 defaults - a per-feature sensitivity
# retune, not a duplicate), its own probe speed, and the wipe-pad geometry keys
# (clr_noz_*/pa_clr_dis_mm_x/y/noz_pos_*/pumpback_mm/etc). Klipper's configfile errors on any
# option in a section that's never read via config.get*(), so all of these are read here even
# where their effect is genuinely unconfirmed (see "accepted but not wired" below) - pasting the
# real section in without this would make Klipper refuse to even start.
#
# Because CRTENSE_NOZZLE_CLEAR must use *this* section's tuning and wipe-geometry keys (not
# [prtouch_v2]'s), this module calls prtouch_nozzle.clear_nozzle() directly with its own `config`
# object rather than delegating to PRTouchV2.clear_nozzle() (which is bound to [prtouch_v2]'s
# config and has none of these keys) - matches DESIGN.md's original intent, not a new decision.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import contextlib
import math

from . import prtouch_nozzle

#: Structured status contract, version 1 - see docs/z_compensate_status_api.md. Consumed by
#: GuppyScreen's recalibration wizard via printer.objects.subscribe, replacing its previous
#: dependence on parsing this module's human-readable gcode response text (the "z_offset:"/
#: "PR_ERR_CODE" scan). get_status() below is the only part of this contract Klipper's own
#: object-status machinery requires - any object with a get_status(eventtime) method is
#: automatically queryable/subscribable under its config section name, no separate
#: registration or explicit "notify" call needed. Klipper's own subscription/webhooks layer
#: periodically polls get_status() and diffs it against the last-sent snapshot, pushing only
#: changed fields - so a transition this module makes between two polls (e.g. straight from
#: "running" to "complete" with nothing in between) is still delivered correctly, just as one
#: state or as a coalesced update; nothing here needs to force an intermediate flush.
_CALIBRATION_STATES = ('idle', 'running', 'complete', 'error')
_MAX_CALIBRATION_ERROR_LEN = 200


def _sanitize_calibration_error(exc):
    """Turns a raised exception into the calibration_error string: the same plain message
    Klipper's own command_error/config_error already carry (str() on a Klipper error never
    includes a traceback, object repr, or memory address - those only appear if something
    explicitly formats the traceback module's output, which this does not do), collapsed to
    one line and length-bounded so a UI status field can't be handed something pathological."""
    text = str(exc).strip()
    if not text:
        text = exc.__class__.__name__
    text = ' '.join(text.split())  # collapse embedded newlines/whitespace to one line
    if len(text) > _MAX_CALIBRATION_ERROR_LEN:
        text = text[:_MAX_CALIBRATION_ERROR_LEN - 3].rstrip() + '...'
    return text


class ZCompensate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        # Must be built here, not lazily inside cmd_nozzle_clear() - see prtouch_nozzle.py's
        # ClearNozzleConfig docstring (confirmed live 2026-08-05: a lazy config.get*() read
        # inside a gcode-command handler is too late, Klipper hard-errors at startup instead).
        self.clear_nozzle_config = prtouch_nozzle.ClearNozzleConfig(config)
        self.prtouch = None
        self.probe = None
        self.bed_mesh = None
        self.home_x = None
        self.home_y = None

        self.hot_start_temp = config.getfloat('hot_start_temp', default=140, minval=80, maxval=200)
        self.hot_rub_temp = config.getfloat('hot_rub_temp', default=180, minval=80, maxval=300)
        # hot_end_temp (real key): final nozzle temp after the wipe - threaded through to
        # prtouch_nozzle.clear_nozzle()'s hot_end_temp param.
        self.hot_end_temp = config.getfloat('hot_end_temp', default=None,
                                             minval=80, maxval=300)
        self.bed_add_temp = config.getfloat('bed_add_temp', default=0, minval=-20, maxval=100)
        bl_offset = config.getfloatlist('bl_offset', default=(0., 0.), count=2)
        self.bl_offset_x, self.bl_offset_y = bl_offset
        self.down_min_z = config.getfloat('z_offset_down_min_z', default=10, minval=1, maxval=50)
        # vs_start_z_pos (real key, replaces the old Klipper-only z_offset_hover_height default):
        # hover height before the Z_OFFSET_CALIBRATION touch, and passed through to
        # clear_nozzle() for its own two wipe-pad touches.
        self.hover_height = config.getfloat('vs_start_z_pos', default=5, minval=1, maxval=50)

        # tri_min_hold/tri_max_hold/speed (real keys): this section's own probe-sensitivity
        # tuning, applied as a temporary override of prtouch_v2's PrtouchProbe attributes for the
        # duration of calls this module makes (see _probe_overrides below) - distinct from
        # [prtouch_v2]'s own defaults, not a duplicate of them.
        self.tri_min_hold = config.getint('tri_min_hold', default=None)
        self.tri_max_hold = config.getint('tri_max_hold', default=None)
        self.probe_speed = config.getfloat('speed', default=None, minval=0.1)
        # tri_expand_mm (real key): no source exists for z_compensate_wrapper.so (see module
        # docstring), so this is an inference, not a confirmed port - applied as a fixed additive
        # correction to the Z_OFFSET_CALIBRATION measurement, on the theory that it accounts for
        # a systematic gap between the load cell's electrical trigger point and true mechanical
        # nozzle-bed contact (compliance in the load-cell mount). Defaults to 0 (no correction)
        # so an unset/misread value can't silently bias every print's Z offset.
        self.tri_expand_mm = config.getfloat('tri_expand_mm', default=0.)
        # pr_probe_cnt (real key): probe-agreement count for this command's own touch, distinct
        # from clear_nozzle()'s own pr_clear_probe_cnt (read in prtouch_nozzle.py).
        self.pr_probe_cnt = config.getint('pr_probe_cnt', default=3, minval=1)

        # Accepted (so Klipper doesn't reject the real config section) but deliberately left
        # unwired - no reference source and no confirmed evidence of their effect, and each
        # would need to be right on the first live attempt: type_nozz (nozzle-type selector,
        # meaning unknown), noz_pos_center/noz_pos_offset (plausibly another XY reference point
        # but nothing confirms it - misapplying it would send the toolhead somewhere unintended),
        # pumpback_mm (plausibly a pre-wipe retract, but unconfirmed - wiring untested E-axis
        # motion into a probe/heat sequence isn't worth the risk on a guess). Flag if any of
        # these turn out to matter after real testing - same "flagged, not silently omitted"
        # pattern as Z_OFFSET_AUTO/env_self_check elsewhere in this file/DESIGN.md.
        self.type_nozz = config.getint('type_nozz', default=0)
        self.noz_pos_center = config.getfloatlist('noz_pos_center', default=(0., 0.), count=2)
        self.noz_pos_offset = config.getfloatlist('noz_pos_offset', default=(0., 0.), count=2)
        self.pumpback_mm = config.getfloat('pumpback_mm', default=0.)

        # Opt-in permanent persistence (see module docstring) - off by default.
        self.persist_offset = config.getboolean('persist_offset', default=False)
        self.save_config_command = config.get('save_config_command', default='SAVE_CONFIG')

        # Structured status contract v1 (see module-level comment + docs/
        # z_compensate_status_api.md) - independent of persist_offset above, which stays a
        # console/config concern; GuppyScreen's own persistence step consumes
        # calibration_z_offset directly and does its own save, regardless of this setting.
        self.calibration_id = 0
        self.calibration_state = "idle"
        self.calibration_z_offset = None
        self.calibration_error = None

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        self.gcode.register_command('CRTENSE_NOZZLE_CLEAR', self.cmd_nozzle_clear,
                                     desc=self.cmd_nozzle_clear_help)
        self.gcode.register_command('Z_OFFSET_CALIBRATION', self.cmd_z_offset_calibration,
                                     desc=self.cmd_z_offset_calibration_help)
        # Z_OFFSET_AUTO: registered by the real z_compensate_wrapper.so but never actually
        # called by any macro on this printer (DESIGN.md open question 2, resolved: skip for
        # v1) - not registering unless something turns out to need it.

    def _handle_connect(self):
        self.prtouch = self.printer.lookup_object('prtouch_v2')
        self.probe = self.printer.lookup_object('probe')
        self.bed_mesh = self.printer.lookup_object('bed_mesh')
        min_x, min_y = self.bed_mesh.bmc.mesh_min
        max_x, max_y = self.bed_mesh.bmc.mesh_max
        self.home_x = min_x + (max_x - min_x) / 2.
        self.home_y = min_y + (max_y - min_y) / 2.

    def get_status(self, eventtime):
        """Structured status contract v1 - see the module-level comment and
        docs/z_compensate_status_api.md. Deliberately does no work, no hardware access, and
        no state mutation: a fresh dict of the four already-computed fields, safe to call at
        any time (including before klippy:connect) and safe for a caller to mutate without
        touching this object's own state."""
        return {
            "calibration_id": self.calibration_id,
            "calibration_state": self.calibration_state,
            "calibration_z_offset": self.calibration_z_offset,
            "calibration_error": self.calibration_error,
        }

    @contextlib.contextmanager
    def _probe_overrides(self):
        """Temporarily apply this section's own tri_min_hold/tri_max_hold/speed onto
        prtouch_v2's shared PrtouchProbe instance for the duration of one call, restoring
        afterward - so this command's real, separately-tuned sensitivity doesn't leak into
        [prtouch_v2]'s own NOZZLE_CLEAR/other future callers."""
        probe = self.prtouch.probe
        overrides = {
            'tri_min_hold': self.tri_min_hold,
            'tri_max_hold': self.tri_max_hold,
            'tri_z_down_spd': self.probe_speed,
        }
        saved = {}
        try:
            for attr, val in overrides.items():
                if val is None:
                    continue
                saved[attr] = getattr(probe, attr)
                setattr(probe, attr, val)
            yield probe
        finally:
            for attr, val in saved.items():
                setattr(probe, attr, val)

    cmd_nozzle_clear_help = "Wipe the nozzle before Z-offset calibration"

    def cmd_nozzle_clear(self, gcmd):
        """Reads HOT_START_TEMP/HOT_RUB_TEMP/HOT_END_TEMP/BED_ADDTEMP params - matches the real
        call site in custom_macro.py's CX_PRINT_LEVELING_CALIBRATION exactly. Calls
        prtouch_nozzle.clear_nozzle() directly with this section's own config (see module
        docstring) - NOT PRTouchV2.clear_nozzle(), which is bound to [prtouch_v2]'s config."""
        hot_start_temp = gcmd.get_float('HOT_START_TEMP', self.hot_start_temp)
        hot_rub_temp = gcmd.get_float('HOT_RUB_TEMP', self.hot_rub_temp)
        hot_end_temp = gcmd.get_float('HOT_END_TEMP', self.hot_end_temp)
        bed_add_temp = gcmd.get_float('BED_ADDTEMP', self.bed_add_temp)
        heater_bed = self.printer.lookup_object('heater_bed')
        bed_target = heater_bed.get_status(self.printer.get_reactor().monotonic())['target']
        toolhead = self.printer.lookup_object('toolhead')
        with self._probe_overrides() as probe:
            prtouch_nozzle.clear_nozzle(
                probe, toolhead, self.gcode, self.prtouch.heaters, self.clear_nozzle_config,
                hot_start_temp, hot_rub_temp, bed_target + bed_add_temp,
                hot_end_temp=hot_end_temp)

    cmd_z_offset_calibration_help = "Auto-tune Z offset via the load-cell nozzle touch"

    def cmd_z_offset_calibration(self, gcmd):
        """Touch-probe at the point BLTouch already homed (bed center, adjusted by bl_offset -
        the nozzle-to-probe-tip distance) via prtouch_v2.touch_probe(), then apply the result as
        a live Z gcode-offset for this print (see module docstring for why not a permanent
        z_offset rewrite by default).

        The math: touch_probe() returns the toolhead Z position (in the current coordinate
        frame, where Z=0 is BLTouch's calibrated bed surface) at which the *nozzle* contacts the
        bed. If BLTouch's static z_offset were perfectly accurate this would read exactly 0; any
        deviation is precisely the correction needed - the same 'offset' value stock Klipper's
        own Z_OFFSET_APPLY_PROBE reads from SET_GCODE_OFFSET and subtracts from the probe's
        z_offset (verified against pellcorp/klipper's probe.py: new_calibrate = z_offset -
        offset), so setting a gcode offset equal to the raw measurement is the correct
        equivalent of "reconciling the two readings" (ANALYSIS.md sec 7) without needing to read
        the probe's current z_offset directly. tri_expand_mm (see __init__) is then applied as a
        fixed additive correction on top - see its own comment for the caveat.
        """
        # Structured status: a new attempt always gets a new id and clears any previous
        # result before doing anything else - a caller polling get_status() must never see a
        # stale "complete"/offset left over from an earlier invocation once a new one has
        # started (see docs/z_compensate_status_api.md's ID-correlation section).
        self.calibration_id += 1
        self.calibration_state = "running"
        self.calibration_z_offset = None
        self.calibration_error = None

        try:
            toolhead = self.printer.lookup_object('toolhead')
            cur_pos = toolhead.get_position()
            target = [self.home_x + self.bl_offset_x, self.home_y + self.bl_offset_y,
                      self.hover_height, cur_pos[3]]
            self.gcode.run_script_from_command(
                'G1 F%d X%.3f Y%.3f Z%.3f' % (200 * 60, target[0], target[1], target[2]))
            toolhead.wait_moves()

            with self._probe_overrides():
                measured_z = self.prtouch.touch_probe(self.down_min_z, pro_cnt=self.pr_probe_cnt)
            measured_z += self.tri_expand_mm

            # A NaN/inf measurement can only mean a bug in the calibration math upstream
            # (prtouch_calibration.py's own interpolation is unconditionally finite for
            # finite inputs - see its own test suite) - surfacing it as a clear command_error
            # here, before it's ever applied as a live offset or published as a completed
            # status, is strictly better than silently accepting a broken value.
            if not math.isfinite(measured_z):
                raise self.printer.command_error(
                    "Z_OFFSET_CALIBRATION: measured value %r is not a finite number"
                    % (measured_z,))

            self.gcode.run_script_from_command('SET_GCODE_OFFSET Z=%.5f MOVE=0' % measured_z)
        except Exception as e:
            self.calibration_state = "error"
            self.calibration_z_offset = None
            self.calibration_error = _sanitize_calibration_error(e)
            raise

        # The calibration itself has now genuinely succeeded - a real measurement was taken
        # and applied as this print's live Z offset. Publish "complete" here, before the
        # optional persist_offset block below, deliberately: GuppyScreen's own persistence
        # step consumes calibration_z_offset directly and does its own save/restart,
        # entirely independent of persist_offset (which stays a separate, opt-in,
        # console/config-file concern - see this module's own docstring on why a restart
        # here would be wrong for the common per-print case). If persist_offset's own extra
        # steps below fail, that failure still propagates as a normal command_error (existing
        # behavior, unchanged) but does not retroactively invalidate a result that was
        # already correct and already applied.
        self.calibration_state = "complete"
        self.calibration_z_offset = measured_z
        self.calibration_error = None

        gcmd.respond_info(
            "Z_OFFSET_CALIBRATION: measured %.5f mm, applied as this print's Z offset"
            % measured_z)

        if self.persist_offset:
            self.gcode.run_script_from_command('Z_OFFSET_APPLY_PROBE')
            self.gcode.run_script_from_command(self.save_config_command)


def load_config(config):
    return ZCompensate(config)
