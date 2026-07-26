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
# This file may be distributed under the terms of the GNU GPLv3 license.


class ZCompensate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.prtouch = None
        self.probe = None
        self.bed_mesh = None
        self.home_x = None
        self.home_y = None

        self.hot_start_temp = config.getfloat('hot_start_temp', default=140, minval=80, maxval=200)
        self.hot_rub_temp = config.getfloat('hot_rub_temp', default=180, minval=80, maxval=300)
        self.bed_add_temp = config.getfloat('bed_add_temp', default=0, minval=-20, maxval=20)
        bl_offset = config.getfloatlist('bl_offset', default=(0., 0.), count=2)
        self.bl_offset_x, self.bl_offset_y = bl_offset
        self.down_min_z = config.getfloat('z_offset_down_min_z', default=10, minval=1, maxval=50)
        self.hover_height = config.getfloat('z_offset_hover_height', default=5, minval=1, maxval=50)

        # Opt-in permanent persistence (see module docstring) - off by default.
        self.persist_offset = config.getboolean('persist_offset', default=False)
        self.save_config_command = config.get('save_config_command', default='SAVE_CONFIG')

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

    cmd_nozzle_clear_help = "Wipe the nozzle before Z-offset calibration"

    def cmd_nozzle_clear(self, gcmd):
        """Reads HOT_START_TEMP/HOT_RUB_TEMP/BED_ADDTEMP params - matches the real call site
        in custom_macro.py's CX_PRINT_LEVELING_CALIBRATION exactly. Delegates to
        prtouch_v2's clear_nozzle(), same underlying routine NOZZLE_CLEAR uses."""
        hot_start_temp = gcmd.get_float('HOT_START_TEMP', self.hot_start_temp)
        hot_rub_temp = gcmd.get_float('HOT_RUB_TEMP', self.hot_rub_temp)
        bed_add_temp = gcmd.get_float('BED_ADDTEMP', self.bed_add_temp)
        heater_bed = self.printer.lookup_object('heater_bed')
        bed_target = heater_bed.get_status(self.printer.get_reactor().monotonic())['target']
        self.prtouch.clear_nozzle(hot_start_temp, hot_rub_temp, bed_target + bed_add_temp)

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
        the probe's current z_offset directly.
        """
        toolhead = self.printer.lookup_object('toolhead')
        cur_pos = toolhead.get_position()
        target = [self.home_x + self.bl_offset_x, self.home_y + self.bl_offset_y,
                  self.hover_height, cur_pos[3]]
        self.gcode.run_script_from_command(
            'G1 F%d X%.3f Y%.3f Z%.3f' % (200 * 60, target[0], target[1], target[2]))
        toolhead.wait_moves()

        measured_z = self.prtouch.touch_probe(self.down_min_z)

        self.gcode.run_script_from_command('SET_GCODE_OFFSET Z=%.5f MOVE=0' % measured_z)
        gcmd.respond_info(
            "Z_OFFSET_CALIBRATION: measured %.5f mm, applied as this print's Z offset"
            % measured_z)

        if self.persist_offset:
            self.gcode.run_script_from_command('Z_OFFSET_APPLY_PROBE')
            self.gcode.run_script_from_command(self.save_config_command)


def load_config(config):
    return ZCompensate(config)
