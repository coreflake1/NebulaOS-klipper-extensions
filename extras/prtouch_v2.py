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

    def _handle_connect(self):
        self.heaters = prtouch_nozzle.NozzleHeaters(self.printer)

    cmd_NOZZLE_CLEAR_help = "Wipe the nozzle using the load-cell touch probe"

    def cmd_NOZZLE_CLEAR(self, gcmd):
        hot_min_temp = gcmd.get_float('HOT_MIN_TEMP', self.hot_min_temp)
        hot_max_temp = gcmd.get_float('HOT_MAX_TEMP', self.hot_max_temp)
        bed_max_temp = gcmd.get_float('BED_MAX_TEMP', self.bed_max_temp)
        self.clear_nozzle(hot_min_temp, hot_max_temp, bed_max_temp)

    cmd_SAFE_MOVE_Z_help = "Raw non-probing Z move via the prtouch MCU step channel"

    def cmd_SAFE_MOVE_Z(self, gcmd):
        direction = gcmd.get_int('DIR', 1, minval=0, maxval=1)
        distance = gcmd.get_float('DIS', 10., above=0.)
        speed = gcmd.get_float('SPD', 5., above=0.)
        self.probe.safe_move_z(direction, distance, speed)

    cmd_READ_PRES_help = "Read raw load-cell sensor channels - no motion, diagnostic only"

    def cmd_READ_PRES(self, gcmd):
        base_cnt = gcmd.get_int('BASE_CNT', 8, minval=1, maxval=32)
        result = self.mcu.deal_avgs(base_cnt=base_cnt)
        gcmd.respond_info(
            "READ_PRES: ch0=%d ch1=%d ch2=%d ch3=%d (tri_min_hold=%d tri_max_hold=%d)"
            % (result['ch0'], result['ch1'], result['ch2'], result['ch3'],
               self.probe.tri_min_hold, self.probe.tri_max_hold))

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
