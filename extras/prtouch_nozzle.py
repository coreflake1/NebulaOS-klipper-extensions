# prtouch_v2 nozzle-wipe routine
#
# Clean-room rewrite of Creality's clear_nozzle() (prtouch_v2_wrapper.py, GPLv3, see
# reference/), read completely and traced in ../ANALYSIS.md sec 4. Not a verbatim port
# (ANALYSIS.md sec 6): drops the per-run velocity/accel override (set_step_par - a wipe-speed
# optimization, not a correctness requirement as long as clr_xy_spd/rdy_xy_spd stay under the
# printer's own configured max_velocity) and the out-of-range Z-reference-reset retry path
# (nozzle_clear_z_out_of_range - only matters if the wipe pad sits implausibly close to
# position_min, which would be a config error worth surfacing directly rather than silently
# working around). Config keys (clr_noz_start_x/y, clr_noz_len_x/y, pa_clr_dis_mm, pa_clr_down_mm,
# clr_xy_spd) match the original's [z_compensate] section exactly, so this is a drop-in
# replacement with zero printer.cfg edits.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import random


class NozzleHeaters:
    """Thin wrapper around Klipper's own heater objects for wait-for-temp semantics
    (set_hot_temps/set_bed_temps-equivalent, ANALYSIS.md sec 4). Built once at connect time and
    shared across every clear_nozzle() call - no global mutable state of its own."""

    def __init__(self, printer):
        self.reactor = printer.get_reactor()
        self.pheaters = printer.lookup_object('heaters')
        self.extruder_heater = printer.lookup_object('extruder').heater
        self.bed_heater = printer.lookup_object('heater_bed').heater

    def set_hot_temp(self, temp, wait=False, tolerance=5.0):
        self.pheaters.set_temperature(self.extruder_heater, temp, False)
        if not wait:
            return
        eventtime = self.reactor.monotonic()
        while (self.extruder_heater.target_temp > 0
               and abs(self.extruder_heater.target_temp
                       - self.extruder_heater.smoothed_temp) > tolerance):
            eventtime = self.reactor.pause(eventtime + 0.1)

    def set_bed_temp(self, temp, wait=False, tolerance=5.0):
        self.pheaters.set_temperature(self.bed_heater, temp, False)
        if not wait:
            return
        eventtime = self.reactor.monotonic()
        while (self.bed_heater.target_temp > 0
               and abs(self.bed_heater.target_temp - self.bed_heater.smoothed_temp) > tolerance):
            eventtime = self.reactor.pause(eventtime + 0.1)


def _move(gcode, toolhead, pos, speed):
    gcode.run_script_from_command(
        'G1 F%d X%.3f Y%.3f Z%.3f' % (speed * 60, pos[0], pos[1], pos[2]))
    toolhead.wait_moves()


def clear_nozzle(probe, toolhead, gcode, heaters, config,
                  hot_min_temp, hot_max_temp, bed_max_temp):
    """clear_nozzle()-equivalent (ANALYSIS.md sec 4): heat bed/nozzle, probe two randomized XY
    points on the wipe pad via probe.touch_probe() to find local Z at each, drag the nozzle
    between them at wipe temp, then cool. `probe` is a PrtouchProbe (prtouch_probe.py); its own
    touch_probe() already suspends the active bed mesh for the duration of each probe."""
    clr_noz_start_x = config.getfloat('clr_noz_start_x', minval=0)
    clr_noz_start_y = config.getfloat('clr_noz_start_y', minval=0)
    clr_noz_len_x = config.getfloat('clr_noz_len_x', minval=1)
    clr_noz_len_y = config.getfloat('clr_noz_len_y', minval=1)
    pa_clr_dis_mm = config.getfloat('pa_clr_dis_mm', default=30, minval=2)
    pa_clr_down_mm = config.getfloat('pa_clr_down_mm', default=-0.15, minval=-1, maxval=1)
    clr_xy_spd = config.getfloat('clr_xy_spd', default=2.0, minval=0.1)
    rdy_xy_spd = config.getfloat('rdy_xy_spd', default=200, minval=1)
    bed_max_err = config.getfloat('bed_max_err', default=5, minval=1)
    g29_down_min_z = config.getfloat('g29_down_min_z', default=25, minval=1)

    heaters.set_bed_temp(bed_max_temp, wait=False)
    heaters.set_hot_temp(hot_min_temp, wait=False)

    src_x = clr_noz_start_x + random.uniform(0, clr_noz_len_x - pa_clr_dis_mm - 5)
    src_y = clr_noz_start_y + random.uniform(0, clr_noz_len_y)
    src_pos = [src_x, src_y, bed_max_err]
    end_pos = [src_x + pa_clr_dis_mm, src_y, bed_max_err]

    heaters.set_hot_temp(hot_min_temp, wait=True)
    heaters.set_hot_temp(hot_min_temp + 40, wait=False)

    _move(gcode, toolhead, src_pos, rdy_xy_spd)
    src_pos[2] = probe.touch_probe(g29_down_min_z, retries=5, pro_cnt=3)

    _move(gcode, toolhead, end_pos, rdy_xy_spd)
    end_pos[2] = probe.touch_probe(g29_down_min_z, retries=5, pro_cnt=3)

    _move(gcode, toolhead, [src_pos[0], src_pos[1], bed_max_err], rdy_xy_spd)
    _move(gcode, toolhead, [src_pos[0], src_pos[1], src_pos[2] - pa_clr_down_mm],
          probe.tri_z_up_spd)
    heaters.set_hot_temp(hot_max_temp, wait=True)

    _move(gcode, toolhead, [end_pos[0], end_pos[1], end_pos[2] + pa_clr_down_mm], clr_xy_spd)
    heaters.set_hot_temp(hot_min_temp, wait=True)

    _move(gcode, toolhead, [end_pos[0] + pa_clr_dis_mm, end_pos[1], end_pos[2] + bed_max_err],
          clr_xy_spd)
    heaters.set_bed_temp(bed_max_temp, wait=True)
