# prtouch_v2 nozzle-wipe routine
#
# Clean-room rewrite of Creality's clear_nozzle() (prtouch_v2_wrapper.py, GPLv3, see
# reference/), read completely and traced in ../ANALYSIS.md sec 4. Not a verbatim port
# (ANALYSIS.md sec 6): drops the per-run velocity/accel override (set_step_par - a wipe-speed
# optimization, not a correctness requirement as long as clr_xy_spd/rdy_xy_spd stay under the
# printer's own configured max_velocity) and the out-of-range Z-reference-reset retry path
# (nozzle_clear_z_out_of_range - only matters if the wipe pad sits implausibly close to
# position_min, which would be a config error worth surfacing directly rather than silently
# working around).
#
# Wipe-drag geometry generalized to a 2D vector (pa_clr_dis_mm_x/y) rather than the single
# X-only pa_clr_dis_mm the reference wrapper reads - this printer's own real [z_compensate]
# section (pulled live via SSH 2026-08-05) has pa_clr_dis_mm_x: 0 / pa_clr_dis_mm_y: 30 against
# clr_noz_len_x: 3 / clr_noz_len_y: 50, i.e. its wipe pad is a narrow strip running along Y, the
# opposite orientation from the generic reference defaults (wide-X/narrow-Y) this file originally
# assumed. Setting pa_clr_dis_mm_y=0 exactly recovers the old X-only behavior, so this is a
# strict generalization, not a behavior change for any config that only sets the X component.
#
# Config reads live in ClearNozzleConfig, built once at __init__/load_config time by whichever
# module owns the config section - NOT inside clear_nozzle() itself. Confirmed live 2026-08-05:
# Klipper's configfile checks that every option present in a section was read at least once
# during the whole startup config-load pass, before any gcode ever runs - reading options lazily
# inside a gcode-command handler is too late and hard-errors at startup ("Option '...' is not
# valid in section '...'") the instant that section has any real value the __init__ path didn't
# already touch. This is why clr_noz_start_x etc. previously lived inside clear_nozzle()'s own
# body: worked fine offline (nothing ever calls it there), broke immediately on a real restart.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import random


class ClearNozzleConfig:
    """All config.get*() reads clear_nozzle() needs, resolved once by the owning module's own
    __init__ (PRTouchV2 for [prtouch_v2], ZCompensate for [z_compensate]) - see module docstring
    for why this can't happen lazily inside clear_nozzle() itself. Defaults are wide enough that
    [prtouch_v2]'s own real section (which has none of these keys at all) doesn't error - its own
    NOZZLE_CLEAR command stays effectively dead code (nothing in real production calls it) but no
    longer crashes the whole printer if it ever is."""

    def __init__(self, config):
        # clr_noz_start_x allows negative (this printer's real value is -3 - the wipe pad sits
        # partly off the near edge of the bed's own X origin, not a typo).
        self.clr_noz_start_x = config.getfloat('clr_noz_start_x', default=0.,
                                                 minval=-50, maxval=1000)
        self.clr_noz_start_y = config.getfloat('clr_noz_start_y', default=0.,
                                                 minval=0, maxval=1000)
        self.clr_noz_len_x = config.getfloat('clr_noz_len_x', default=1., minval=1)
        self.clr_noz_len_y = config.getfloat('clr_noz_len_y', default=1., minval=1)
        self.pa_clr_dis_mm_x = config.getfloat('pa_clr_dis_mm_x', default=30,
                                                minval=-100, maxval=100)
        self.pa_clr_dis_mm_y = config.getfloat('pa_clr_dis_mm_y', default=0,
                                                minval=-100, maxval=100)
        self.pa_clr_down_mm = config.getfloat('pa_clr_down_mm', default=-0.15,
                                               minval=-1, maxval=1)
        self.clr_xy_spd = config.getfloat('clr_xy_spd', default=2.0, minval=0.1)
        self.rdy_xy_spd = config.getfloat('rdy_xy_spd', default=200, minval=1)
        self.bed_max_err = config.getfloat('bed_max_err', default=5, minval=1)
        self.g29_down_min_z = config.getfloat('g29_down_min_z', default=25, minval=1)
        # vs_start_z_pos (real key): hover height before each wipe-pad touch probe. Falls back to
        # bed_max_err (the pre-existing dual-use default) when unset.
        self.hover_z = config.getfloat('vs_start_z_pos', default=self.bed_max_err)
        # pr_clear_probe_cnt (real key): probe-agreement count for these two wipe-pad touches,
        # distinct from Z_OFFSET_CALIBRATION's own pr_probe_cnt (read in z_compensate.py).
        self.pr_clear_probe_cnt = config.getint('pr_clear_probe_cnt', default=3, minval=1)


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


def clear_nozzle(probe, toolhead, gcode, heaters, params,
                  hot_min_temp, hot_max_temp, bed_max_temp, hot_end_temp=None):
    """clear_nozzle()-equivalent (ANALYSIS.md sec 4): heat bed/nozzle, probe two randomized XY
    points on the wipe pad via probe.touch_probe() to find local Z at each, drag the nozzle
    between them at wipe temp, then cool. `probe` is a PrtouchProbe (prtouch_probe.py); its own
    touch_probe() already suspends the active bed mesh for the duration of each probe. `params`
    is a ClearNozzleConfig, already resolved from config at __init__ time by the caller (see
    module docstring for why this can't be read lazily here).

    `hot_end_temp` (real config key, [z_compensate]-only - not read here as a bare default
    because [prtouch_v2]'s own real section never sets it): final nozzle temp to settle at once
    the wipe finishes, defaulting to hot_min_temp (the pre-existing behavior) when omitted."""
    clr_noz_start_x = params.clr_noz_start_x
    clr_noz_start_y = params.clr_noz_start_y
    clr_noz_len_x = params.clr_noz_len_x
    clr_noz_len_y = params.clr_noz_len_y
    pa_clr_dis_mm_x = params.pa_clr_dis_mm_x
    pa_clr_dis_mm_y = params.pa_clr_dis_mm_y
    pa_clr_down_mm = params.pa_clr_down_mm
    clr_xy_spd = params.clr_xy_spd
    rdy_xy_spd = params.rdy_xy_spd
    bed_max_err = params.bed_max_err
    g29_down_min_z = params.g29_down_min_z
    hover_z = params.hover_z
    pr_clear_probe_cnt = params.pr_clear_probe_cnt

    heaters.set_bed_temp(bed_max_temp, wait=False)
    heaters.set_hot_temp(hot_min_temp, wait=False)

    margin = 5
    avail_x = max(clr_noz_len_x - abs(pa_clr_dis_mm_x) - margin, 0)
    avail_y = max(clr_noz_len_y - abs(pa_clr_dis_mm_y) - margin, 0)
    src_x = clr_noz_start_x + random.uniform(0, avail_x)
    src_y = clr_noz_start_y + random.uniform(0, avail_y)
    src_pos = [src_x, src_y, hover_z]
    end_pos = [src_x + pa_clr_dis_mm_x, src_y + pa_clr_dis_mm_y, hover_z]

    heaters.set_hot_temp(hot_min_temp, wait=True)
    heaters.set_hot_temp(hot_min_temp + 40, wait=False)

    _move(gcode, toolhead, src_pos, rdy_xy_spd)
    src_pos[2] = probe.touch_probe(g29_down_min_z, retries=5, pro_cnt=pr_clear_probe_cnt)

    _move(gcode, toolhead, end_pos, rdy_xy_spd)
    end_pos[2] = probe.touch_probe(g29_down_min_z, retries=5, pro_cnt=pr_clear_probe_cnt)

    _move(gcode, toolhead, [src_pos[0], src_pos[1], hover_z], rdy_xy_spd)
    _move(gcode, toolhead, [src_pos[0], src_pos[1], src_pos[2] - pa_clr_down_mm],
          probe.tri_z_up_spd)
    heaters.set_hot_temp(hot_max_temp, wait=True)

    _move(gcode, toolhead, [end_pos[0], end_pos[1], end_pos[2] + pa_clr_down_mm], clr_xy_spd)
    heaters.set_hot_temp(hot_min_temp, wait=True)

    _move(gcode, toolhead,
          [end_pos[0] + pa_clr_dis_mm_x, end_pos[1] + pa_clr_dis_mm_y, end_pos[2] + bed_max_err],
          clr_xy_spd)
    heaters.set_hot_temp(hot_min_temp if hot_end_temp is None else hot_end_temp, wait=False)
    heaters.set_bed_temp(bed_max_temp, wait=True)
