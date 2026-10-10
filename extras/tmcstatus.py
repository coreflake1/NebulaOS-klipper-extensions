# TMC stepper-driver status reporting for GuppyScreen's TMC panel
#
# Copyright (C) 2024  ballaswag <https://github.com/ballaswag>
# Copyright (C) 2026  NebulaOS contributors (klippy:connect deferral,
#     cached-status reactor-safety fix, see below)
#
# Originally from ballaswag/guppyscreen, k1/k1_mods/tmcstatus.py, first published in commit
# 1d7e584 ("Add tmc metrics graphs...", 2024-02-01). See VENDORED.md.
#
# NebulaOS deltas beyond this header:
#
#   1. handle_connect() is registered on "klippy:connect" instead of being called directly
#      from __init__. The original's direct call depends on printer.cfg section order.
#
#   2. (2026-08, reactor crash fix) get_status() no longer performs synchronous UART/SPI
#      register reads. The original called mcu_tmc.get_register() inside get_status(), which
#      invokes reactor.pause() to wait for the MCU response. When a webhook/status client
#      queries objects during shutdown or from a timer callback where reactor.pause() is
#      disabled, this produced:
#
#          tmcstatus: skipping tmc2208 stepper_x: Internal error - reactor pause disabled
#
#      for each configured driver, followed by a cascading AttributeError
#      ('NoneType' object has no attribute 'timer_is_running') that crashed Klipper.
#
#      Fix: a reactor timer refreshes TMC registers periodically in a safe context, and
#      get_status() returns only the cached data. No reactor interaction on the status path.
#
#   3. (2026-10, GUI readiness) TMCSTATUS ENABLE=0|1 pauses/resumes the refresh at runtime
#      (status becomes {} while paused) and `enabled:` sets the start state (default True).
#      GuppyScreen's TMC panel toggle used _GUPPY_LOAD_MODULE / _GUPPY_UNLOAD_MODULE, which
#      rewrite printer.cfg through a shell macro NebulaOS does not ship.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"]
REFRESH_INTERVAL = 1.0

class TMCStatus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.configured_steppers = []
        self.sense_resistor = {}
        self.tmcs = {}
        self._cache = {}
        self._refresh_timer = None
        self._shutdown = False
        self._connected = False
        self.enabled = config.getboolean('enabled', True)

        for driver in TRINAMIC_DRIVERS:
            for n in config.get_prefix_sections(driver):
                name = n.get_name()
                self.configured_steppers.append(name)
                self.sense_resistor[name] = n.getfloat('sense_resistor', None)

        self.printer.register_event_handler("klippy:connect",
                                             self.handle_connect)
        self.printer.register_event_handler("klippy:shutdown",
                                             self._handle_shutdown)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('TMCSTATUS', self.cmd_TMCSTATUS,
                               desc=self.cmd_TMCSTATUS_help)

    def handle_connect(self):
        for s in self.configured_steppers:
            self.tmcs[s] = self.printer.lookup_object(s)
        self._connected = True
        if self.enabled:
            self._start()

    def _start(self):
        if self._refresh_timer is not None or self._shutdown:
            return
        self._refresh_cache()
        self._refresh_timer = self.reactor.register_timer(
            self._timer_refresh, self.reactor.monotonic() + REFRESH_INTERVAL)

    def _stop(self):
        if self._refresh_timer is not None:
            self.reactor.unregister_timer(self._refresh_timer)
            self._refresh_timer = None
        self._cache = {}

    cmd_TMCSTATUS_help = ("Pause (ENABLE=0) or resume (ENABLE=1) the periodic TMC "
                          "driver status refresh; without ENABLE reports the state")

    def cmd_TMCSTATUS(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        if enable is not None:
            self.enabled = bool(enable)
            if not self._connected:
                pass
            elif self.enabled:
                self._start()
            else:
                self._stop()
        gcmd.respond_info("tmcstatus: %s (%d drivers)"
                          % ("enabled" if self.enabled else "paused",
                             len(self.configured_steppers)))

    def _handle_shutdown(self):
        self._shutdown = True
        if self._refresh_timer is not None:
            self.reactor.unregister_timer(self._refresh_timer)
            self._refresh_timer = None

    def _timer_refresh(self, eventtime):
        if self._shutdown:
            return self.reactor.NEVER
        self._refresh_cache()
        return eventtime + REFRESH_INTERVAL

    def _refresh_cache(self):
        for tmc, tmcobj in self.tmcs.items():
            try:
                self._cache[tmc] = self._collect(tmc, tmcobj)
            except Exception as e:
                logging.info("tmcstatus: refresh skipped %s: %s", tmc, e)

    def get_status(self, eventtime):
        return dict(self._cache)

    def _collect(self, tmc, tmcobj):
        fobj = tmcobj.fields

        def gf(field):
            if fobj.lookup_register(field, None) is not None:
                return fobj.get_field(field)
            return None

        drv_status_val = tmcobj.mcu_tmc.get_register('DRV_STATUS')
        fields = fobj.get_reg_fields('DRV_STATUS', drv_status_val)
        drv_fields = {n: v for n, v in fields.items() if v}
        tmc_data = {
            'drv_status': drv_fields,
            'hstrt': gf('hstrt'),
            'hend': gf('hend'),
            'pwm_autoscale': gf('pwm_autoscale'),
            'pwm_autograd': gf('pwm_autograd'),
            'pwm_grad': gf('pwm_grad'),
            'pwm_ofs': gf('pwm_ofs'),
            'pwm_reg': gf('pwm_reg'),
            'pwm_lim': gf('pwm_lim'),
            'tpwmthrs': gf('tpwmthrs'),
            'en_spreadcycle': gf('en_spreadcycle'),
            'tbl': gf('tbl'),
            'toff': gf('toff'),
            'tcoolthrs': gf('tcoolthrs'),
            'semin': gf('semin'),
            'semax': gf('semax'),
            'seup': gf('seup'),
            'sedn': gf('sedn'),
            'seimin': gf('seimin'),
        }

        if fobj.lookup_register('sg_result', None) is not None:
            tmc_data['sg_result'] = tmcobj.mcu_tmc.get_register('SG_RESULT')

        if 'cs_actual' in drv_fields:
            irms = self._cs_to_rms(drv_fields['cs_actual'], tmc, tmcobj)
            if irms is not None:
                tmc_data['i_rms'] = irms

        if fobj.lookup_register('en_pwm_mode', None) is not None:
            tmc_data['en_pwm_mode'] = fobj.get_field('en_pwm_mode')

        return tmc_data

    def _cs_to_rms(self, cs, tmc, tmcobj):
        rsense = self.sense_resistor.get(tmc)
        if rsense is None:
            return None
        vsense = tmcobj.fields.get_field('vsense')
        return (cs+1)/32.0 * (0.180 if vsense == 1 else 0.325)/(rsense+0.02) / 1.41421 * 1000

def load_config(config):
    return TMCStatus(config)
