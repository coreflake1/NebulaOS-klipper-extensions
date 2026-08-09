# prtouch_v2 MCU protocol - oid/config setup, raw command send, response buffering
#
# Clean-room rewrite of the MCU-facing half of Creality's prtouch_v2_wrapper.py (GPLv3, see
# reference/) against the *existing, unreflashed* toolhead firmware - same wire protocol, same
# standard Klipper host APIs (create_oid/add_config_cmd/lookup_command/register_response) already
# proven on this device by hx711s.py. See ../ANALYSIS.md secs 1-2 for the full protocol trace this
# is built from and ../DESIGN.md for how this file fits the six-file layout.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import prtouch_units as units

MAX_BUF_LEN = 32
MAX_PRES_CNT = 4
POLL_INTERVAL = 0.010


class PrtouchProtocolError(Exception):
    pass


class PrtouchMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        ppins = self.printer.lookup_object('pins')

        self.use_adc = config.getboolean('use_adc', default=False)
        self.pres_cnt = config.getint('pres_cnt', 1, minval=1, maxval=MAX_PRES_CNT)
        self.sys_time_duty = config.getfloat('sys_time_duty', default=0.001,
                                              minval=0.00001, maxval=0.010)

        step_swap_pin = config.get('step_swap_pin')
        pres_swap_pin = config.get('pres_swap_pin')
        step_swap = ppins.parse_pin(step_swap_pin, True, True)
        pres_swap = ppins.parse_pin(pres_swap_pin, True, True)
        self.step_mcu = step_swap['chip']
        self.pres_mcu = pres_swap['chip']
        self._step_swap_pin_name = step_swap['pin']
        self._pres_swap_pin_name = pres_swap['pin']

        self.is_corexz = config.getsection('printer').get('kinematics', '') == 'corexz'
        self._z_step_pins = []
        self._z_dir_pins = []
        for name in ('stepper_z', 'stepper_x' if self.is_corexz else 'stepper_z1',
                     'stepper_z2', 'stepper_z3'):
            if config.has_section(name):
                sec = config.getsection(name)
                self._z_step_pins.append(sec.get('step_pin'))
                self._z_dir_pins.append(sec.get('dir_pin'))
        if not self._z_step_pins:
            raise config.error("prtouch_mcu: no stepper_z section found")

        self._pres_clk_pins = []
        self._pres_sdo_pins = []
        self._pres_adc_pins = []
        for i in range(self.pres_cnt):
            if self.use_adc:
                self._pres_adc_pins.append(config.get('pres%d_adc_pins' % i))
            else:
                self._pres_clk_pins.append(config.get('pres%d_clk_pins' % i))
                self._pres_sdo_pins.append(config.get('pres%d_sdo_pins' % i))

        self.step_oid = self.step_mcu.create_oid()
        self.pres_oid = self.pres_mcu.create_oid()
        self.step_mcu.register_config_callback(self._build_step_config)
        self.pres_mcu.register_config_callback(self._build_pres_config)

        self.step_res = []
        self.pres_res = []
        self.step_tri_time = 0.
        self.pres_tri_time = 0.
        self.pres_tri_chs = 0
        self.pres_buf_cnt = 0

        self.read_swap_prtouch_cmd = None
        self.start_step_prtouch_cmd = None
        self.manual_get_steps_cmd = None
        self.write_swap_prtouch_cmd = None
        self.read_pres_prtouch_cmd = None
        self.start_pres_prtouch_cmd = None
        self.deal_avgs_prtouch_cmd = None
        self.manual_get_pres_cmd = None

        self.step_mcu.register_response(self._handle_result_run_step_prtouch,
                                         "result_run_step_prtouch", self.step_oid)
        self.pres_mcu.register_response(self._handle_result_run_pres_prtouch,
                                         "result_run_pres_prtouch", self.pres_oid)
        self.pres_mcu.register_response(self._handle_result_read_pres_prtouch,
                                         "result_read_pres_prtouch", self.pres_oid)

    def _build_step_config(self):
        ppins = self.printer.lookup_object('pins')
        self.step_mcu.add_config_cmd(
            'config_step_prtouch oid=%d step_cnt=%d swap_pin=%s sys_time_duty=%u' % (
                self.step_oid, len(self._z_step_pins), self._step_swap_pin_name,
                units.duty_fraction_to_scaled_units(self.sys_time_duty)))
        for i in range(len(self._z_step_pins)):
            step_par = ppins.parse_pin(self._z_step_pins[i], True, True)
            dir_par = ppins.parse_pin(self._z_dir_pins[i], True, True)
            dir_invert = dir_par['invert']
            if self.is_corexz and i == 0:
                dir_invert = not dir_invert
            self.step_mcu.add_config_cmd(
                'add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s '
                'dir_invert=%d step_invert=%d' % (
                    self.step_oid, i, dir_par['pin'], step_par['pin'],
                    dir_invert, step_par['invert']))
        self.read_swap_prtouch_cmd = self.step_mcu.lookup_query_command(
            'read_swap_prtouch oid=%c', 'result_read_swap_prtouch oid=%c sta=%c',
            oid=self.step_oid)
        self.start_step_prtouch_cmd = self.step_mcu.lookup_command(
            'start_step_prtouch oid=%c dir=%c send_ms=%c step_cnt=%u step_us=%u '
            'acc_ctl_cnt=%u low_spd_nul=%c send_step_duty=%c auto_rtn=%c', cq=None)
        self.manual_get_steps_cmd = self.step_mcu.lookup_query_command(
            'manual_get_steps oid=%c index=%c',
            'result_manual_get_steps oid=%c index=%c tri_time=%u '
            'tick0=%u tick1=%u tick2=%u tick3=%u step0=%u step1=%u step2=%u step3=%u',
            oid=self.step_oid)

    def _build_pres_config(self):
        ppins = self.printer.lookup_object('pins')
        self.pres_mcu.add_config_cmd(
            'config_pres_prtouch oid=%d use_adc=%d pres_cnt=%d swap_pin=%s sys_time_duty=%u' % (
                self.pres_oid, self.use_adc, self.pres_cnt, self._pres_swap_pin_name,
                units.duty_fraction_to_scaled_units(self.sys_time_duty)))
        for i in range(self.pres_cnt):
            if self.use_adc:
                adc_par = ppins.parse_pin(self._pres_adc_pins[i], True, True)
                clk_pin = sdo_pin = adc_par['pin']
            else:
                clk_par = ppins.parse_pin(self._pres_clk_pins[i], True, True)
                sdo_par = ppins.parse_pin(self._pres_sdo_pins[i], True, True)
                clk_pin, sdo_pin = clk_par['pin'], sdo_par['pin']
            self.pres_mcu.add_config_cmd(
                'add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (
                    self.pres_oid, i, clk_pin, sdo_pin))
        self.write_swap_prtouch_cmd = self.pres_mcu.lookup_query_command(
            'write_swap_prtouch oid=%c sta=%c', 'resault_write_swap_prtouch oid=%c',
            oid=self.pres_oid)
        self.read_pres_prtouch_cmd = self.pres_mcu.lookup_command(
            'read_pres_prtouch oid=%c acq_ms=%u cnt=%u', cq=None)
        self.start_pres_prtouch_cmd = self.pres_mcu.lookup_command(
            'start_pres_prtouch oid=%c tri_dir=%c acq_ms=%c send_ms=%c need_cnt=%c '
            'tri_hftr_cut=%u tri_lftr_k1=%u min_hold=%u max_hold=%u', cq=None)
        self.deal_avgs_prtouch_cmd = self.pres_mcu.lookup_query_command(
            'deal_avgs_prtouch oid=%c base_cnt=%c',
            'result_deal_avgs_prtouch oid=%c ch0=%i ch1=%i ch2=%i ch3=%i', oid=self.pres_oid)
        self.manual_get_pres_cmd = self.pres_mcu.lookup_query_command(
            'manual_get_pres oid=%c index=%c',
            'resault_manual_get_pres oid=%c index=%c tri_time=%u tri_chs=%c buf_cnt=%u '
            'tick_0=%u ch0_0=%i ch1_0=%i ch2_0=%i ch3_0=%i '
            'tick_1=%u ch0_1=%i ch1_1=%i ch2_1=%i ch3_1=%i', oid=self.pres_oid)

    # -- async response handlers --------------------------------------------------

    def _handle_result_run_step_prtouch(self, params):
        self.step_tri_time = units.mcu_ticks_to_seconds(params['tri_time'])
        for i in range(4):
            self.step_res.append({
                'tick': units.mcu_ticks_to_seconds(params['tick%d' % i]),
                'step': params['step%d' % i],
                'index': params['index'],
            })

    def _handle_result_run_pres_prtouch(self, params):
        self.pres_tri_time = units.mcu_ticks_to_seconds(params['tri_time'])
        self.pres_tri_chs = params['tri_chs']
        self.pres_buf_cnt = params['buf_cnt']
        for i in range(2):
            self.pres_res.append({
                'tick': units.mcu_ticks_to_seconds(params['tick_%d' % i]),
                'ch0': params['ch0_%d' % i], 'ch1': params['ch1_%d' % i],
                'ch2': params['ch2_%d' % i], 'ch3': params['ch3_%d' % i],
                'index': params['index'],
            })

    def _handle_result_read_pres_prtouch(self, params):
        self.pres_res.append(params)

    # -- public API -----------------------------------------------------------

    def reset_buffers(self):
        self.step_res = []
        self.pres_res = []

    def start_step(self, direction, step_cnt, step_us, acc_ctl_cnt, send_ms=10,
                   low_spd_nul=5, send_step_duty=16, auto_rtn=0):
        self.start_step_prtouch_cmd.send([
            self.step_oid, direction, send_ms, step_cnt, step_us, acc_ctl_cnt,
            low_spd_nul, send_step_duty, auto_rtn])

    def start_pres(self, direction, acq_ms, send_ms, need_cnt, hftr_cut, lftr_k1,
                   min_hold, max_hold):
        self.start_pres_prtouch_cmd.send([
            self.pres_oid, direction, acq_ms, send_ms, need_cnt,
            units.to_fixed_point(hftr_cut), units.to_fixed_point(lftr_k1),
            int(min_hold), int(max_hold)])

    def stop(self):
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, 5, 16, 0])
        self.start_pres_prtouch_cmd.send([self.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])

    def deal_avgs(self, base_cnt=8):
        return self.deal_avgs_prtouch_cmd.send([self.pres_oid, base_cnt])

    def read_swap(self):
        params = self.read_swap_prtouch_cmd.send([self.step_oid])
        return bool(params['sta'])

    def write_swap(self, state):
        self.write_swap_prtouch_cmd.send([self.pres_oid, int(state)])

    def collect_step_samples(self, timeout_s):
        end_time = self.reactor.monotonic() + timeout_s
        eventtime = self.reactor.monotonic()
        while len(self.step_res) != MAX_BUF_LEN and eventtime < end_time:
            eventtime = self.reactor.pause(eventtime + POLL_INTERVAL)
        if len(self.step_res) != MAX_BUF_LEN:
            self._repair_step_samples()
        return list(self.step_res)

    def collect_pres_samples(self, timeout_s):
        end_time = self.reactor.monotonic() + timeout_s
        eventtime = self.reactor.monotonic()
        while len(self.pres_res) != MAX_BUF_LEN and eventtime < end_time:
            eventtime = self.reactor.pause(eventtime + POLL_INTERVAL)
        if len(self.pres_res) != MAX_BUF_LEN:
            self._repair_pres_samples()
        return list(self.pres_res)

    def _repair_step_samples(self):
        logging.info("prtouch_mcu: repairing step samples, got %d/%d",
                      len(self.step_res), MAX_BUF_LEN)
        for i in range(0, MAX_BUF_LEN, 4):
            if len(self.step_res) > i and self.step_res[i]['index'] == i:
                continue
            params = self.manual_get_steps_cmd.send([self.step_oid, i])
            self.step_tri_time = units.mcu_ticks_to_seconds(params['tri_time'])
            for j in range(4):
                self.step_res.insert(i + j, {
                    'tick': units.mcu_ticks_to_seconds(params['tick%d' % j]),
                    'step': params['step%d' % j],
                    'index': params['index'],
                })
        if len(self.step_res) != MAX_BUF_LEN:
            raise PrtouchProtocolError(
                "step sample repair failed: got %d/%d" % (len(self.step_res), MAX_BUF_LEN))

    def _repair_pres_samples(self):
        logging.info("prtouch_mcu: repairing pres samples, got %d/%d",
                      len(self.pres_res), MAX_BUF_LEN)
        for i in range(0, MAX_BUF_LEN, 2):
            if len(self.pres_res) > i and self.pres_res[i]['index'] == i:
                continue
            # NOTE: the original (prtouch_v2_wrapper.py line 641) sends self.step_oid here,
            # which looks like a copy-paste bug from ck_and_manual_get_step - manual_get_pres
            # is registered under pres_oid (config_pres_prtouch/add_pres_prtouch), so this uses
            # pres_oid instead. This is a clean rewrite, not a verbatim port (ANALYSIS.md sec 6),
            # so this was corrected rather than preserved; flagged in case real-hardware testing
            # ever shows the original's behavior was intentional for some reason not visible in
            # the source.
            params = self.manual_get_pres_cmd.send([self.pres_oid, i])
            self.pres_tri_time = units.mcu_ticks_to_seconds(params['tri_time'])
            self.pres_tri_chs = params['tri_chs']
            self.pres_buf_cnt = params['buf_cnt']
            for j in range(2):
                self.pres_res.insert(i + j, {
                    'tick': units.mcu_ticks_to_seconds(params['tick_%d' % j]),
                    'ch0': params['ch0_%d' % j], 'ch1': params['ch1_%d' % j],
                    'ch2': params['ch2_%d' % j], 'ch3': params['ch3_%d' % j],
                    'index': params['index'],
                })
        if len(self.pres_res) != MAX_BUF_LEN:
            raise PrtouchProtocolError(
                "pres sample repair failed: got %d/%d" % (len(self.pres_res), MAX_BUF_LEN))
