# prtouch_v2 MCU protocol - oid/config setup, raw command send, response buffering
#
# Clean-room rewrite of the MCU-facing half of Creality's prtouch_v2_wrapper.py (GPLv3, see
# reference/) against the *existing, unreflashed* toolhead firmware - same wire protocol, same
# standard Klipper host APIs (create_oid/add_config_cmd/lookup_command/
# register_serial_response) already proven on this device by hx711s.py. See
# ../ANALYSIS.md secs 1-2 for the full protocol trace this is built from and ../DESIGN.md for
# how this file fits the six-file layout.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import prtouch_units as units

MAX_BUF_LEN = 32
MAX_PRES_CNT = 4
POLL_INTERVAL = 0.010


class PrtouchProtocolError(Exception):
    pass


# ---------------------------------------------------------------------------------------
# Async response subscription formats (official-mainline migration, 2026-08-17)
# ---------------------------------------------------------------------------------------
#
# Mainline commit c89393cda (2026-02-26, "mcu: Rework mcu.register_response() to
# mcu.register_serial_response()") replaced the old MCU.register_response(cb, NAME, oid) with
# MCU.register_serial_response(cb, MSGFORMAT, oid). Three things changed, all of which this
# module has to account for - a plain rename would be wrong:
#
#   1. The second argument is now the FULL message format string, not the message name.
#      AsyncResponseWrapper takes the name back off it with msgformat.split()[0].
#   2. The format is VALIDATED: AsyncResponseWrapper._register() calls
#      msgparser.lookup_command(msgformat), which is an exact string comparison against the
#      format the MCU itself declared in its dictionary, and raises on any mismatch.
#   3. Registration is DEFERRED. If the MCU's config is not yet finalized, the wrapper
#      installs a discard-everything placeholder immediately and registers the real callback
#      from a post-init callback instead, so a callback can never see data left over from a
#      previous session.
#
# Consequence for this module: (2) means these subscriptions can no longer be set up in
# __init__ - at config-parse time no serial connection exists yet, so there is no MCU
# dictionary to validate against. They are set up from the existing config callbacks
# (_build_step_config/_build_pres_config) instead, which mainline runs from
# MCUConfigHelper._finalize_config() *after* MCU identify and *before* _config_finalized is
# set. That is the same point at which this module's existing lookup_query_command() calls
# already validate their own formats against the real dictionary, so it is a proven-available
# moment, and because the config is not finalized yet the wrapper still takes its intended
# deferred-registration path (3).
#
# On the format strings themselves, stated honestly: NebulaOS does not own this MCU firmware.
# The KE runs Creality's proprietary, unreflashed toolhead firmware, and its dictionary is the
# only authority on the exact declared format. That dictionary is not reproducible offline
# from anything in this project (the fw/*.bin blobs inherited from the Pellcorp base are K1
# builds and contain no prtouch messages at all - checked, not assumed). What IS known
# exactly is the field set each handler below consumes, and the byte-for-byte format of the
# *query* responses that carry the same payloads - result_manual_get_steps and
# resault_manual_get_pres - because those formats are already passed to
# lookup_query_command() by this same module and are therefore already validated against the
# real dictionary on every single boot today.
#
# So each subscription declares an ordered list of candidate formats derived from exactly
# that evidence, and _subscribe() below picks the one the connected MCU actually declares
# using MCU.check_valid_response() - a public, non-raising mainline API (klippy/mcu.py, right
# beside register_serial_response itself). If none match, it refuses to start with a message
# naming the message, every candidate tried, and how to obtain the real format. It never
# silently skips a probe-telemetry subscription: a PRTouch descent with a dead telemetry
# callback is a nozzle driven into the bed with no trigger path.
#
# Each entry: (required_param_names, (candidate_format, ...))

# _handle_result_run_step_prtouch reads: index, tri_time, tick0..3, step0..3. Its `index` read
# is unconditional, so there is exactly one viable candidate here - a no-index variant would
# be rejected by the required-field check below anyway, and listing one would be dead weight
# that reads like a working fallback. The candidate is result_manual_get_steps' payload
# verbatim, which this module already validates against the real dictionary on every boot.
RESULT_RUN_STEP_PRTOUCH = (
    ('index', 'tri_time', 'tick0', 'tick1', 'tick2', 'tick3',
     'step0', 'step1', 'step2', 'step3'),
    ('result_run_step_prtouch oid=%c index=%c tri_time=%u'
     ' tick0=%u tick1=%u tick2=%u tick3=%u'
     ' step0=%u step1=%u step2=%u step3=%u',),
)

# _handle_result_run_pres_prtouch reads: tri_time, tri_chs, buf_cnt, tick_N, chX_N (N in 0..1).
# Candidate 1 is resault_manual_get_pres' already-validated payload, verbatim. _repair_pres_
# samples() compares self.pres_res[i]['index'], which is independent evidence that this
# message family carries an index field - but this particular handler never reads it, so a
# no-index variant is genuinely usable here and is offered as a real second candidate.
RESULT_RUN_PRES_PRTOUCH = (
    ('tri_time', 'tri_chs', 'buf_cnt',
     'tick_0', 'ch0_0', 'ch1_0', 'ch2_0', 'ch3_0',
     'tick_1', 'ch0_1', 'ch1_1', 'ch2_1', 'ch3_1'),
    ('result_run_pres_prtouch oid=%c index=%c tri_time=%u tri_chs=%c buf_cnt=%u'
     ' tick_0=%u ch0_0=%i ch1_0=%i ch2_0=%i ch3_0=%i'
     ' tick_1=%u ch0_1=%i ch1_1=%i ch2_1=%i ch3_1=%i',
     'result_run_pres_prtouch oid=%c tri_time=%u tri_chs=%c buf_cnt=%u'
     ' tick_0=%u ch0_0=%i ch1_0=%i ch2_0=%i ch3_0=%i'
     ' tick_1=%u ch0_1=%i ch1_1=%i ch2_1=%i ch3_1=%i'),
)

# Phase 1.5 hardware closure (2026-08-19): the entry above (candidate 1, borrowed from
# resault_manual_get_pres) was WRONG - real hardware rejected it outright ("MCU ... does not
# declare any known format for the async response 'result_read_pres_prtouch'"). The assumption
# that read_pres_prtouch shares manual_get_pres's dual-reading/index/tri_time/tri_chs/buf_cnt
# payload doesn't hold: they are two different commands in two different code paths in the real
# firmware. Confirmed directly, byte-for-byte, against this project's own vendored ground truth:
#   - reference/prtouch_v2.c:766 (Creality's own source, prtouch_pres_task's `read_fix_cnt`
#     branch): sendf("result_read_pres_prtouch oid=%c tick=%u ch0=%i ch1=%i ch2=%i ch3=%i", ...)
#     - one broadcast per requested sample (driven by read_pres_prtouch's own acq_ms/cnt
#     arguments, see command_read_pres_prtouch at prtouch_v2.c:622), not a dual-buffered,
#     index-tagged, triggered batch like result_run_pres_prtouch's genuinely separate sendf at
#     prtouch_v2.c:783 (RESULT_RUN_PRES_PRTOUCH above is unaffected - that candidate is correct).
#   - ANALYSIS.md:33 independently documents the identical mapping:
#     `result_read_pres_prtouch oid=%c tick=%u ch0..3=%i`.
# No index field exists on the wire for this message; the real firmware never tags a read_pres
# broadcast with a buffer position, unlike the two commands that do (manual_get_steps/
# manual_get_pres, and result_run_step_prtouch). Handled in _handle_result_read_pres_prtouch()
# below, which assigns 'index' as the append position - exactly the same position-tracking
# invariant _repair_pres_samples()'s own self.pres_res[i]['index'] == i check already relies on.
RESULT_READ_PRES_PRTOUCH = (
    ('tick', 'ch0', 'ch1', 'ch2', 'ch3'),
    ('result_read_pres_prtouch oid=%c tick=%u ch0=%i ch1=%i ch2=%i ch3=%i',),
)


def format_param_names(msgformat):
    """Parameter names declared by a Klipper message format string, in order, excluding the
    leading message name. 'result_x oid=%c tri_time=%u' -> ['oid', 'tri_time']."""
    return [part.split('=', 1)[0]
            for part in msgformat.strip().split()[1:] if '=' in part]


def select_response_format(mcu, subscription, msgname):
    """Pick the candidate format the connected MCU actually declares.

    Uses only MCU.check_valid_response(), a public mainline API that returns a bool rather
    than raising. Returns the winning format string, or None if the MCU declares none of
    them (the caller turns that into a fail-closed config error)."""
    required, candidates = subscription
    for msgformat in candidates:
        if not mcu.check_valid_response(msgformat):
            continue
        declared = set(format_param_names(msgformat))
        missing = [name for name in required if name not in declared]
        if missing:
            # The MCU agrees this format exists, but it does not carry a field one of this
            # module's own handlers reads. Treat that as no match rather than registering a
            # subscription that would KeyError on the first real sample.
            logging.warning(
                "prtouch_mcu: '%s' candidate format validated but is missing handler-required"
                " field(s) %s - skipping candidate", msgname, ', '.join(missing))
            continue
        return msgformat
    return None


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

        # Async telemetry subscriptions are NOT set up here any more - mainline's
        # register_serial_response() validates the message format against the MCU's own
        # dictionary, which does not exist yet at config-parse time. They are registered from
        # _build_step_config/_build_pres_config instead; see the module header for the full
        # reasoning. These hold the returned AsyncResponseWrapper objects (which expose
        # .unregister()) so nothing here depends on wrapper identity being discarded.
        self.step_response = None
        self.pres_run_response = None
        self.pres_read_response = None

    def _subscribe(self, mcu, callback, subscription, msgname, oid):
        """Register one async response subscription against mainline's
        MCU.register_serial_response(), resolving the exact declared format first.

        Fail-closed: if the connected MCU declares none of the candidate formats, this raises
        a config error rather than leaving PRTouch running with a telemetry callback that can
        never fire."""
        msgformat = select_response_format(mcu, subscription, msgname)
        if msgformat is None:
            required, candidates = subscription
            raise self.printer.config_error(
                "prtouch_mcu: MCU '%s' does not declare any known format for the async"
                " response '%s', so this printer's load-cell probe telemetry cannot be"
                " subscribed to and PRTouch must not run.\n"
                "  Handler-required fields: %s\n"
                "  Formats tried:\n    %s\n"
                "This means the toolhead firmware's message dictionary differs from the one"
                " NebulaOS was qualified against. Recover the real format with Klipper's own"
                " console tool (python3 klippy/console.py <serial>, then LIST) and add it as a"
                " candidate in prtouch_mcu.py's %s table - do not remove the subscription."
                % (mcu.get_name(), msgname, ', '.join(required),
                   '\n    '.join(candidates), msgname.upper()))
        logging.info("prtouch_mcu: subscribing to '%s' as: %s", msgname, msgformat)
        return mcu.register_serial_response(callback, msgformat, oid)

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
        self.step_response = self._subscribe(
            self.step_mcu, self._handle_result_run_step_prtouch,
            RESULT_RUN_STEP_PRTOUCH, 'result_run_step_prtouch', self.step_oid)

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
        self.pres_run_response = self._subscribe(
            self.pres_mcu, self._handle_result_run_pres_prtouch,
            RESULT_RUN_PRES_PRTOUCH, 'result_run_pres_prtouch', self.pres_oid)
        self.pres_read_response = self._subscribe(
            self.pres_mcu, self._handle_result_read_pres_prtouch,
            RESULT_READ_PRES_PRTOUCH, 'result_read_pres_prtouch', self.pres_oid)

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
        # No index on the wire for this message (see RESULT_READ_PRES_PRTOUCH's header note) -
        # assigned here as the append position, matching the normalized entry shape every other
        # producer of self.pres_res (result_run_pres_prtouch, _repair_pres_samples) already uses.
        index = len(self.pres_res)
        self.pres_res.append({
            'tick': units.mcu_ticks_to_seconds(params['tick']),
            'ch0': params['ch0'], 'ch1': params['ch1'],
            'ch2': params['ch2'], 'ch3': params['ch3'],
            'index': index,
        })

    # -- public API -----------------------------------------------------------

    def reset_buffers(self):
        self.step_res = []
        self.pres_res = []

    def start_step(self, direction, step_cnt, step_us, acc_ctl_cnt, send_ms=10,
                   low_spd_nul=5, send_step_duty=16, auto_rtn=0):
        """ARM only. 2026-08-14 (disarm-protocol mission): step_cnt=0 is rejected here, not
        silently accepted - see stop_step()'s own docstring for why a step_cnt=0 call through
        THIS method (which defaults send_ms=10) is not the same thing as a real disarm on the
        actual MCU wire protocol, and would not cleanly stop the step timer."""
        if step_cnt == 0:
            raise ValueError(
                "prtouch_mcu: start_step() called with step_cnt=0 (send_ms=%d) - this is not "
                "a valid disarm on the real MCU protocol (see stop_step()'s own docstring); "
                "call stop_step() instead" % send_ms)
        self.start_step_prtouch_cmd.send([
            self.step_oid, direction, send_ms, step_cnt, step_us, acc_ctl_cnt,
            low_spd_nul, send_step_duty, auto_rtn])

    def stop_step(self):
        """The one real step-disarm packet. 2026-08-14 (disarm-protocol mission - see
        docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md sec 18/19): reference/prtouch_v2.c's
        command_start_step_prtouch checks send_ms==0 (its 3rd wire field, `args[2]`) as the
        dedicated stop sentinel - on a match it sets need_stop=1, calls stop_sys_time(), and
        returns immediately, WITHOUT ever reaching sched_add_timer(). Every real stock disarm
        call (reference/prtouch_v2_wrapper.py, e.g. line 445) sends send_ms=0 for exactly this
        reason. This host's own disarm calls previously went through start_step()'s own
        send_ms=10 default instead (step_cnt=0 but send_ms=10) - on the real protocol that
        does NOT hit the send_ms==0 early-return, and instead falls through to the normal arm
        path with step_cnt=0/step_us=0/acc_ctl_cnt=0 - a degenerate re-arm, not a clean stop.
        This method exists so that mistake is structurally impossible to make again: it always
        sends the exact stock disarm shape (all fields zero but oid), and start_step() itself
        now refuses a step_cnt=0 call rather than silently accepting one."""
        self.start_step_prtouch_cmd.send([self.step_oid, 0, 0, 0, 0, 0, 0, 0, 0])

    def start_pres(self, direction, acq_ms, send_ms, need_cnt, hftr_cut, lftr_k1,
                   min_hold, max_hold):
        self.start_pres_prtouch_cmd.send([
            self.pres_oid, direction, acq_ms, send_ms, need_cnt,
            units.to_fixed_point(hftr_cut), units.to_fixed_point(lftr_k1),
            int(min_hold), int(max_hold)])

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
