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
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import prtouch_calibration


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
        self.tri_z_down_spd = config.getfloat('tri_z_down_spd', default=(10. if use_adc else 2.5),
                                               minval=0.1)
        self.tri_z_up_spd = config.getfloat(
            'lift_speed', default=self.tri_z_down_spd * (1.0 if use_adc else 2.0), minval=0.1)
        self.acc_ctl_mm = config.getfloat('acc_ctl_mm', default=(0.5 if use_adc else 0.25),
                                           minval=0)
        self.low_spd_nul = config.getint('low_spd_nul', default=5, minval=1, maxval=10)
        self.send_step_duty = config.getint('send_step_duty', default=16, minval=0, maxval=10)
        self.probe_min_3err = config.getfloat('probe_min_3err', default=0.1, minval=0.01)
        self.step_base = config.getint('step_base', default=1, minval=1)

        self.mm_per_step = None
        self.bed_mesh = None
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

    def get_step_counts(self, distance, speed):
        """get_step_cnts-equivalent: distance/speed -> (step_cnt, step_us, acc_ctl_cnt) for the
        MCU's start_step_prtouch pulse-train command."""
        step_cnt = int(distance / self.mm_per_step)
        if step_cnt <= 0:
            return 0, 0, 0
        step_us = int((distance / speed) * 1000. * 1000. / step_cnt)
        acc_ctl_cnt = int(self.acc_ctl_mm / self.mm_per_step)
        return step_cnt, step_us, acc_ctl_cnt

    def safe_move_z(self, direction, distance, speed):
        """Non-probing raw Z move via the same MCU step command (safe_move_z-equivalent) - used
        for the pre-error safety lift and general manual moves outside a probe cycle. direction:
        1 = up, 0 = down."""
        step_cnt, step_us, acc_ctl_cnt = self.get_step_counts(distance, speed)
        if step_cnt == 0:
            return
        self.mcu.reset_buffers()
        self.mcu.start_step(direction, step_cnt, step_us, acc_ctl_cnt,
                             send_ms=self.tri_send_ms, low_spd_nul=self.low_spd_nul,
                             send_step_duty=self.send_step_duty)
        self.mcu.collect_step_samples(distance / speed + 5.0)
        self.mcu.start_step(direction, 0, 0, 0, low_spd_nul=self.low_spd_nul,
                             send_step_duty=self.send_step_duty)

    def _fail(self, message):
        """ck_and_raise_error-equivalent: Z motion during a probe is a raw, non-interruptible
        MCU-side pulse train, not a Klipper-queued move (ANALYSIS.md sec 6) - the only real
        safety net is lifting clear before surfacing the error, which is what this does."""
        logging.info("prtouch_probe: %s", message)
        self.safe_move_z(1, 5.0, 10.0)
        raise self.printer.command_error("prtouch: " + message)

    def touch_probe(self, down_min_z, retries=10, pro_cnt=3, tolerance=None):
        """run_step_prtouch-equivalent (ANALYSIS.md secs 3-4): send start_step+start_pres
        concurrently, collect both buffers, compute one Z sample via
        prtouch_calibration.compute_trigger_z(), lift back to the start height, and repeat
        until either two samples agree within tolerance or pro_cnt samples have been collected.
        Raises command_error (via _fail, with the safety-lift courtesy) after `retries` attempts
        without a usable sample - mirrors PR_NOT_TRIGGER/STEP_LOST/PRES_LOST (ANALYSIS.md sec 1)
        at a reduced surface, using plain command_error rather than Creality's PR_ERR_CODE_*
        catalog (DESIGN.md open question 3, resolved this way for the clean rewrite).

        Suspends any active bed mesh for the duration of the probe cycle (matches the original):
        a loaded mesh applies a Z-compensation transform to every toolhead move, which would
        skew a raw touch-probe reading taken at a specific mesh-relative point.
        """
        saved_mesh = self.bed_mesh.get_mesh() if self.bed_mesh is not None else None
        if saved_mesh is not None:
            self.bed_mesh.set_mesh(None)
        try:
            return self._touch_probe(down_min_z, retries, pro_cnt, tolerance)
        finally:
            if saved_mesh is not None:
                self.bed_mesh.set_mesh(saved_mesh)

    def _touch_probe(self, down_min_z, retries, pro_cnt, tolerance):
        if tolerance is None:
            tolerance = self.probe_min_3err
        results = []
        attempt = 0
        while len(results) < pro_cnt:
            if attempt >= retries:
                self._fail("touch_probe did not converge after %d attempts (results=%s)"
                            % (attempt, results))
            attempt += 1

            self.mcu.reset_buffers()
            self.mcu.deal_avgs(base_cnt=8)
            step_cnt, step_us, acc_ctl_cnt = self.get_step_counts(down_min_z, self.tri_z_down_spd)
            start_pos_z = self.toolhead.get_position()[2]

            self.mcu.start_pres(0, self.tri_acq_ms, self.tri_send_ms, self.tri_need_cnt,
                                 self.tri_hftr_cut, self.tri_lftr_k1,
                                 self.tri_min_hold, self.tri_max_hold)
            self.mcu.start_step(0, step_cnt, step_us, acc_ctl_cnt, send_ms=self.tri_send_ms,
                                 low_spd_nul=self.low_spd_nul, send_step_duty=self.send_step_duty)
            timeout = down_min_z / self.tri_z_down_spd + 2.0
            step_samples = self.mcu.collect_step_samples(timeout)
            pres_samples = self.mcu.collect_pres_samples(timeout)
            self.mcu.start_step(0, 0, 0, 0, low_spd_nul=self.low_spd_nul,
                                 send_step_duty=self.send_step_duty)
            self.mcu.start_pres(0, 0, 0, 0, 0, 0, 0, 0)

            if not step_samples or not pres_samples:
                logging.info("prtouch_probe: no trigger on attempt %d/%d", attempt, retries)
                continue

            try:
                z = prtouch_calibration.compute_trigger_z(
                    step_samples, pres_samples, self.mcu.step_tri_time, self.mcu.pres_tri_time,
                    self.mcu.pres_tri_chs, step_cnt, start_pos_z, self.mm_per_step,
                    self.mcu.use_adc, self.tri_acq_ms, self.cal_hftr_cut, self.cal_lftr_k1,
                    pres_cnt=self.mcu.pres_cnt)
            except ValueError as e:
                logging.info("prtouch_probe: %s on attempt %d/%d", e, attempt, retries)
                self._lift_after_down(step_cnt, step_samples)
                continue

            results.append(z)
            self._lift_after_down(step_cnt, step_samples)

            if len(results) >= 2 and (max(results) - min(results)) <= tolerance:
                break

        results.sort()
        n = len(results)
        return results[n // 2] if n % 2 == 1 else (results[n // 2 - 1] + results[n // 2]) / 2.

    def _lift_after_down(self, step_cnt, step_samples):
        traveled = (step_cnt - step_samples[-1]['step']) * self.mm_per_step
        if traveled <= 0:
            return
        up_cnt, up_us, up_acc = self.get_step_counts(traveled, self.tri_z_up_spd)
        if up_cnt == 0:
            return
        self.mcu.reset_buffers()
        self.mcu.start_step(1, up_cnt, up_us, up_acc, send_ms=self.tri_send_ms,
                             low_spd_nul=self.low_spd_nul, send_step_duty=self.send_step_duty)
        self.mcu.collect_step_samples(traveled / self.tri_z_up_spd + 2.0)
        self.mcu.start_step(1, 0, 0, 0, low_spd_nul=self.low_spd_nul,
                             send_step_duty=self.send_step_duty)
