# Protocol parity tests - field-by-field comparison of every command PrtouchMCU sends
# against the real, published Creality reference (reference/prtouch_v2_wrapper.py), using
# the real production PRTouchV2/PrtouchMCU/PrtouchProbe classes wired to a fake MCU.
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_protocol -v (this fork's own layout - klippy/extras/
# is a real Python package named 'extras', not 'klippy_extras' - see NebulaOS-firmware's
# klippy_extras/ mirror of this same file for that repo's own invocation form)
# (not from within klippy_extras/, unlike test_prtouch_calibration.py - prtouch_v2.py's
# `from . import ...` relative imports need a real package, see prtouch_test_support.py's
# module docstring for why).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_test_support as fake
from . import prtouch_v2


def _build():
    printer, mcu, pins, values = fake.build_environment()
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    fake.connect(printer, mcu)
    config.assert_all_consumed()
    return printer, mcu, pv2


class ConfigTimeCommandsTest(unittest.TestCase):
    """config_step_prtouch/add_step_prtouch/config_pres_prtouch/add_pres_prtouch - sent
    once via add_config_cmd during each MCU's config callback. Reference format strings,
    confirmed by direct grep of reference/prtouch_v2_wrapper.py lines 267-303."""

    def test_config_step_prtouch_fields(self):
        _, mcu, pv2 = _build()
        cmd = mcu.config_cmds[0]
        self.assertEqual(
            cmd, 'config_step_prtouch oid=%d step_cnt=%d swap_pin=%s sys_time_duty=%u' % (
                0, 1, 'PA15', 100))

    def test_add_step_prtouch_fields(self):
        _, mcu, pv2 = _build()
        cmd = mcu.config_cmds[1]
        self.assertEqual(
            cmd, 'add_step_prtouch oid=%d index=%d dir_pin=%s step_pin=%s '
                 'dir_invert=%d step_invert=%d' % (0, 0, 'PB5', 'PB6', 1, 0))

    def test_config_pres_prtouch_fields(self):
        _, mcu, pv2 = _build()
        cmd = next(c for c in mcu.config_cmds if c.startswith('config_pres_prtouch'))
        self.assertEqual(
            cmd, 'config_pres_prtouch oid=%d use_adc=%d pres_cnt=%d swap_pin=%s '
                 'sys_time_duty=%u' % (1, False, 1, 'PA15', 100))

    def test_add_pres_prtouch_fields(self):
        _, mcu, pv2 = _build()
        cmd = next(c for c in mcu.config_cmds if c.startswith('add_pres_prtouch'))
        self.assertEqual(
            cmd, 'add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s' % (1, 0, 'PA4', 'PC6'))

    def test_step_and_pres_get_distinct_oids(self):
        # this printer shares one physical MCU/pin (PA15) for both swap signals but the two
        # protocol channels must still be logically distinct (separate create_oid() calls) -
        # confirmed against reference lines 254-256 (self.step_oid/self.pres_oid, two
        # separate create_oid() results).
        _, mcu, pv2 = _build()
        self.assertNotEqual(pv2.mcu.step_oid, pv2.mcu.pres_oid)

    def test_adc_mode_uses_same_pin_for_clk_and_sda(self):
        # reference line 292: add_pres_prtouch(..., adc_par['pin'], adc_par['pin']) - clk and
        # sda are the SAME pin in ADC/piezo mode, confirmed intentional, not a typo.
        printer, mcu, pins, values = fake.build_environment({
            'use_adc': 'True', 'pres0_adc_pins': 'PC5',
        })
        del values['pres0_clk_pins']
        del values['pres0_sdo_pins']
        config = fake.make_prtouch_v2_config(printer, pins, values)
        pins._chip_by_pin['PC5'] = mcu
        pv2 = prtouch_v2.PRTouchV2(config)
        fake.connect(printer, mcu)
        config.assert_all_consumed()
        cmd = next(c for c in mcu.config_cmds if c.startswith('add_pres_prtouch'))
        self.assertEqual(cmd, 'add_pres_prtouch oid=%d index=%d clk_pin=%s sda_pin=%s'
                          % (1, 0, 'PC5', 'PC5'))


class RuntimeCommandFieldsTest(unittest.TestCase):
    """start_step_prtouch/start_pres_prtouch field order and scaling, confirmed against
    reference lines 1179-1181 (the real run_step_prtouch's own send calls)."""

    def test_start_step_prtouch_field_order(self):
        _, mcu, pv2 = _build()
        pv2.probe.mcu.start_step(0, 100, 2000, 4, send_ms=10, low_spd_nul=5, send_step_duty=16)
        call = mcu.last_call('start_step_prtouch')
        self.assertEqual(call.args, [pv2.mcu.step_oid, 0, 10, 100, 2000, 4, 5, 16, 0])
        self.assertEqual(call.by_field, {
            'oid': pv2.mcu.step_oid, 'dir': 0, 'send_ms': 10, 'step_cnt': 100,
            'step_us': 2000, 'acc_ctl_cnt': 4, 'low_spd_nul': 5, 'send_step_duty': 16,
            'auto_rtn': 0,
        })

    def test_start_pres_prtouch_field_order_and_fixed_point_scaling(self):
        # reference: int(use_tri_hftr_cut * 1000), int(use_tri_lftr_k1 * 1000) - both hftr_cut
        # and lftr_k1 are floats scaled by 1000 into integer wire fields; min/max_hold are
        # sent as plain ints, NOT scaled.
        _, mcu, pv2 = _build()
        pv2.probe.mcu.start_pres(0, 12, 10, 1, 2.0, 0.7, 1000, 1500)
        call = mcu.last_call('start_pres_prtouch')
        self.assertEqual(call.by_field, {
            'oid': pv2.mcu.pres_oid, 'tri_dir': 0, 'acq_ms': 12, 'send_ms': 10, 'need_cnt': 1,
            'tri_hftr_cut': 2000, 'tri_lftr_k1': 700, 'min_hold': 1000, 'max_hold': 1500,
        })

    def test_deal_avgs_prtouch_fields(self):
        _, mcu, pv2 = _build()
        mcu.set_query_response('deal_avgs_prtouch',
                                {'oid': pv2.mcu.pres_oid, 'ch0': -251471, 'ch1': 0, 'ch2': 0,
                                 'ch3': 0})
        result = pv2.mcu.deal_avgs(base_cnt=8)
        call = mcu.last_call('deal_avgs_prtouch')
        self.assertEqual(call.by_field, {'oid': pv2.mcu.pres_oid, 'base_cnt': 8})
        self.assertEqual(result['ch0'], -251471)

    def test_zero_arm_stop_matches_reference_stop_pattern(self):
        # reference's own "stop" idiom (e.g. line 1035-1036): re-send both commands with all
        # zero fields to halt. Confirms our stop() helper matches the same zero-field shape.
        _, mcu, pv2 = _build()
        pv2.mcu.stop()
        step_call = mcu.last_call('start_step_prtouch')
        pres_call = mcu.last_call('start_pres_prtouch')
        self.assertEqual(step_call.args, [pv2.mcu.step_oid, 0, 0, 0, 0, 0, 5, 16, 0])
        self.assertEqual(pres_call.args, [pv2.mcu.pres_oid, 0, 0, 0, 0, 0, 0, 0, 0])


class ResponseHandlerUnmarshalTest(unittest.TestCase):
    """result_run_step_prtouch/result_run_pres_prtouch unmarshaling - the real tick fields
    arrive as integer MCU ticks scaled by 10000 (confirmed: our own _handle_result_* divide
    by 10000., matching every `/ 10000.` in the reference's equivalent handlers - grep
    'tri_time'] / 10000' across reference/prtouch_v2_wrapper.py confirms the same divisor
    used at every one of its own call sites, e.g. line 621's manual_get_steps handling)."""

    def test_step_response_tick_scaling(self):
        _, mcu, pv2 = _build()
        mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, {
            'oid': pv2.mcu.step_oid, 'tri_time': 12345, 'index': 0,
            'tick0': 100, 'tick1': 200, 'tick2': 300, 'tick3': 400,
            'step0': 1, 'step1': 2, 'step2': 3, 'step3': 4,
        })
        self.assertAlmostEqual(pv2.mcu.step_tri_time, 1.2345)
        self.assertEqual(len(pv2.mcu.step_res), 4)
        self.assertAlmostEqual(pv2.mcu.step_res[0]['tick'], 0.01)
        self.assertEqual(pv2.mcu.step_res[0]['step'], 1)

    def test_pres_response_tick_scaling_and_metadata(self):
        _, mcu, pv2 = _build()
        mcu.push_response('result_run_pres_prtouch', pv2.mcu.pres_oid, {
            'oid': pv2.mcu.pres_oid, 'tri_time': 5000, 'tri_chs': 0x1, 'buf_cnt': 32,
            'index': 0, 'tick_0': 10, 'ch0_0': -100, 'ch1_0': 0, 'ch2_0': 0, 'ch3_0': 0,
            'tick_1': 20, 'ch0_1': -110, 'ch1_1': 0, 'ch2_1': 0, 'ch3_1': 0,
        })
        self.assertAlmostEqual(pv2.mcu.pres_tri_time, 0.5)
        self.assertEqual(pv2.mcu.pres_tri_chs, 0x1)
        self.assertEqual(pv2.mcu.pres_buf_cnt, 32)
        self.assertEqual(len(pv2.mcu.pres_res), 2)
        self.assertEqual(pv2.mcu.pres_res[0]['ch0'], -100)

    def test_manual_get_pres_repair_uses_pres_oid_not_step_oid(self):
        # documented, deliberate deviation from the reference: reference's own
        # ck_and_manual_get_pres (line 641) sends self.manual_get_pres_cmd.send([self.step_oid,
        # i]) - a real copy-paste bug in the published source (manual_get_pres is registered
        # under pres_oid, config_pres_prtouch/add_pres_prtouch section). This test proves our
        # port uses the corrected oid, not the original's bug.
        _, mcu, pv2 = _build()
        seen_oids = []

        def provider(call):
            seen_oids.append(call.args[0])
            return {
                'oid': pv2.mcu.pres_oid, 'index': call.args[1], 'tri_time': 0,
                'tri_chs': 0, 'buf_cnt': 0,
                'tick_0': 0, 'ch0_0': 0, 'ch1_0': 0, 'ch2_0': 0, 'ch3_0': 0,
                'tick_1': 0, 'ch0_1': 0, 'ch1_1': 0, 'ch2_1': 0, 'ch3_1': 0,
            }
        mcu.set_query_response('manual_get_pres', provider)
        pv2.mcu.collect_pres_samples(0.0)  # empty buffer, immediate timeout -> forces repair
        self.assertTrue(seen_oids)
        self.assertTrue(all(oid == pv2.mcu.pres_oid for oid in seen_oids),
                         "repair must query pres_oid (the corrected value), not step_oid")


if __name__ == '__main__':
    unittest.main()
