# Async-response subscription tests - covers this module set's adaptation to mainline
# Klipper's MCU.register_serial_response(), which replaced MCU.register_response() in
# upstream commit c89393cda (2026-02-26, "mcu: Rework mcu.register_response() to
# mcu.register_serial_response()").
#
# The rename is the least of it. The three real behaviour changes this file exists to pin
# down are: the second argument is now a full message FORMAT rather than a message name; the
# format is validated against the MCU's own declared dictionary; and registration is deferred
# until after MCU config finalization. prtouch_mcu.py therefore had to move its three
# subscriptions out of __init__ (no serial connection exists at config-parse time, so there
# is no dictionary to validate against) and into its existing per-MCU config callbacks.
#
# The single most important property asserted here is the fail-closed one: if the connected
# MCU declares none of the known formats for a probe-telemetry message, PRTouch must refuse
# to start. A load-cell probe that descends with a telemetry callback which can never fire is
# a nozzle being driven into the bed with no trigger path.
#
# Run from klippy/: python3 -m unittest extras.test_prtouch_serial_response -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest

from . import prtouch_mcu
from . import prtouch_test_support as fake
from . import prtouch_v2


ASYNC_SUBSCRIPTIONS = (
    ('result_run_step_prtouch', prtouch_mcu.RESULT_RUN_STEP_PRTOUCH),
    ('result_run_pres_prtouch', prtouch_mcu.RESULT_RUN_PRES_PRTOUCH),
    ('result_read_pres_prtouch', prtouch_mcu.RESULT_READ_PRES_PRTOUCH),
)


def _build_unconnected():
    """PRTouchV2 constructed, but neither config callbacks nor klippy:connect run yet -
    i.e. exactly the state real Klipper is in while it is still parsing printer.cfg."""
    printer, mcu, pins, values = fake.build_environment()
    config = fake.make_prtouch_v2_config(printer, pins, values)
    pv2 = prtouch_v2.PRTouchV2(config)
    return printer, mcu, pv2


class FormatParamNamesTest(unittest.TestCase):
    """format_param_names() is what decides whether a format the MCU declares actually
    carries every field this module's handlers read, so its parsing has to match Klipper's
    own msgproto convention exactly."""

    def test_extracts_names_in_order_excluding_message_name(self):
        self.assertEqual(
            prtouch_mcu.format_param_names('result_x oid=%c tri_time=%u step0=%i'),
            ['oid', 'tri_time', 'step0'])

    def test_message_name_alone_yields_no_params(self):
        self.assertEqual(prtouch_mcu.format_param_names('result_x'), [])

    def test_tolerates_surrounding_whitespace(self):
        self.assertEqual(
            prtouch_mcu.format_param_names('  result_x oid=%c  '), ['oid'])


class CandidateFormatTableTest(unittest.TestCase):
    """The candidate tables are the contract with Creality's proprietary MCU dictionary.
    These assertions keep them internally consistent, which is checkable offline, even though
    the dictionary itself is not."""

    def test_every_candidate_names_its_own_message(self):
        for msgname, (_required, candidates) in ASYNC_SUBSCRIPTIONS:
            self.assertTrue(candidates, "%s has no candidate formats" % msgname)
            for fmt in candidates:
                self.assertEqual(fmt.split()[0], msgname)

    def test_every_candidate_declares_every_required_field(self):
        # A candidate that omits a handler-required field can never be selected, because
        # select_response_format() rejects it - so listing one is dead weight that reads like
        # a working fallback. This assertion exists because an earlier revision of the step
        # table did exactly that, and this test is what caught it.
        for msgname, (required, candidates) in ASYNC_SUBSCRIPTIONS:
            for fmt in candidates:
                declared = set(prtouch_mcu.format_param_names(fmt))
                for name in required:
                    self.assertIn(name, declared,
                                  "%s: candidate omits required field %s: %s"
                                  % (msgname, name, fmt))

    def test_every_candidate_carries_an_oid_field(self):
        # All three subscriptions are registered with an oid, and Klipper's serial layer
        # keys handlers on (name, oid), so a format without an oid field could never be
        # dispatched to the right subscriber.
        for msgname, (_required, candidates) in ASYNC_SUBSCRIPTIONS:
            for fmt in candidates:
                self.assertIn('oid', prtouch_mcu.format_param_names(fmt), fmt)


class SelectResponseFormatTest(unittest.TestCase):
    def test_selects_the_first_candidate_the_mcu_declares(self):
        mcu = fake.FakeMCU()
        for msgname, subscription in ASYNC_SUBSCRIPTIONS:
            chosen = prtouch_mcu.select_response_format(mcu, subscription, msgname)
            self.assertEqual(chosen, subscription[1][0])

    def test_falls_back_to_a_later_candidate(self):
        # Models an MCU whose result_run_pres_prtouch omits the index field: the first
        # candidate is not declared, the second is, and the second still carries every field
        # _handle_result_run_pres_prtouch reads (that handler never reads index).
        _required, candidates = prtouch_mcu.RESULT_RUN_PRES_PRTOUCH
        self.assertGreaterEqual(len(candidates), 2)
        mcu = fake.FakeMCU()
        mcu.valid_response_formats = {candidates[1]}
        chosen = prtouch_mcu.select_response_format(
            mcu, prtouch_mcu.RESULT_RUN_PRES_PRTOUCH, 'result_run_pres_prtouch')
        self.assertEqual(chosen, candidates[1])

    def test_rejects_a_declared_format_missing_a_handler_required_field(self):
        # The MCU happily declares this format - but it does not carry tri_time, which
        # _handle_result_run_step_prtouch reads unconditionally. Registering it would turn
        # the first real probe sample into a KeyError mid-descent, so it must not be chosen.
        truncated = 'result_run_step_prtouch oid=%c index=%c'
        mcu = fake.FakeMCU()
        mcu.valid_response_formats = {truncated}
        chosen = prtouch_mcu.select_response_format(
            mcu, prtouch_mcu.RESULT_RUN_STEP_PRTOUCH, 'result_run_step_prtouch')
        self.assertIsNone(chosen)

    def test_returns_none_when_the_mcu_declares_nothing(self):
        mcu = fake.FakeMCU()
        mcu.valid_response_formats = set()
        for msgname, subscription in ASYNC_SUBSCRIPTIONS:
            self.assertIsNone(
                prtouch_mcu.select_response_format(mcu, subscription, msgname))


class SubscriptionLifecycleTest(unittest.TestCase):
    """Where the subscriptions happen is itself the adaptation - mainline validates the
    format against a dictionary that does not exist during config parsing."""

    def test_no_subscription_happens_during_config_parsing(self):
        _printer, mcu, _pv2 = _build_unconnected()
        self.assertEqual(mcu.response_handlers, {},
                         "async responses must not be registered before the MCU has "
                         "identified - there is no dictionary to validate a format against")

    def test_all_three_subscribe_once_config_callbacks_run(self):
        printer, mcu, _pv2 = _build_unconnected()
        fake.connect(printer, mcu)
        for msgname, _subscription in ASYNC_SUBSCRIPTIONS:
            self.assertIn(msgname, mcu.response_handlers,
                          "%s was never subscribed" % msgname)

    def test_registered_with_the_full_format_not_the_bare_name(self):
        printer, mcu, _pv2 = _build_unconnected()
        fake.connect(printer, mcu)
        for msgname, subscription in ASYNC_SUBSCRIPTIONS:
            registered = mcu.registered_response_formats[msgname]
            self.assertEqual(registered, subscription[1][0])
            self.assertNotEqual(registered, msgname)

    def test_subscribed_under_the_correct_oids(self):
        printer, mcu, pv2 = _build_unconnected()
        fake.connect(printer, mcu)
        self.assertIn(pv2.mcu.step_oid,
                      mcu.response_handlers['result_run_step_prtouch'])
        self.assertIn(pv2.mcu.pres_oid,
                      mcu.response_handlers['result_run_pres_prtouch'])
        self.assertIn(pv2.mcu.pres_oid,
                      mcu.response_handlers['result_read_pres_prtouch'])

    def test_wrapper_objects_are_retained_and_can_unregister(self):
        printer, mcu, pv2 = _build_unconnected()
        fake.connect(printer, mcu)
        for wrapper in (pv2.mcu.step_response, pv2.mcu.pres_run_response,
                        pv2.mcu.pres_read_response):
            self.assertIsNotNone(wrapper)
        pv2.mcu.step_response.unregister()
        self.assertNotIn(pv2.mcu.step_oid,
                         mcu.response_handlers['result_run_step_prtouch'])

    def test_delivered_samples_still_reach_the_real_handler(self):
        # End-to-end: the subscription registered through the new API dispatches a real
        # payload into the real production handler, and the parsed result is what the rest
        # of PRTouch consumes.
        printer, mcu, pv2 = _build_unconnected()
        fake.connect(printer, mcu)
        chunks = fake.make_step_result(
            pv2.mcu.step_oid, 12345,
            [(1000, 10), (2000, 20), (3000, 30), (4000, 40)])
        for chunk in chunks:
            mcu.push_response('result_run_step_prtouch', pv2.mcu.step_oid, chunk)
        self.assertEqual(len(pv2.mcu.step_res), 4)
        self.assertEqual(pv2.mcu.step_res[0]['step'], 10)


class FailClosedTest(unittest.TestCase):
    """An MCU whose dictionary this build does not recognise must stop Klippy, not degrade."""

    def _connect_with_formats(self, formats):
        printer, mcu, _pv2 = _build_unconnected()
        mcu.valid_response_formats = set(formats)
        return printer, mcu

    def test_unknown_dictionary_refuses_to_start(self):
        printer, mcu = self._connect_with_formats([])
        with self.assertRaises(fake.ConfigError):
            fake.connect(printer, mcu)

    def test_failure_names_the_message_and_the_attempted_formats(self):
        printer, mcu = self._connect_with_formats([])
        try:
            fake.connect(printer, mcu)
        except fake.ConfigError as exc:
            msg = str(exc)
        else:
            self.fail("expected a ConfigError")
        # The first subscription attempted is the step one, so that is the message named.
        self.assertIn('result_run_step_prtouch', msg)
        self.assertIn('tri_time', msg)
        self.assertIn(prtouch_mcu.RESULT_RUN_STEP_PRTOUCH[1][0], msg)
        self.assertIn('console.py', msg)

    def test_a_partial_dictionary_still_refuses(self):
        # Step telemetry is fine, pressure telemetry is not. This must still refuse: a
        # descent driven with working step counts and no load-cell readings is precisely the
        # unsafe case.
        step_ok = prtouch_mcu.RESULT_RUN_STEP_PRTOUCH[1][0]
        printer, mcu = self._connect_with_formats([step_ok])
        with self.assertRaises(fake.ConfigError) as ctx:
            fake.connect(printer, mcu)
        self.assertIn('result_run_pres_prtouch', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
