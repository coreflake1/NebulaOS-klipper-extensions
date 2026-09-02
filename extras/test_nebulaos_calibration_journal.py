# Tests for extras/nebulaos_calibration_journal.py (Phase 2 mission §12).
#
# Pure stdlib module - no Klipper/fake.py dependency, no package-relative
# import needed either (mirrors test_nebulaos_plr_journal.py's own
# convention: run as `python3 test_nebulaos_calibration_journal.py` from
# extras/, or via unittest discovery either style, since it uses a bare
# `import nebulaos_calibration_journal` that resolves the same way from
# either invocation location this repo documents).
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_calibration_journal -v
import os
import shutil
import tempfile
import unittest

try:
    from . import nebulaos_calibration_journal as journal
except (ImportError, ValueError):
    import nebulaos_calibration_journal as journal


class NewJournalTest(unittest.TestCase):
    def test_fresh_journal_has_expected_shape(self):
        j = journal.new_journal(1, 'auto_calibrate', now=1000.0)
        self.assertEqual(j['schema_version'], journal.SCHEMA_VERSION)
        self.assertEqual(j['calibration_id'], 1)
        self.assertEqual(j['workflow'], 'auto_calibrate')
        self.assertIsNone(j['stage'])
        self.assertEqual(j['state'], journal.STATE_RUNNING)
        self.assertEqual(j['started_at'], 1000.0)
        self.assertEqual(j['updated_at'], 1000.0)
        self.assertEqual(j['completed_stages'], [])
        self.assertEqual(j['expected_values'], {})
        self.assertFalse(j['commit_requested'])
        self.assertFalse(j['restart_pending'])
        self.assertFalse(j['verification_pending'])
        self.assertIsNone(j['result'])
        self.assertIsNone(j['error'])


class AdvanceStageTest(unittest.TestCase):
    def test_first_stage_has_no_completed_predecessor(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.advance_stage(j, 'preflight', now=1.0)
        self.assertEqual(j['stage'], 'preflight')
        self.assertEqual(j['completed_stages'], [])
        self.assertEqual(j['updated_at'], 1.0)

    def test_advancing_marks_previous_stage_completed(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.advance_stage(j, 'preflight', now=1.0)
        journal.advance_stage(j, 'home', now=2.0)
        self.assertEqual(j['stage'], 'home')
        self.assertEqual(j['completed_stages'], ['preflight'])

    def test_completed_stages_accumulate_in_order(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        for i, stage in enumerate(journal.STAGES):
            journal.advance_stage(j, stage, now=float(i))
        self.assertEqual(list(journal.STAGES[:-1]), j['completed_stages'])
        self.assertEqual(j['stage'], journal.STAGES[-1])

    def test_unknown_stage_rejected(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        with self.assertRaises(ValueError):
            journal.advance_stage(j, 'not_a_real_stage', now=1.0)

    def test_input_shaper_stages_accepted_with_explicit_stages_arg(self):
        j = journal.new_journal(1, 'input_shaper_calibrate', now=0.0)
        journal.advance_stage(j, 'measure', now=1.0,
                               stages=journal.INPUT_SHAPER_STAGES)
        self.assertEqual(j['stage'], 'measure')

    def test_input_shaper_stage_rejected_against_default_stages(self):
        j = journal.new_journal(1, 'input_shaper_calibrate', now=0.0)
        with self.assertRaises(ValueError):
            journal.advance_stage(j, 'measure', now=1.0)

    def test_auto_calibrate_stage_rejected_against_input_shaper_stages(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        with self.assertRaises(ValueError):
            journal.advance_stage(j, 'pid_bed', now=1.0,
                                   stages=journal.INPUT_SHAPER_STAGES)

    def test_input_shaper_stages_full_walk_accumulates_in_order(self):
        j = journal.new_journal(1, 'input_shaper_calibrate', now=0.0)
        for i, stage in enumerate(journal.INPUT_SHAPER_STAGES):
            journal.advance_stage(j, stage, now=float(i),
                                   stages=journal.INPUT_SHAPER_STAGES)
        self.assertEqual(
            list(journal.INPUT_SHAPER_STAGES[:-1]), j['completed_stages'])
        self.assertEqual(j['stage'], journal.INPUT_SHAPER_STAGES[-1])


class CommitAndVerificationTest(unittest.TestCase):
    def test_mark_commit_requested_sets_all_three_flags(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.advance_stage(j, 'final_validation', now=1.0)
        journal.mark_commit_requested(
            j, {'bltouch.z_offset': 1.234}, now=2.0)
        self.assertEqual(j['state'], journal.STATE_COMMIT_REQUESTED)
        self.assertTrue(j['commit_requested'])
        self.assertTrue(j['restart_pending'])
        self.assertTrue(j['verification_pending'])
        self.assertEqual(j['expected_values'], {'bltouch.z_offset': 1.234})
        self.assertEqual(j['stage'], 'commit')
        # 'final_validation' (the PREVIOUS stage) is now completed;
        # 'commit' itself is the current stage, not yet completed - it
        # only lands in completed_stages once mark_verification_result()
        # advances past it.
        self.assertIn('final_validation', j['completed_stages'])
        self.assertNotIn('commit', j['completed_stages'])

    def test_successful_verification_clears_pending_flags(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.mark_commit_requested(j, {'bltouch.z_offset': 1.234}, now=1.0)
        journal.mark_verification_result(
            j, success=True, result={'bltouch.z_offset': 1.234},
            error=None, now=2.0)
        self.assertEqual(j['state'], journal.STATE_COMPLETE)
        self.assertFalse(j['restart_pending'])
        self.assertFalse(j['verification_pending'])
        self.assertEqual(j['result'], {'bltouch.z_offset': 1.234})
        self.assertIsNone(j['error'])
        self.assertEqual(j['stage'], 'post_restart_verification')
        self.assertIn('commit', j['completed_stages'])

    def test_failed_verification_is_a_distinct_error_state(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.mark_commit_requested(j, {'bltouch.z_offset': 1.234}, now=1.0)
        journal.mark_verification_result(
            j, success=False, result=None,
            error='persisted z_offset does not match expected value',
            now=2.0)
        self.assertEqual(j['state'], journal.STATE_ERROR)
        # Both flags still cleared even on failure - this state is
        # reached and does not loop back into "still pending".
        self.assertFalse(j['restart_pending'])
        self.assertFalse(j['verification_pending'])
        self.assertIsNotNone(j['error'])

    def test_mark_error_before_commit_leaves_commit_flags_false(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.advance_stage(j, 'pid_bed', now=1.0)
        journal.mark_error(j, 'PID_CALIBRATE failed: heater fault', now=2.0)
        self.assertEqual(j['state'], journal.STATE_ERROR)
        self.assertFalse(j['commit_requested'])
        self.assertFalse(j['restart_pending'])
        self.assertFalse(j['verification_pending'])
        self.assertEqual(j['error'], 'PID_CALIBRATE failed: heater fault')

    def test_mark_cancelled(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.advance_stage(j, 'pid_bed', now=1.0)
        journal.mark_cancelled(j, now=2.0)
        self.assertEqual(j['state'], journal.STATE_CANCELLED)


class ReadWriteRoundTripTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.path = os.path.join(self.tmpdir, 'sub', 'journal.json')

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_write_creates_missing_directory(self):
        self.assertFalse(os.path.isdir(os.path.dirname(self.path)))
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.write_journal(j, path=self.path)
        self.assertTrue(os.path.isfile(self.path))

    def test_round_trip_preserves_content(self):
        j = journal.new_journal(7, 'auto_calibrate', now=123.5)
        journal.advance_stage(j, 'pid_bed', now=124.0)
        journal.write_journal(j, path=self.path)
        loaded = journal.read_journal(path=self.path)
        self.assertEqual(loaded, j)

    def test_no_tmp_file_left_behind_after_write(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.write_journal(j, path=self.path)
        self.assertFalse(os.path.isfile(self.path + '.tmp'))

    def test_missing_file_returns_none(self):
        self.assertIsNone(journal.read_journal(path=self.path))

    def test_torn_file_returns_none(self):
        os.makedirs(os.path.dirname(self.path))
        with open(self.path, 'wb') as handle:
            handle.write(b'{not valid json')
        self.assertIsNone(journal.read_journal(path=self.path))

    def test_wrong_schema_version_returns_none(self):
        j = journal.new_journal(1, 'auto_calibrate', now=0.0)
        j['schema_version'] = 999
        journal.write_journal(j, path=self.path)
        self.assertIsNone(journal.read_journal(path=self.path))

    def test_second_write_overwrites_first(self):
        j1 = journal.new_journal(1, 'auto_calibrate', now=0.0)
        journal.write_journal(j1, path=self.path)
        j2 = journal.new_journal(2, 'auto_calibrate', now=1.0)
        journal.write_journal(j2, path=self.path)
        loaded = journal.read_journal(path=self.path)
        self.assertEqual(loaded['calibration_id'], 2)


if __name__ == '__main__':
    unittest.main()
