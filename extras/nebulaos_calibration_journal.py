# nebulaos_calibration_journal.py - persistent transaction journal for
# NebulaOS guided calibration workflows (Phase 2 mission, §12).
#
# Pure stdlib, zero Klipper dependencies - deliberately, matching
# nebulaos_plr_journal.py's own stated design principle (usable standalone,
# unit-testable against a plain path with no Klipper printer object in
# sight). Not the same journal as that module: nebulaos_plr_journal.py is a
# fixed-16-byte-record EEPROM ring buffer for power-loss file-position
# recovery; this is a single, larger, human-readable JSON document for
# calibration workflow progress, because the persistence target here is
# /usr/data (plain filesystem), not a wear-limited I2C EEPROM, and the
# fields this mission's own §12 asks for (workflow, stage, completed
# stages, expected persisted values, ...) are naturally a JSON object, not
# a fixed-width record.
#
# Atomic write pattern reused from nebulaos_power_loss_recovery.py's own
# atomic_write_json() (write .tmp, fsync, os.replace, fsync parent dir) -
# duplicated rather than imported, matching this codebase's own stated
# preference elsewhere for small, independent modules over cross-imports
# between otherwise-unrelated subsystems.
#
# Why a journal at all: NEBULAOS_AUTO_CALIBRATE's own SAVE_CONFIG call both
# writes printer.cfg AND immediately restarts klippy in the same
# operation - there is no code path that runs "after SAVE_CONFIG, in the
# same process" to confirm it worked. If klippy crashes, hangs, or the MCU
# reports a fault (see this mission's own operational-learnings note about
# restarting with active heaters) anywhere between that SAVE_CONFIG call
# and the NEXT klippy boot's own verification, the ONLY durable record that
# a calibration was in flight - and whether it reached the point of
# writing new values to disk - is this journal file, written just before
# SAVE_CONFIG is called (not after - there is no "after" in the same
# process).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os

SCHEMA_VERSION = 1

DEFAULT_JOURNAL_DIR = "/usr/data/nebulaos/printer_data/calibration"
DEFAULT_JOURNAL_PATH = DEFAULT_JOURNAL_DIR + "/auto_calibrate_journal.json"

# Workflow stage identifiers, in the exact order mission §11 specifies.
# Exposed as a plain tuple (not an enum - this module targets Python
# versions without guaranteed enum availability, matching the rest of this
# codebase's stdlib-only style) so callers can validate/iterate.
STAGES = (
    'preflight',
    'home',
    'pid_bed',
    'pid_hotend',
    'nozzle_clean',
    'establish_thermal_state',
    'stabilize',
    'localized_z_offset',
    'bed_mesh',
    'final_validation',
    'commit',
    'restart',
    'post_restart_verification',
)

# Journal-level state (distinct from STAGES: state is the ENGINE's own
# transaction status; stage is WHICH step of the workflow it refers to).
STATE_RUNNING = 'running'
STATE_COMMIT_REQUESTED = 'commit_requested'
STATE_RESTART_PENDING = 'restart_pending'
STATE_VERIFICATION_PENDING = 'verification_pending'
STATE_COMPLETE = 'complete'
STATE_ERROR = 'error'
STATE_CANCELLED = 'cancelled'


def new_journal(calibration_id, workflow, now):
    """A fresh journal record for a workflow that is just starting. `now`
    is a caller-supplied timestamp (float epoch seconds, or any
    JSON-serializable value the caller consistently uses) - this module
    never calls time.time() itself, so tests can drive it with fixed
    values and the real caller can use Klipper's own reactor.monotonic()
    or time.time() as it prefers."""
    return {
        'schema_version': SCHEMA_VERSION,
        'calibration_id': calibration_id,
        'workflow': workflow,
        'stage': None,
        'state': STATE_RUNNING,
        'started_at': now,
        'updated_at': now,
        'completed_stages': [],
        'expected_values': {},
        'commit_requested': False,
        'restart_pending': False,
        'verification_pending': False,
        'result': None,
        'error': None,
    }


def advance_stage(journal, stage, now):
    """Mark `stage` as the current stage and append the PREVIOUS current
    stage (if any) to completed_stages - called once per stage transition,
    BEFORE that stage's own work runs, so a journal read mid-crash always
    shows the stage that was in progress, not the last one that finished."""
    if stage not in STAGES:
        raise ValueError("unknown calibration stage %r" % (stage,))
    prev = journal.get('stage')
    if prev is not None and prev not in journal['completed_stages']:
        journal['completed_stages'].append(prev)
    journal['stage'] = stage
    journal['updated_at'] = now
    return journal


def mark_commit_requested(journal, expected_values, now):
    """Called immediately before the single SAVE_CONFIG call - this is the
    LAST write this module makes before control leaves this process (the
    save+restart happens inside the same gcode call but this write must
    land on disk first). expected_values is the small, plain dict of
    persisted-config values (e.g. {'bltouch.z_offset': 1.234,
    'bed_mesh.profile': 'default'}) post-restart verification will check
    against the real, reloaded config."""
    advance_stage(journal, 'commit', now)
    journal['state'] = STATE_COMMIT_REQUESTED
    journal['commit_requested'] = True
    journal['restart_pending'] = True
    journal['verification_pending'] = True
    journal['expected_values'] = dict(expected_values)
    return journal


def mark_verification_result(journal, success, result, error, now):
    """Called from the NEXT klippy boot's own post-restart verification
    (mission §11's last stage) - the only code that ever clears
    verification_pending. success=False keeps restart_pending/
    verification_pending visible in the resulting state (STATE_ERROR) so a
    caller polling status can tell "committed but verification found a
    mismatch" apart from every earlier failure mode, which all leave
    commit_requested/restart_pending/verification_pending False (nothing
    was ever staged)."""
    advance_stage(journal, 'post_restart_verification', now)
    journal['restart_pending'] = False
    journal['verification_pending'] = False
    journal['state'] = STATE_COMPLETE if success else STATE_ERROR
    journal['result'] = result
    journal['error'] = error
    return journal


def mark_error(journal, error, now):
    """Called on any failure BEFORE mark_commit_requested() ever ran -
    commit_requested/restart_pending/verification_pending are left False
    (whatever they already were - never forced True here), because nothing
    was ever staged to disk on this path; Klipper's own in-memory
    configfile state is simply discarded when this process's caller
    restarts or the config is next reloaded, with no NebulaOS action
    required to "roll back" it."""
    journal['state'] = STATE_ERROR
    journal['error'] = error
    journal['updated_at'] = now
    return journal


def mark_cancelled(journal, now):
    journal['state'] = STATE_CANCELLED
    journal['updated_at'] = now
    return journal


def atomic_write_json(tmp_path, final_path, data):
    """Write .tmp, flush + fsync(file), os.replace (atomic rename),
    fsync(parent directory) - identical pattern to
    nebulaos_power_loss_recovery.py's own atomic_write_json()."""
    payload = json.dumps(data, sort_keys=True).encode('utf-8')
    with open(tmp_path, 'wb') as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, final_path)
    parent = os.path.dirname(final_path) or "."
    parent_fd = os.open(parent, os.O_RDONLY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def write_journal(journal, path=DEFAULT_JOURNAL_PATH):
    """Ensures the journal directory exists, then writes atomically."""
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    atomic_write_json(path + ".tmp", path, journal)


def read_journal(path=DEFAULT_JOURNAL_PATH):
    """Returns the parsed journal dict, or None if the file is missing or
    torn (unparseable) - both treated identically, matching
    nebulaos_power_loss_recovery.py's own read_sidecar() convention: no
    recovery/verification is possible from an unreadable journal, so a
    caller must treat None the same as "no calibration was ever run"."""
    try:
        with open(path, 'rb') as handle:
            payload = handle.read()
    except (IOError, OSError):
        return None
    try:
        data = json.loads(payload.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(data, dict) or data.get('schema_version') != SCHEMA_VERSION:
        return None
    return data
