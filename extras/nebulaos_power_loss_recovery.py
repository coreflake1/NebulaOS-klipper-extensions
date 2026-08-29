# nebulaos_power_loss_recovery.py - NebulaOS's own power-loss recovery
# extra (Phase 1.9B), built entirely on public/mainline Klipper interfaces.
#
# Persistence has two tiers:
#   - A tiny (16-byte) durable record on the physical BL24C16F EEPROM, owned
#     in production by the Linux 6.6 in-tree at24/nvmem driver (see
#     nebulaos_plr_journal.py's own header for the full physical layout and
#     why [bl24c16f]/i2c-chardev are NOT used here) - generation counter +
#     file_position + a CRC, durable across genuine power loss because it is
#     the only thing this module trusts as ground truth for "did a
#     checkpoint really land".
#   - A larger, human-readable JSON "sidecar" file on /usr/data (dual A/B
#     files, selected by the EEPROM generation's parity) holding the full
#     recoverable gcode/motion/thermal/etc state. The EEPROM record is
#     written LAST, only after the sidecar write is fully durable (fsync'd
#     file, atomically renamed, fsync'd parent directory) - see
#     _perform_checkpoint_blocking() for the exact ordering this enforces.
#
# No Klipper core patches. No private Klipper internals - every field this
# module reads comes from a public get_status() dict; every field it
# restores is set back through a standard, public gcode command. See
# docs/ (added alongside this mission) for the full design rationale.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import hashlib
import json
import logging
import os

from . import nebulaos_plr_journal as journal

DEFAULT_EEPROM_PATH = "/sys/bus/i2c/devices/2-0050/eeprom"
DEFAULT_SIDECAR_DIR = "/usr/data/nebulaos/printer_data/plr"
DEFAULT_CHECKPOINT_INTERVAL = 5.0
DEFAULT_POLL_INTERVAL = 0.75
DEFAULT_MIN_Z_FOR_START = 0.6

SCHEMA_VERSION = 1

SIDECAR_FILENAMES = ("state-a.json", "state-b.json")

_HASH_CHUNK_SIZE = 1024 * 1024


# ---------------------------------------------------------------------------
# Pure logic - no Klipper objects, no I/O side effects beyond what's passed
# in explicitly. Every function here is exercised directly by
# test_nebulaos_power_loss_recovery.py with plain dicts/tmp files, with zero
# printer/reactor/hardware involved.
# ---------------------------------------------------------------------------

def meaningful_progress(print_stats_status, virtual_sdcard_status,
                         toolhead_status, min_z):
    """Gate for starting persistent checkpointing at all - deliberately NOT
    Creality's layer-comment/G1-count approach, just simple, public,
    already-available status fields."""
    if print_stats_status.get('state') != 'printing':
        return False
    if not virtual_sdcard_status.get('is_active'):
        return False
    if virtual_sdcard_status.get('file_position', 0) <= 0:
        return False
    if print_stats_status.get('print_duration', 0) <= 0:
        return False
    position = toolhead_status.get('position')
    z = position.z if position is not None else 0.0
    if z <= min_z:
        return False
    return True


def sidecar_path_for_generation(sidecar_dir, generation):
    index = journal.sidecar_parity(generation)
    return os.path.join(sidecar_dir, SIDECAR_FILENAMES[index])


def coord_to_list(coord):
    """Converts a Klipper gcode_move `Coord` (a namedtuple-like object with
    .x/.y/.z/.e) into a plain JSON-serializable [x, y, z, e] list."""
    return [coord.x, coord.y, coord.z, coord.e]


def build_sidecar_state(generation, file_info, file_position,
                         gcode_move_status, toolhead_status,
                         extruder_status, bed_mesh_status,
                         exclude_object_status, fan_status,
                         firmware_retraction_status=None):
    """Pure assembly of the "Sidecar V1 State" schema from already-fetched
    get_status() dicts (or plain equivalents in tests). Never touches
    gcode_move.base_position or any other private attribute - everything
    here is a documented public get_status() key."""
    state = {
        "schema_version": SCHEMA_VERSION,
        "generation": generation,
        "file": {
            "path": file_info["path"],
            "size": file_info["size"],
            "sha256": file_info["sha256"],
        },
        "file_position": file_position,
        "gcode": {
            "gcode_position": coord_to_list(gcode_move_status["gcode_position"]),
            "absolute_coordinates": gcode_move_status["absolute_coordinates"],
            "absolute_extrude": gcode_move_status["absolute_extrude"],
            "speed_factor": gcode_move_status["speed_factor"],
            "extrude_factor": gcode_move_status["extrude_factor"],
            "homing_origin": coord_to_list(gcode_move_status["homing_origin"]),
        },
        "motion": {
            "max_velocity": toolhead_status["max_velocity"],
            "max_accel": toolhead_status["max_accel"],
            "minimum_cruise_ratio": toolhead_status["minimum_cruise_ratio"],
            "square_corner_velocity": toolhead_status["square_corner_velocity"],
        },
        "thermal": {
            "extruder_target": extruder_status.get("target", 0.0),
            "bed_target": extruder_status.get("bed_target", 0.0),
        },
        "fan": {
            "speed": fan_status.get("speed", 0.0),
        },
        "extruder": {
            "pressure_advance": extruder_status.get("pressure_advance", 0.0),
            "smooth_time": extruder_status.get("smooth_time", 0.0),
        },
        "bed_mesh": {
            "profile_name": bed_mesh_status.get("profile_name", ""),
        },
        "exclude_object": {
            "objects": exclude_object_status.get("objects", []),
            "excluded_objects": exclude_object_status.get("excluded_objects", []),
        },
    }
    if firmware_retraction_status is not None:
        state["firmware_retraction"] = {
            "retract_length": firmware_retraction_status.get("retract_length", 0.0),
            "retract_speed": firmware_retraction_status.get("retract_speed", 0.0),
            "unretract_extra_length":
                firmware_retraction_status.get("unretract_extra_length", 0.0),
            "unretract_speed": firmware_retraction_status.get("unretract_speed", 0.0),
        }
    return state


def hash_file(path, chunk_size=_HASH_CHUNK_SIZE):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(tmp_path, final_path, data):
    """Checkpoint transaction steps 2-5, exactly as specified:
    write .tmp, flush + fsync(file), os.replace (atomic rename),
    fsync(parent directory). Runs on a background thread via aio_executor -
    this function itself is plain blocking I/O, deliberately."""
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


def read_sidecar(path):
    """Returns the parsed sidecar dict, or None if the file is missing or
    torn (unparseable) - both are treated identically by callers (no
    recovery available from this slot), never raised as an error."""
    try:
        with open(path, 'rb') as handle:
            payload = handle.read()
    except (IOError, OSError):
        return None
    try:
        return json.loads(payload.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return None


class IntegrityFailure(Exception):
    """Raised by validate_recovery() for any integrity failure. ALLOW_UNSAFE
    may NEVER suppress this - it only ever exists to report a reason
    string, and validate_recovery() never even looks at ALLOW_UNSAFE except
    for the one dedicated position-safety check."""
    def __init__(self, reason):
        super(IntegrityFailure, self).__init__(reason)
        self.reason = reason


def validate_recovery(eeprom_record, sidecar_state, current_file_path,
                       current_file_size, current_file_sha256,
                       allow_unsafe):
    """Runs the full validation chain from the mission's File Identity and
    Physical Position Policy sections, in order, and returns the validated
    sidecar_state dict on success.

    Raises IntegrityFailure for any of: missing/tombstoned EEPROM state,
    missing/corrupt sidecar, schema mismatch, stale sidecar generation,
    missing gcode file path, file size/sha256/path mismatch. These can
    NEVER be bypassed by allow_unsafe.

    Raises PositionUnsafe (a distinct exception) ONLY for the physical
    Z-position-uncertainty gate, which allow_unsafe=True is permitted to
    override - and only that gate.
    """
    if eeprom_record is None:
        raise IntegrityFailure("no EEPROM checkpoint available")
    if sidecar_state is None:
        raise IntegrityFailure("sidecar missing or unreadable (torn/absent)")
    if sidecar_state.get("schema_version") != SCHEMA_VERSION:
        raise IntegrityFailure("sidecar schema_version mismatch")
    if sidecar_state.get("generation") != eeprom_record.generation:
        raise IntegrityFailure(
            "stale sidecar generation (sidecar=%r, eeprom=%r)"
            % (sidecar_state.get("generation"), eeprom_record.generation))

    file_info = sidecar_state.get("file") or {}
    saved_path = file_info.get("path")
    saved_size = file_info.get("size")
    saved_sha256 = file_info.get("sha256")
    if not saved_path:
        raise IntegrityFailure("sidecar has no recorded gcode file path")
    if (saved_path != current_file_path or saved_size != current_file_size
            or saved_sha256 != current_file_sha256):
        raise IntegrityFailure(
            "file identity mismatch (path/size/sha256 changed since checkpoint)")

    # Every integrity gate above is unconditional. Only the physical
    # position-safety gate below is subject to allow_unsafe, and it is
    # checked LAST, deliberately, so a caller can never short-circuit past
    # an integrity failure by also passing allow_unsafe=True.
    if not allow_unsafe:
        raise PositionUnsafe(
            "physical Z position cannot be proven safe after power loss; "
            "resume refused (NEBULAOS_PLR_RESUME ALLOW_UNSAFE=1 to override "
            "this specific check only)")

    return sidecar_state


class PositionUnsafe(Exception):
    def __init__(self, reason):
        super(PositionUnsafe, self).__init__(reason)
        self.reason = reason


def build_resume_gcode_lines(state):
    """Builds the standard-command restoration sequence from the mission's
    "Public Klipper Resume Path" section. Pure string assembly - no gcode
    is actually run here, the caller feeds this list to
    run_script_from_command().

    Deliberately excludes M24 and any G28/motion command: this mission
    does not implement or hardware-qualify physical position recovery, so
    the toolhead is never moved and printing is never resumed
    automatically by this function - see the module docstring and the
    mission's own "Physical Position Policy" section. The caller is
    responsible for M24 as a clearly separate, later, manual step once a
    real physical-position-recovery design exists.
    """
    gcode_state = state["gcode"]
    motion = state["motion"]
    thermal = state["thermal"]
    extruder = state["extruder"]
    fan = state["fan"]
    bed_mesh = state["bed_mesh"]
    file_info = state["file"]

    lines = []
    lines.append("M23 %s" % file_info["path"])

    for name in state["exclude_object"].get("objects", []):
        lines.append("EXCLUDE_OBJECT_DEFINE NAME=%s" % name)
    for name in state["exclude_object"].get("excluded_objects", []):
        lines.append("EXCLUDE_OBJECT NAME=%s" % name)

    lines.append("M26 S%d" % state["file_position"])

    lines.append("G90" if gcode_state["absolute_coordinates"] else "G91")
    lines.append("M82" if gcode_state["absolute_extrude"] else "M83")
    lines.append("M220 S%g" % (gcode_state["speed_factor"] * 100.0))
    lines.append("M221 S%g" % (gcode_state["extrude_factor"] * 100.0))

    homing_origin = gcode_state["homing_origin"]
    lines.append("SET_GCODE_OFFSET Z=%g MOVE=0" % homing_origin[2])
    lines.append("G92 E%g" % gcode_state["gcode_position"][3])

    lines.append(
        "SET_VELOCITY_LIMIT VELOCITY=%g ACCEL=%g "
        "MINIMUM_CRUISE_RATIO=%g SQUARE_CORNER_VELOCITY=%g"
        % (motion["max_velocity"], motion["max_accel"],
           motion["minimum_cruise_ratio"], motion["square_corner_velocity"]))
    lines.append("SET_PRESSURE_ADVANCE ADVANCE=%g SMOOTH_TIME=%g"
                  % (extruder["pressure_advance"], extruder["smooth_time"]))

    fan_speed = fan.get("speed", 0.0)
    if fan_speed > 0:
        lines.append("M106 S%d" % round(fan_speed * 255))
    else:
        lines.append("M107")

    if bed_mesh.get("profile_name"):
        lines.append("BED_MESH_PROFILE LOAD=%s" % bed_mesh["profile_name"])
    else:
        lines.append("BED_MESH_CLEAR")

    retraction = state.get("firmware_retraction")
    if retraction:
        lines.append(
            "SET_RETRACTION RETRACT_LENGTH=%g RETRACT_SPEED=%g "
            "UNRETRACT_EXTRA_LENGTH=%g UNRETRACT_SPEED=%g"
            % (retraction["retract_length"], retraction["retract_speed"],
               retraction["unretract_extra_length"],
               retraction["unretract_speed"]))

    # Restoring heater targets is the one thermal action this mission
    # performs automatically, and only here - after an explicit,
    # human-issued NEBULAOS_PLR_RESUME command (never at boot, never
    # automatically). It is not a physical-position action.
    if thermal.get("extruder_target", 0.0) > 0:
        lines.append("M104 S%g" % thermal["extruder_target"])
    if thermal.get("bed_target", 0.0) > 0:
        lines.append("M140 S%g" % thermal["bed_target"])

    return lines


# ---------------------------------------------------------------------------
# Klipper glue.
# ---------------------------------------------------------------------------

class NebulaOSPowerLossRecovery:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.eeprom_path = config.get('eeprom_path', DEFAULT_EEPROM_PATH)
        self.sidecar_dir = config.get('sidecar_dir', DEFAULT_SIDECAR_DIR)
        self.checkpoint_interval = config.getfloat(
            'checkpoint_interval', DEFAULT_CHECKPOINT_INTERVAL, above=0.)
        self.poll_interval = config.getfloat(
            'poll_interval', DEFAULT_POLL_INTERVAL, above=0.)
        self.min_z_for_start = config.getfloat(
            'min_z_for_start', DEFAULT_MIN_Z_FOR_START, minval=0.)

        self.executor = None
        self.timer_handler = None

        self._session_file_info = None  # {"path","size","sha256"} once known
        self._last_checkpoint_time = 0.0
        self._checkpoint_in_flight = False
        self._resume_in_progress = False
        self._have_active_session = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("virtual_sdcard:reset_file",
                                             self._handle_reset_file)

        self.gcode.register_command(
            "NEBULAOS_PLR_STATUS", self.cmd_NEBULAOS_PLR_STATUS,
            desc="Report NebulaOS power-loss-recovery status")
        self.gcode.register_command(
            "NEBULAOS_PLR_RESUME", self.cmd_NEBULAOS_PLR_RESUME,
            desc="Resume a NebulaOS power-loss-recovery checkpoint")
        self.gcode.register_command(
            "NEBULAOS_PLR_DISCARD", self.cmd_NEBULAOS_PLR_DISCARD,
            desc="Discard any NebulaOS power-loss-recovery checkpoint")

    # -- setup -------------------------------------------------------------

    def _handle_ready(self):
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.print_stats = self.printer.lookup_object('print_stats')
        self.virtual_sdcard = self.printer.lookup_object('virtual_sdcard')
        self.exclude_object = self.printer.lookup_object('exclude_object', None)
        self.bed_mesh = self.printer.lookup_object('bed_mesh', None)
        self.extruder = self.printer.lookup_object('extruder', None)
        self.fan = self.printer.lookup_object('fan', None)
        self.firmware_retraction = self.printer.lookup_object(
            'firmware_retraction', None)

        # Same pattern virtual_sdcard.py itself uses for background file
        # I/O - load_object() force-loads the [aio_executor] singleton if
        # no other extra has already, then hands out a private Executor.
        aio = self.printer.load_object(self.config, 'aio_executor')
        self.executor = aio.allocate_executor("nebulaos_plr")

        if not os.path.isdir(self.sidecar_dir):
            try:
                os.makedirs(self.sidecar_dir)
            except OSError as exc:
                raise self.printer.config_error(
                    "nebulaos_power_loss_recovery: could not create "
                    "sidecar_dir %r: %s" % (self.sidecar_dir, exc))

        self._ensure_eeprom_accessible()

        self.timer_handler = self.reactor.register_timer(
            self._timer_event, self.reactor.NOW)

    def _ensure_eeprom_accessible(self):
        try:
            with open(self.eeprom_path, 'r+b'):
                pass
        except PermissionError as exc:
            # Per this mission's explicit instruction: never invent a
            # permission workaround (chmod/sudo/etc) - report the real
            # service user and stop.
            uid = os.getuid()
            raise self.printer.config_error(
                "nebulaos_power_loss_recovery: insufficient permission to "
                "open %r (running as uid=%d). This module refuses to "
                "chmod/sudo its way around this - fix the underlying "
                "device/service permissions instead. Original error: %s"
                % (self.eeprom_path, uid, exc))
        except (IOError, OSError) as exc:
            raise self.printer.config_error(
                "nebulaos_power_loss_recovery: could not open EEPROM path "
                "%r: %s" % (self.eeprom_path, exc))

    def _open_eeprom(self):
        return open(self.eeprom_path, 'r+b')

    # -- periodic state machine --------------------------------------------

    def _timer_event(self, eventtime):
        try:
            self._tick(eventtime)
        except Exception:
            logging.exception("nebulaos_power_loss_recovery: timer tick failed")
        return eventtime + self.poll_interval

    def _tick(self, eventtime):
        ps_status = self.print_stats.get_status(eventtime)
        state = ps_status.get('state')

        if state == 'printing':
            self._tick_printing(eventtime, ps_status)
        elif state in ('complete', 'cancelled'):
            if self._have_active_session:
                self._tombstone_session("print_%s" % state)
        # paused / error / standby: preserve the latest checkpoint, take no
        # further action (no new writes, no tombstone).

    def _tick_printing(self, eventtime, ps_status):
        vsd_status = self.virtual_sdcard.get_status(eventtime)
        th_status = self.toolhead.get_status(eventtime)

        if not self._have_active_session:
            if not meaningful_progress(ps_status, vsd_status, th_status,
                                        self.min_z_for_start):
                return
            self._start_session(vsd_status, eventtime)

        if not self._have_active_session:
            return  # _start_session may decline (e.g. hash failure)

        if eventtime - self._last_checkpoint_time < self.checkpoint_interval:
            return
        if self._checkpoint_in_flight:
            logging.info(
                "nebulaos_power_loss_recovery: checkpoint already in "
                "flight, skipping this interval rather than queueing")
            return
        self._checkpoint(eventtime, ps_status, vsd_status, th_status)

    def _start_session(self, vsd_status, eventtime):
        file_path = vsd_status.get('file_path')
        if not file_path:
            return
        try:
            size = os.path.getsize(file_path)
            sha256 = hash_file(file_path)
        except OSError:
            logging.exception(
                "nebulaos_power_loss_recovery: could not hash %r, "
                "not starting a PLR session", file_path)
            return
        self._session_file_info = {
            "path": self._relative_sdcard_path(file_path),
            "size": size,
            "sha256": sha256,
        }
        self._have_active_session = True
        # Force the very first checkpoint to be immediately due, regardless
        # of how large `eventtime` (reactor uptime, not wall-clock) already
        # is at session-start time.
        self._last_checkpoint_time = eventtime - self.checkpoint_interval

    def _relative_sdcard_path(self, file_path):
        sdcard_root = getattr(self.virtual_sdcard, 'sdcard_dirname', None)
        if sdcard_root and file_path.startswith(sdcard_root):
            rel = os.path.relpath(file_path, sdcard_root)
            return rel
        return file_path

    def _checkpoint(self, eventtime, ps_status, vsd_status, th_status):
        gm_status = self.gcode_move.get_status(eventtime)
        # "extruder_status" doubles as the thermal snapshot: extruder's own
        # get_status() already merges the heater's 'target' key with
        # pressure_advance/smooth_time (kinematics/extruder.py), and
        # bed_target is folded in here from the separate heater_bed object
        # so build_sidecar_state() has one dict to read both from.
        extruder_status = dict(self.extruder.get_status(eventtime)) \
            if self.extruder is not None else {}
        heater_bed = self.printer.lookup_object('heater_bed', None)
        if heater_bed is not None:
            extruder_status['bed_target'] = heater_bed.get_status(
                eventtime).get('target', 0.0)
        bed_mesh_status = self.bed_mesh.get_status(eventtime) \
            if self.bed_mesh is not None else {}
        exclude_object_status = self.exclude_object.get_status(eventtime) \
            if self.exclude_object is not None else {}
        fan_status = self.fan.get_status(eventtime) if self.fan is not None else {}
        fr_status = self.firmware_retraction.get_status(eventtime) \
            if self.firmware_retraction is not None else None

        file_position = vsd_status.get('file_position', 0)

        self._checkpoint_in_flight = True
        try:
            self.executor.submit(
                self._perform_checkpoint_blocking,
                self._session_file_info, file_position, gm_status,
                th_status, extruder_status, bed_mesh_status,
                exclude_object_status, fan_status, fr_status)
            self._last_checkpoint_time = eventtime
        except Exception:
            logging.exception(
                "nebulaos_power_loss_recovery: checkpoint failed")
        finally:
            self._checkpoint_in_flight = False

    def _perform_checkpoint_blocking(self, file_info, file_position,
                                      gm_status, th_status, extruder_status,
                                      bed_mesh_status, exclude_object_status,
                                      fan_status, fr_status):
        """Runs on the aio_executor background thread. Implements the
        checkpoint transaction's full 7-step ordering: build the sidecar
        payload, durably write+rename it, fsync the directory, and ONLY
        THEN write+verify the EEPROM record."""
        with self._open_eeprom() as eeprom:
            current = journal.scan_journal(eeprom)
            _, next_generation = journal.next_commit_target(current)

            state = build_sidecar_state(
                next_generation, file_info, file_position, gm_status,
                th_status, extruder_status, bed_mesh_status,
                exclude_object_status, fan_status, fr_status)

            final_path = sidecar_path_for_generation(self.sidecar_dir,
                                                      next_generation)
            tmp_path = final_path + ".tmp"
            # Sidecar write (steps 2-5) happens BEFORE the EEPROM write
            # (steps 6-7) - the file must be fully durable on disk first,
            # so a crash between the two never leaves the EEPROM pointing
            # at a generation whose sidecar was never actually written.
            atomic_write_json(tmp_path, final_path, state)
            journal.commit_checkpoint(eeprom, file_position)

    def _tombstone_session(self, reason):
        try:
            with self._open_eeprom() as eeprom:
                journal.commit_tombstone(eeprom)
        except Exception:
            logging.exception(
                "nebulaos_power_loss_recovery: tombstone (%s) failed", reason)
        self._have_active_session = False
        self._session_file_info = None

    def _handle_reset_file(self):
        if self._resume_in_progress:
            # Our own NEBULAOS_PLR_RESUME issues M23 deliberately - this is
            # expected, not a foreign file change. Never invalidate our own
            # recovery transaction because of it.
            return
        if self._have_active_session:
            logging.info(
                "nebulaos_power_loss_recovery: virtual_sdcard file changed "
                "mid-session - invalidating PLR rather than silently "
                "accepting it")
            self._tombstone_session("virtual_sdcard_reset_file")

    # -- gcode commands ------------------------------------------------------

    def cmd_NEBULAOS_PLR_STATUS(self, gcmd):
        try:
            with self._open_eeprom() as eeprom:
                record = journal.read_recovery_state(eeprom)
        except (IOError, OSError) as exc:
            gcmd.respond_info("NebulaOS PLR: EEPROM read failed: %s" % exc)
            return
        if record is None:
            gcmd.respond_info("NebulaOS PLR: no recovery available")
            return
        sidecar_path = sidecar_path_for_generation(self.sidecar_dir,
                                                    record.generation)
        sidecar = read_sidecar(sidecar_path)
        if sidecar is None:
            gcmd.respond_info(
                "NebulaOS PLR: EEPROM generation %d present but sidecar %s "
                "missing/unreadable" % (record.generation, sidecar_path))
            return
        gcmd.respond_info(
            "NebulaOS PLR: recovery available (generation=%d, "
            "file_position=%d, file=%s) - status "
            "AVAILABLE_POSITION_UNSAFE, use NEBULAOS_PLR_RESUME "
            "ALLOW_UNSAFE=1 to override the position-safety gate"
            % (record.generation, record.file_position,
               sidecar.get("file", {}).get("path")))

    def cmd_NEBULAOS_PLR_RESUME(self, gcmd):
        allow_unsafe = bool(gcmd.get_int('ALLOW_UNSAFE', 0, minval=0, maxval=1))

        with self._open_eeprom() as eeprom:
            record = journal.read_recovery_state(eeprom)

        current_file_info = None
        if record is not None:
            sidecar_path = sidecar_path_for_generation(self.sidecar_dir,
                                                        record.generation)
            sidecar = read_sidecar(sidecar_path)
        else:
            sidecar = None

        try:
            current_path = None
            current_size = None
            current_sha256 = None
            if sidecar is not None:
                file_info = sidecar.get("file") or {}
                candidate_path = file_info.get("path")
                if candidate_path:
                    resolved = self._resolve_sdcard_path(candidate_path)
                    if resolved and os.path.isfile(resolved):
                        current_path = candidate_path
                        current_size = os.path.getsize(resolved)
                        current_sha256 = hash_file(resolved)
            validated = validate_recovery(
                record, sidecar, current_path, current_size, current_sha256,
                allow_unsafe)
        except PositionUnsafe as exc:
            raise gcmd.error(str(exc))
        except IntegrityFailure as exc:
            raise gcmd.error("NebulaOS PLR: recovery refused: %s" % exc)

        self._resume_in_progress = True
        try:
            for line in build_resume_gcode_lines(validated):
                self.gcode.run_script_from_command(line)
        finally:
            self._resume_in_progress = False

        gcmd.respond_info(
            "NebulaOS PLR: resume state restored (generation=%d). "
            "No motion was performed and M24 was NOT issued - verify the "
            "physical position is safe, then resume printing manually."
            % validated["generation"])

    def cmd_NEBULAOS_PLR_DISCARD(self, gcmd):
        self._tombstone_session("manual_discard")
        gcmd.respond_info("NebulaOS PLR: checkpoint discarded (tombstoned)")

    def _resolve_sdcard_path(self, relative_path):
        sdcard_root = getattr(self.virtual_sdcard, 'sdcard_dirname', None)
        if sdcard_root:
            return os.path.join(sdcard_root, relative_path)
        return relative_path

    def get_status(self, eventtime):
        return {
            "active_session": self._have_active_session,
            "resume_in_progress": self._resume_in_progress,
        }


def load_config(config):
    return NebulaOSPowerLossRecovery(config)
