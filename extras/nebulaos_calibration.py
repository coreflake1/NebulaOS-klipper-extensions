# NebulaOS calibration coordinator (Phase 2 calibration-framework mission).
#
# The single [nebulaos_calibration] object that owns the canonical
# NEBULAOS_* public calibration API. This mission's first slice: standalone
# NEBULAOS_Z_OFFSET_CALIBRATE (both METHOD=LOAD_CELL and METHOD=MANUAL) and
# thin delegating wrappers for PID/bed-mesh. NEBULAOS_AUTO_CALIBRATE,
# NEBULAOS_AXIS_TWIST_CALIBRATE, the guided Input Shaper/E-Steps workflows,
# and the persistent calibration journal are NOT part of this slice - see
# the Phase 2 mission report for exactly what remains and why.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from . import nebulaos_probe_pair

_MAX_ERROR_LEN = 200


def _sanitize_error(exc):
    """Same convention as z_compensate.py's own _sanitize_calibration_error
    - str() on a Klipper error never includes a traceback/repr, so this
    just collapses whitespace and bounds length for a status field."""
    text = str(exc).strip() or exc.__class__.__name__
    text = ' '.join(text.split())
    if len(text) > _MAX_ERROR_LEN:
        text = text[:_MAX_ERROR_LEN - 3].rstrip() + '...'
    return text


class NebulaOSCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.down_min_z = config.getfloat('down_min_z', default=10., minval=1., maxval=50.)
        self.pro_cnt = config.getint('pro_cnt', default=3, minval=1)
        self.horizontal_move_z = config.getfloat(
            'horizontal_move_z', default=5., minval=1., maxval=50.)
        self.travel_speed = config.getfloat('travel_speed', default=200., above=0.)
        self.probe_lift_speed = config.getfloat('probe_lift_speed', default=20., above=0.)
        # Same conservative default and rationale as z_compensate.py's own
        # max_offset_correction_mm - a sanity ceiling, not a physical bound
        # (that is [heater_bed]/[extruder] max_temp's job for temperatures,
        # and here simply "this is a fine-tune, not a re-leveling").
        self.max_offset_correction_mm = config.getfloat(
            'max_offset_correction_mm', default=2., minval=0.1, maxval=10.)
        self.default_bed_pid_target = config.getfloat(
            'default_bed_pid_target', default=65., above=0.)
        self.default_hotend_pid_target = config.getfloat(
            'default_hotend_pid_target', default=230., above=0.)
        self.bed_mesh_profile_name = config.get(
            'bed_mesh_profile_name', default='nebulaos_calibration')

        self.z_offset_state = 'idle'
        self.z_offset_id = 0
        self.z_offset_result = None
        self.z_offset_error = None

        self.gcode.register_command(
            'NEBULAOS_Z_OFFSET_CALIBRATE', self.cmd_z_offset_calibrate,
            desc=self.cmd_z_offset_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_PID_CALIBRATE_BED', self.cmd_pid_calibrate_bed,
            desc=self.cmd_pid_calibrate_bed_help)
        self.gcode.register_command(
            'NEBULAOS_PID_CALIBRATE_HOTEND', self.cmd_pid_calibrate_hotend,
            desc=self.cmd_pid_calibrate_hotend_help)
        self.gcode.register_command(
            'NEBULAOS_BED_MESH_CALIBRATE', self.cmd_bed_mesh_calibrate,
            desc=self.cmd_bed_mesh_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_STATUS', self.cmd_calibration_status,
            desc=self.cmd_calibration_status_help)

    # ------------------------------------------------------------------
    # Reference XY point
    # ------------------------------------------------------------------
    def _resolve_reference_xy(self, gcmd):
        """The bed point NEBULAOS_Z_OFFSET_CALIBRATE measures at, absent an
        explicit X=/Y= override. Deliberately REUSES [z_compensate]'s own
        already-correct, already-hardware-qualified home_x/home_y
        resolution (z_compensate.py's _resolve_z_home_xy(), which fixed a
        real 1.5mm Y-axis bug in an earlier mission by preferring
        [gcode_macro _HOMING_PARAMS]'s real homing target over a
        [bed_mesh]-center approximation) rather than re-deriving the same
        logic a second time here - one source of truth, not two. This is
        legacy-depends-on-canonical for now, the opposite of the eventual
        target shape; acceptable while [z_compensate] remains a required,
        always-included section on this project's single supported
        printer model. A future cleanup can extract this resolution into
        its own small shared helper if/when z_compensate.py is ever fully
        retired - tracked, not silently left as an accident."""
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is not None and y is not None:
            return x, y
        zc = self.printer.lookup_object('z_compensate', None)
        if zc is None or zc.home_x is None or zc.home_y is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no X=/Y= given and no "
                "[z_compensate] reference point is available - specify "
                "X=<pos> Y=<pos> explicitly")
        return zc.home_x, zc.home_y

    def get_status(self, eventtime):
        return {
            'z_offset_state': self.z_offset_state,
            'z_offset_id': self.z_offset_id,
            'z_offset_result': self.z_offset_result,
            'z_offset_error': self.z_offset_error,
        }

    # ------------------------------------------------------------------
    # NEBULAOS_Z_OFFSET_CALIBRATE
    # ------------------------------------------------------------------
    cmd_z_offset_calibrate_help = (
        "Calibrate the BLTouch Z-offset (METHOD=LOAD_CELL|MANUAL)")

    def cmd_z_offset_calibrate(self, gcmd):
        method = gcmd.get('METHOD', 'LOAD_CELL').upper()
        if method == 'MANUAL':
            self._z_offset_calibrate_manual(gcmd)
        elif method == 'LOAD_CELL':
            self._z_offset_calibrate_load_cell(gcmd)
        else:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: unknown METHOD='%s' - "
                "expected LOAD_CELL or MANUAL" % (method,))

    def _z_offset_calibrate_manual(self, gcmd):
        """No duplicate paper-test implementation - delegates straight to
        stock PROBE_CALIBRATE, which already does exactly the "normal
        probe measurement + upstream Klipper manual nozzle reference"
        sequence this mode calls for (automatic BLTouch probe, move nozzle
        over that point, ManualProbeHelper's TESTZ/ACCEPT/ABORT, then its
        own configfile.set()). This call only starts that interactive
        session; the user continues it with TESTZ/ACCEPT/ABORT exactly as
        they would after calling PROBE_CALIBRATE directly."""
        self.gcode.respond_info(
            "NEBULAOS_Z_OFFSET_CALIBRATE: starting a manual Z-offset "
            "calibration via stock PROBE_CALIBRATE - continue with TESTZ/"
            "ACCEPT/ABORT.")
        self.gcode.run_script_from_command('PROBE_CALIBRATE')

    def _z_offset_calibrate_load_cell(self, gcmd):
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no [nebulaos_z_offset_probe] "
                "is configured on this printer - use METHOD=MANUAL instead, "
                "or add [nebulaos_z_offset_probe] to printer.cfg")
        if not z_offset_probe.get_status(self.reactor.monotonic())['is_calibrated']:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: the load cell has not been "
                "calibrated yet - run LOAD_CELL_CALIBRATE first, or use "
                "METHOD=MANUAL")

        x, y = self._resolve_reference_xy(gcmd)
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, _z_offset = probe_obj.get_offsets()

        self.z_offset_id += 1
        self.z_offset_state = 'running'
        self.z_offset_result = None
        self.z_offset_error = None

        try:
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed)
            new_offset = measurement.probe_z_offset

            if not math.isfinite(new_offset):
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %r is "
                    "not a finite number" % (new_offset,))
            if abs(new_offset) > self.max_offset_correction_mm:
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %.5fmm "
                    "exceeds max_offset_correction_mm=%.5fmm - refusing "
                    "to apply an implausibly large correction"
                    % (new_offset, self.max_offset_correction_mm))

            # Effective in the CURRENT session immediately (needed before
            # Bed Mesh runs later in a future Auto-Calibrate sequence) -
            # pinned upstream Klipper exposes no public runtime command to
            # replace the registered probe's live z_offset directly (only
            # a full config reload/restart, or Z_OFFSET_APPLY_PROBE +
            # SAVE_CONFIG, which is exactly the restart this must avoid
            # mid-sequence). This is the minimum NebulaOS-side adapter for
            # that gap: confirmed directly against the pinned source
            # (klippy/extras/bltouch.py's PrinterBLTouch.__init__ does
            # `self.probe_offsets = probe.ProbeOffsetsHelper(config)`, and
            # every get_offsets() call reads that same live instance) that
            # mutating probe_obj.probe_offsets.z_offset is the correct,
            # real seam - not an upstream Klipper patch, nothing on disk
            # is touched, and it is undone automatically by the next
            # config (re)load like any other unsaved runtime state.
            probe_obj.probe_offsets.z_offset = new_offset

            configfile = self.printer.lookup_object('configfile')
            configfile.set(probe_obj.cmd_helper.name, 'z_offset',
                            "%.3f" % (new_offset,))
        except Exception as e:
            self.z_offset_state = 'error'
            self.z_offset_result = None
            self.z_offset_error = _sanitize_error(e)
            raise

        self.z_offset_state = 'complete'
        self.z_offset_result = new_offset
        self.z_offset_error = None
        self.gcode.respond_info(
            "NEBULAOS_Z_OFFSET_CALIBRATE: measured %.5f mm (probe trigger "
            "%.5f, nozzle contact %.5f at X=%.3f Y=%.3f), applied live. "
            "The SAVE_CONFIG command will make this permanent."
            % (new_offset, measurement.raw_probe_trigger_z,
               measurement.raw_nozzle_contact_z, x, y))

    # ------------------------------------------------------------------
    # Thin delegating wrappers (real logic stays entirely upstream)
    # ------------------------------------------------------------------
    cmd_pid_calibrate_bed_help = "Calibrate bed heater PID (stock PID_CALIBRATE)"

    def cmd_pid_calibrate_bed(self, gcmd):
        target = gcmd.get_float('TARGET', self.default_bed_pid_target)
        self.gcode.run_script_from_command(
            'PID_CALIBRATE HEATER=heater_bed TARGET=%.2f' % (target,))

    cmd_pid_calibrate_hotend_help = "Calibrate hotend PID (stock PID_CALIBRATE)"

    def cmd_pid_calibrate_hotend(self, gcmd):
        target = gcmd.get_float('TARGET', self.default_hotend_pid_target)
        self.gcode.run_script_from_command(
            'PID_CALIBRATE HEATER=extruder TARGET=%.2f' % (target,))

    cmd_bed_mesh_calibrate_help = (
        "Full reference bed mesh, saved under a named profile (stock BED_MESH_CALIBRATE)")

    def cmd_bed_mesh_calibrate(self, gcmd):
        profile = gcmd.get('PROFILE', self.bed_mesh_profile_name)
        self.gcode.run_script_from_command('BED_MESH_CALIBRATE')
        self.gcode.run_script_from_command(
            'BED_MESH_PROFILE SAVE="%s"' % (profile,))

    # ------------------------------------------------------------------
    # NEBULAOS_CALIBRATION_STATUS
    # ------------------------------------------------------------------
    cmd_calibration_status_help = "Report the current NebulaOS calibration status"

    def cmd_calibration_status(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: z_offset_state=%s z_offset_id=%s "
            "z_offset_result=%s z_offset_error=%s"
            % (status['z_offset_state'], status['z_offset_id'],
               status['z_offset_result'], status['z_offset_error']))


def load_config(config):
    return NebulaOSCalibration(config)
