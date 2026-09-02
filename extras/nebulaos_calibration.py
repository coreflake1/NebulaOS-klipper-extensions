# NebulaOS calibration coordinator (Phase 2 calibration-framework mission;
# contact-safety stabilization rewrite, corrected).
#
# The single [nebulaos_calibration] object that owns the canonical
# NEBULAOS_* public calibration API. Implemented: standalone
# NEBULAOS_Z_OFFSET_CALIBRATE (LOAD_CELL only, bounded-descent envelope +
# measurement-quality/repeatability gating, plus its SIMULATE_BOOTSTRAP=1
# qualification aid - mission §8); NEBULAOS_AXIS_TWIST_CALIBRATE (AXIS=X|
# Y|BOTH - see "Axis Twist" section below: HARD BLOCKED pending remote
# load-cell contact hardware qualification); NEBULAOS_AUTO_CALIBRATE, the
# fully-automatic preflight->home->PID->nozzle-clean->Z-offset->bed-mesh->
# one-SAVE_CONFIG-restart-and-verify sequence (mission §11), backed by
# nebulaos_calibration_journal.py's persistent transaction journal (mission
# §12) and NEBULAOS_CALIBRATION_CANCEL. The guided Input Shaper/E-Steps
# workflows (separate commands, not part of NEBULAOS_AUTO_CALIBRATE - see
# the Phase 2 mission document's own §13/§14) are NOT part of this slice.
#
# ---------------------------------------------------------------------
# Upstream-first cleanup (Overnight Contact-Safety Stabilization mission)
# ---------------------------------------------------------------------
# Manual Axis Twist and manual Z-offset are NO LONGER wrapped here at all.
# There is exactly one way to run each: call pristine upstream's own
# AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=X|Y or PROBE_CALIBRATE directly
# (both their own interactive TESTZ/ACCEPT/ABORT workflows) - this module
# used to offer thin METHOD=MANUAL passthroughs for both, which added a
# second name for the exact same upstream command with no product value,
# and both have been removed. klippy/extras/axis_twist_compensation.py and
# probe.py (58bd67db..., NOT modified, NOT shadowed, NOT monkeypatched -
# see docs/NEBULAOS_PRISTINE_KLIPPER.md) remain fully authoritative,
# exactly as before.
#
# The PID-default-target and bed-mesh-named-profile Python wrappers
# (NEBULAOS_PID_CALIBRATE_BED/_HOTEND, NEBULAOS_BED_MESH_CALIBRATE) are
# also removed. The bed-mesh one was a real duplication, not just a
# convenience: pinned upstream bed_mesh.py's own cmd_BED_MESH_CALIBRATE
# already reads `gcmd.get('PROFILE', "default")` and saves under it
# directly (confirmed against the pinned source, not assumed) - so
# `BED_MESH_CALIBRATE PROFILE=<name>` alone is the exact upstream
# equivalent of what this module's own wrapper did in two commands. PID's
# default-target convenience has no upstream equivalent (TARGET is
# required, no default), but that is a firmware-config concern, not a
# reason to keep Python in this repository - see NebulaOS-firmware's own
# gcode_macro convenience wrappers instead.
#
# ---------------------------------------------------------------------
# Axis Twist: automatic (LOAD_CELL) path is HARD BLOCKED
# ---------------------------------------------------------------------
# NEBULAOS_AXIS_TWIST_CALIBRATE remains NebulaOS-owned - upstream provides
# no automatic load-cell nozzle-reference frontend, and one is genuinely
# useful once the underlying contact primitive is hardware-qualified for
# REMOTE bed points (away from [nebulaos_z_offset_probe]'s own qualified
# reference point - see the real safety incident this whole mission is
# built on, _evidence/overnight-hx711-investigation-20260831-233518/
# REPORT.md). That qualification has NOT happened. Until it does, this
# command performs ZERO movement, ZERO nozzle contact, and ZERO
# compensation/config changes for any AXIS value - see
# cmd_axis_twist_calibrate() below, which returns a structured
# REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED error before looking up any
# hardware object at all. There is deliberately no unsafe public override.
#
# The corrected pure geometry-preflight math this future qualification
# will need (axis_twist_geometry_preflight() below) is implemented and
# fully tested now, but is NOT wired to any live motion path - it is dead
# code from cmd_axis_twist_calibrate's point of view until a future
# mission re-enables the LOAD_CELL path. Its own header comment explains
# the transform and the real analysis mistake (sign error) an earlier
# session made and then corrected by reading nebulaos_probe_pair.py's real
# code, not re-deriving it - the SAME subtraction-based transform is used
# in both places for exactly that reason (a second, potentially-divergent
# copy of this math is the failure mode being avoided).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import time

from . import nebulaos_probe_pair
from . import nebulaos_calibration_journal as calibration_journal

_MAX_ERROR_LEN = 200

# Matches upstream's own axis_twist_compensation.py DEFAULT_SAMPLE_COUNT.
_DEFAULT_AXIS_TWIST_SAMPLE_COUNT = 3


def axis_twist_bed_points(compensation, axis, sample_count):
    """Reproduces ONLY the linear-interpolation bed-point arithmetic from
    pinned upstream klippy/extras/axis_twist_compensation.py's
    Calibrater.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE (58bd67db..., the
    'if axis == "X" / elif axis == "Y"' block) - upstream does not expose
    this as a separately-callable helper. `compensation` is the real,
    already-configured AxisTwistCompensation object (its calibrate_start_x/
    end_x/y and calibrate_start_y/end_y/x are the same six config values
    Calibrater's own __init__ already reads for x_start_point/x_end_point/
    y_start_point/y_end_point - this function deliberately reads them from
    the SAME object, not a second copy, so the two can never disagree).
    Returns a list of (x, y) bed-frame nozzle-target points, matching
    upstream's own bed_points list exactly (in order)."""
    if axis == 'X':
        start = (compensation.calibrate_start_x, compensation.calibrate_y)
        end = (compensation.calibrate_end_x, compensation.calibrate_y)
        if start[0] is None or end[0] is None or start[1] is None:
            raise ValueError(
                "axis_twist_compensation for X axis requires "
                "calibrate_start_x, calibrate_end_x and calibrate_y to be defined")
        axis_range = end[0] - start[0]
        interval = axis_range / (sample_count - 1)
        return [(start[0] + i * interval, start[1]) for i in range(sample_count)]
    elif axis == 'Y':
        start = (compensation.calibrate_x, compensation.calibrate_start_y)
        end = (compensation.calibrate_x, compensation.calibrate_end_y)
        if start[1] is None or end[1] is None or start[0] is None:
            raise ValueError(
                "axis_twist_compensation for Y axis requires "
                "calibrate_start_y, calibrate_end_y and calibrate_x to be defined")
        axis_range = end[1] - start[1]
        interval = axis_range / (sample_count - 1)
        return [(start[0], start[1] + i * interval) for i in range(sample_count)]
    else:
        raise ValueError("axis_twist_bed_points: axis must be 'X' or 'Y', got %r" % (axis,))


def axis_twist_geometry_preflight(axis, bed_points, probe_x_offset,
                                   probe_y_offset, axis_minimum,
                                   axis_maximum, safety_margin_mm=0.0):
    """Pure geometry validation for an Axis Twist LOAD_CELL calibration run
    - NO printer/toolhead access, NO movement of any kind, safe to call at
    any time (including before homing). NOT currently reachable from
    cmd_axis_twist_calibrate() - see this module's own header comment on
    why the LOAD_CELL path is hard blocked - but implemented and tested
    now so a future qualification mission has a correct, ready-made check
    to wire in rather than deriving this again.

    For each nozzle-frame bed point `P` (as returned by
    axis_twist_bed_points), computes BOTH carriage targets a real
    calibration run would need to reach:
        nozzle_carriage_target = P                          (no offset)
        probe_carriage_target  = P - (probe_x_offset, probe_y_offset)
    - the SAME subtraction-based transform nebulaos_probe_pair.py's real,
    hardware-qualified measure_probe_nozzle_pair() uses (confirmed
    directly against its source, not re-derived a second, potentially-
    divergent way - see this module's header comment for why an earlier,
    ADDITION-based version of this exact check was wrong and led to a real
    live-session safety incident).

    Validates the axis this run cares about (X for axis=='X', Y for
    axis=='Y') on BOTH targets against [axis_minimum, axis_maximum],
    shrunk on both ends by safety_margin_mm. Returns the ordered list of
    (nozzle_target, probe_target) pairs on success. Raises ValueError
    naming the FIRST invalid point (and whether it was the nozzle or probe
    target) on failure - never silently clamps, skips, or adjusts an
    endpoint.
    """
    if axis not in ('X', 'Y'):
        raise ValueError(
            "axis_twist_geometry_preflight: axis must be 'X' or 'Y', got %r"
            % (axis,))
    axis_index = 0 if axis == 'X' else 1
    lo = axis_minimum + safety_margin_mm
    hi = axis_maximum - safety_margin_mm
    offset = probe_x_offset if axis == 'X' else probe_y_offset

    results = []
    for i, point in enumerate(bed_points):
        nozzle_target = point
        probe_target = (point[0] - probe_x_offset, point[1] - probe_y_offset)

        nozzle_value = nozzle_target[axis_index]
        probe_value = probe_target[axis_index]

        if not (lo <= nozzle_value <= hi):
            raise ValueError(
                "axis_twist_geometry_preflight: AXIS=%s point %d/%d - "
                "nozzle carriage target %.3f is outside [%.3f, %.3f] "
                "(safety_margin_mm=%.3f)"
                % (axis, i + 1, len(bed_points), nozzle_value, lo, hi,
                   safety_margin_mm))
        if not (lo <= probe_value <= hi):
            raise ValueError(
                "axis_twist_geometry_preflight: AXIS=%s point %d/%d - "
                "probe carriage target %.3f (offset=%.3f) is outside "
                "[%.3f, %.3f] (safety_margin_mm=%.3f)"
                % (axis, i + 1, len(bed_points), probe_value, offset, lo,
                   hi, safety_margin_mm))

        results.append((nozzle_target, probe_target))
    return results


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
        # reference_x/reference_y (Phase 2 contact-safety mission, §11):
        # the canonical physical bed point NEBULAOS_Z_OFFSET_CALIBRATE
        # LOAD_CELL measures at, absent an explicit X=/Y= override -
        # firmware config's job, not a hardcoded Python constant. Factory
        # candidate is the stock KE's own validated touch-reference region
        # (noz_pos_center: 20,25 in z_compensate.py/[z_compensate], the
        # same real, stock-precedented point the overnight investigation
        # confirmed is stock's sole validated single-strain-gauge contact
        # location - see _project/missions/
        # nebulaos-klipper-loadcell-architecture-history.md). Left unset
        # (None) here on purpose; _resolve_reference_xy() falls back to
        # [z_compensate]'s own home_x/home_y when neither this nor an
        # explicit X=/Y= is available, so an older printer.cfg without
        # these two options still works exactly as before.
        self.reference_x = config.getfloat('reference_x', default=None)
        self.reference_y = config.getfloat('reference_y', default=None)
        # Nozzle-contact safety envelope constants (corrected) - see
        # nebulaos_probe_pair.py's own header for the exact two formulas.
        # Deliberately no invented production defaults for either; exactly
        # one is required per run, depending on whether the CURRENT live
        # probe z_offset is a credible physical prior (ESTABLISHED) or not
        # (BOOTSTRAP/virgin, e.g. the real factory default of 0.000).
        #
        # established_contact_margin_mm: how much FURTHER than the
        # predicted nozzle-contact plane (raw_probe_trigger_z -
        # CURRENT probe z_offset) the nozzle may still be commanded to
        # travel. Deliberately does NOT also carry the probe's own Z
        # offset the way an earlier, incorrect version of this envelope
        # (anchored to the raw probe trigger Z directly) accidentally did
        # - on this printer's real captured hardware state that offset is
        # ~1.795mm, far larger than any sane margin, which is exactly the
        # bug this rename and re-derivation fixes.
        #
        # Phase 2 mission §7 qualification (2026-09-02): 1.0mm, qualified
        # from CR-Touch repeatability at a nearby point (~0.01mm, two
        # orders of magnitude tighter than this margin - see
        # _evidence/phase2-contact-safety-hwqual-20260901-182810/
        # 07-contact-parameter-proposal/07-proposal.txt for the original
        # derivation) and confirmed with zero established-envelope
        # incidents across 22 real ESTABLISHED touch_probe runs (66
        # individual contacts) spanning cold (20C ambient) and hot (bed
        # 65C/hotend 230C) sessions on 2026-09-01 and 2026-09-02 - see
        # _evidence/phase2-live-full-stack-closure-20260902-180602/
        # REPORT.md. No longer requires a qualification-only printer.cfg
        # override.
        self.established_contact_margin_mm = config.getfloat(
            'established_contact_margin_mm', default=1.0,
            minval=0.01, maxval=5.)
        # bootstrap_contact_envelope_mm: how far below the nozzle's own
        # actual, known, currently-measured starting Z (horizontal_move_z,
        # default 5.0mm) a BOOTSTRAP/virgin calibration run (no credible
        # prior) may blindly search. A SEPARATE value from down_min_z -
        # never silently reused as one.
        #
        # Phase 2 mission §8 qualification (2026-09-02): 8.0mm, derived
        # from this unit's own measured z_offset history (1.6-2.3mm across
        # cold/hot sessions this mission) plus real margin - a genuinely
        # virgin unit (probe_z_offset=0.000 at homing) needs to search
        # from horizontal_move_z=5.0mm down to roughly -(1.6 to 2.3)mm in
        # that frame, i.e. 6.6-7.3mm of real depth; 8.0mm leaves ~0.7-1.4mm
        # of margin beyond this unit's own worst observed case for
        # ordinary unit-to-unit assembly variance, while the resulting
        # commanded_floor_z (5.0-8.0=-3.0mm) still leaves 2mm of
        # independent margin before the absolute stepper floor
        # (position_min=-5.0mm on this printer).
        #
        # Physically qualified via NEBULAOS_Z_OFFSET_CALIBRATE
        # SIMULATE_BOOTSTRAP=1 (see that command's own docstring): 6 real
        # BOOTSTRAP-mode nozzle contacts on hardware that already has a
        # trusted ESTABLISHED calibration, WITHOUT ever reading, mutating,
        # or risking that real calibration - 5 at ENVELOPE=7.0mm, all
        # succeeding with results (1.014-1.161mm) closely agreeing with an
        # ESTABLISHED-mode measurement taken moments earlier in the same
        # session (1.014-2.06mm range that same session) - cross-
        # validating that the BOOTSTRAP formula/motion path measures the
        # same real physical quantity correctly. A 6th run at a
        # deliberately excessive ENVELOPE=12.0mm (commanded_floor_z=-7.0mm,
        # beyond position_min) completed identically and safely: real
        # contact triggered at ~1.16mm long before any floor was
        # approached, and touch_probe()'s own contact_floor=max(z_floor,
        # minimum_allowed_z) clamp means the ACTUAL floor a probing move
        # could ever reach is structurally bounded by the independent
        # absolute z_floor regardless of how generous
        # bootstrap_contact_envelope_mm is - confirming this constant
        # cannot itself cause a deeper-than-intended search even if
        # mis-qualified. Full evidence:
        # _evidence/phase2-live-full-stack-closure-20260902-180602/
        # 03-bootstrap-simulation-qualification/. Not yet tested with an
        # actual, deeper real virgin-unit contact point (this unit's own
        # current ESTABLISHED calibration only lets the simulation reach
        # its OWN, comparatively shallow, contact point) - the mechanism,
        # formula, and safety clamping are proven; the specific 8.0mm
        # number is a geometry-and-margin-based estimate for a genuinely
        # virgin unit, not itself hardware-proven at full depth.
        self.bootstrap_contact_envelope_mm = config.getfloat(
            'bootstrap_contact_envelope_mm', default=8.0,
            minval=0.5, maxval=30.)
        # Measurement-quality and repeatability acceptance bounds (§7/§9/
        # §10).
        #
        # Phase 2 mission §7 qualification (2026-09-02): all three
        # defaults below are qualified from the same 22-run/66-touch
        # cold+hot dataset referenced above. Observed maxima against each
        # bound (all comfortably passing, no violations in any run):
        #   |fit_delta|:            0.2586mm observed vs 0.3mm bound
        #                           (~14% headroom - the tightest of the
        #                           three; matches the original proposal's
        #                           intent of rejecting the real recorded
        #                           ~1.106mm historical incident with wide
        #                           margin while still accepting healthy
        #                           trigger-latency corrections)
        #   repeatability range:    0.0407mm observed vs 0.15mm bound
        #                           (~3.7x headroom)
        #   repeatability stddev:   0.01973mm observed vs 0.06mm bound
        #                           (~3x headroom)
        # min_accepted_samples=2 (of pro_cnt=3): every one of the 22 runs
        # in this dataset accepted all 3 samples; no evidence to justify
        # a different value. No longer requires a qualification-only
        # printer.cfg override for any of the four.
        self.max_abs_fit_delta = config.getfloat(
            'max_abs_fit_delta', default=0.3, minval=0.001, maxval=10.)
        self.min_accepted_samples = config.getint(
            'min_accepted_samples', default=2, minval=1)
        self.max_repeatability_range = config.getfloat(
            'max_repeatability_range', default=0.15, minval=0.001, maxval=10.)
        self.max_repeatability_stddev = config.getfloat(
            'max_repeatability_stddev', default=0.06, minval=0.001, maxval=10.)
        self.axis_twist_sample_count = config.getint(
            'axis_twist_sample_count', default=_DEFAULT_AXIS_TWIST_SAMPLE_COUNT,
            minval=2)

        # NEBULAOS_AUTO_CALIBRATE (mission §11). pid_bed_target/
        # pid_hotend_target are the PID-tune targets only (PID_CALIBRATE's
        # own TARGET=, run early in the sequence, well before nozzle
        # clean) - defaults are the qualification thermal state this
        # mission's hot-reliability evidence was gathered at (bed 65C /
        # hotend 230C).
        #
        # z_offset_reference_temp (root-caused 2026-09-02/03, live A/B
        # test - see _evidence/phase2-live-full-stack-closure-
        # 20260902-180602/07-nozzle-contamination-test/): the LOCALIZED_
        # Z_OFFSET stage deliberately does NOT reheat to pid_hotend_target
        # (230C) - it measures at this lower, separate temperature
        # instead. Root cause: residual nozzle ooze/contamination
        # accumulating during a long hot dwell (PID-hotend tune + a hot
        # nozzle-clean cycle) was found to bias the load-cell contact
        # measurement low by ~0.6mm (a reproducible ~1.01-1.15mm cluster,
        # against a normal ~1.73-1.80mm cluster) - proven by measuring
        # immediately after a fresh nozzle clean, at 140C instead of
        # 230C: 3/3 clean runs landed back in the normal cluster
        # (1.733-1.742mm, range 0.0095mm) on the SAME unit, SAME
        # persisted prior, SAME contact algorithm - nothing else changed.
        # 140C default matches nozzle_clean's own hot_end_temp resting
        # temperature ([z_compensate] config), so this is normally a
        # no-op confirmation of a state nozzle_clean already reached, not
        # a second real heat cycle. The contact algorithm and safety
        # envelope (established_contact_margin_mm etc.) are UNCHANGED -
        # this fix is sequencing/thermal-reference only.
        #
        # thermal_soak_seconds is an EXPLICIT additional dwell after
        # M190/M109 already report the target reached (their own
        # convergence tolerance is looser than "fully thermally
        # settled") - a plain upstream G4 dwell, not a reimplemented wait
        # loop. bed_mesh_profile matches BED_MESH_CALIBRATE's own
        # upstream default (gcmd.get('PROFILE', "default")) so a plain
        # `BED_MESH_CALIBRATE` invoked by a user later, with no PROFILE=,
        # loads exactly what this orchestrator saved.
        self.pid_bed_target = config.getfloat(
            'pid_bed_target', default=65., minval=0., maxval=120.)
        self.pid_hotend_target = config.getfloat(
            'pid_hotend_target', default=230., minval=0., maxval=320.)
        self.z_offset_reference_temp = config.getfloat(
            'z_offset_reference_temp', default=140., minval=0., maxval=320.)
        self.thermal_soak_seconds = config.getfloat(
            'thermal_soak_seconds', default=15., minval=0., maxval=300.)
        self.bed_mesh_profile = config.get('bed_mesh_profile', default='default')
        # Overridable only for unit tests (a real printer always uses the
        # real default path under /usr/data) - see
        # nebulaos_calibration_journal.py's own header for why this journal
        # exists at all (crash-safety around the one SAVE_CONFIG+restart
        # boundary, not routine status - NEBULAOS_CALIBRATION_STATUS's
        # auto_calibrate_* fields are the normal way to poll progress).
        self.journal_path = config.get(
            'journal_path', default=calibration_journal.DEFAULT_JOURNAL_PATH)

        self.z_offset_state = 'idle'
        self.z_offset_id = 0
        self.z_offset_result = None
        self.z_offset_error = None
        # Structured diagnostics (§8/§22) for the most recent LOAD_CELL
        # attempt - the small, Moonraker-status-safe summary fields, NOT
        # the full per-sample ContactSample list (see get_status()'s own
        # comment on why that stays out of normal status).
        self.z_offset_physical_x = None
        self.z_offset_physical_y = None
        self.z_offset_contact_mode = None
        self.z_offset_predicted_nozzle_contact_z = None
        self.z_offset_commanded_floor_z = None
        self.z_offset_raw_probe_trigger_z = None
        self.z_offset_sample_count = None
        self.z_offset_accepted_count = None
        self.z_offset_range = None
        self.z_offset_stddev = None

        # SIMULATE_BOOTSTRAP (mission §8) - a completely separate status
        # namespace from the real z_offset_* fields above, on purpose: a
        # simulation result must never be mistakable for (or accidentally
        # overwrite the display of) a real applied/staged calibration.
        self.bootstrap_sim_id = 0
        self.bootstrap_sim_state = 'idle'
        self.bootstrap_sim_result = None
        self.bootstrap_sim_error = None
        self.bootstrap_sim_envelope_mm = None
        self.bootstrap_sim_raw_probe_trigger_z = None
        self.bootstrap_sim_commanded_floor_z = None

        # Axis Twist state - kept PER AXIS (not one shared state) so
        # AXIS=BOTH's status can distinguish X progress from Y progress,
        # and so recalibrating one axis never disturbs the other's last
        # known result in this module's own status view (the underlying
        # upstream object's own compensation arrays already have this
        # property independently - see clear_compensations(axis) - this
        # just mirrors it in the status model).
        self.axis_twist_id = 0
        self.axis_twist_method = None
        self.axis_twist_current_axis = None  # the sub-axis actively running now, or None
        self.axis_twist_sample_index = 0
        self.axis_twist_sample_total = 0
        self.axis_twist_x_state = 'idle'
        self.axis_twist_x_result = None
        self.axis_twist_x_error = None
        self.axis_twist_y_state = 'idle'
        self.axis_twist_y_result = None
        self.axis_twist_y_error = None

        # NEBULAOS_AUTO_CALIBRATE / journal state (mission §11/§12).
        # auto_calibrate_stage mirrors calibration_journal.STAGES; the
        # journal itself is the crash-safe copy of this same information
        # on disk, written at every stage transition (see
        # _auto_calibrate_advance() below) - these in-memory fields are
        # what NEBULAOS_CALIBRATION_STATUS actually reports during a
        # normal (non-crashed) run, cheaper than re-reading the journal
        # file on every status poll.
        self.auto_calibrate_id = 0
        self.auto_calibrate_state = 'idle'
        self.auto_calibrate_stage = None
        self.auto_calibrate_error = None
        self.auto_calibrate_result = None
        self._auto_calibrate_cancel_requested = False

        self.gcode.register_command(
            'NEBULAOS_Z_OFFSET_CALIBRATE', self.cmd_z_offset_calibrate,
            desc=self.cmd_z_offset_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_STATUS', self.cmd_calibration_status,
            desc=self.cmd_calibration_status_help)
        self.gcode.register_command(
            'NEBULAOS_AXIS_TWIST_CALIBRATE', self.cmd_axis_twist_calibrate,
            desc=self.cmd_axis_twist_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_AUTO_CALIBRATE', self.cmd_auto_calibrate,
            desc=self.cmd_auto_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_CANCEL', self.cmd_calibration_cancel,
            desc=self.cmd_calibration_cancel_help)
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)

    # ------------------------------------------------------------------
    # Reference XY point
    # ------------------------------------------------------------------
    def _resolve_reference_xy(self, gcmd):
        """The bed point NEBULAOS_Z_OFFSET_CALIBRATE measures at, absent an
        explicit X=/Y= override.

        Phase 2 contact-safety mission (§11): the canonical reference is
        now this section's OWN configured reference_x/reference_y (the
        local sensor area, e.g. the stock-precedented 20,25 region) -
        firmware config's job, not a hardcoded Python constant and not
        borrowed from a different section's concern.

        Falls back to [z_compensate]'s own already-correct, already-
        hardware-qualified home_x/home_y resolution (z_compensate.py's
        _resolve_z_home_xy(), which fixed a real 1.5mm Y-axis bug in an
        earlier mission by preferring [gcode_macro _HOMING_PARAMS]'s real
        homing target over a [bed_mesh]-center approximation) ONLY when
        reference_x/reference_y are both unset - keeps an older
        printer.cfg without those two new options working exactly as
        before, without silently overriding an operator who has not yet
        adopted the new config keys."""
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is not None and y is not None:
            return x, y
        if self.reference_x is not None and self.reference_y is not None:
            return self.reference_x, self.reference_y
        zc = self.printer.lookup_object('z_compensate', None)
        if zc is None or zc.home_x is None or zc.home_y is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no X=/Y= given, no "
                "reference_x/reference_y configured on [nebulaos_calibration], "
                "and no [z_compensate] reference point is available - "
                "specify X=<pos> Y=<pos> explicitly")
        return zc.home_x, zc.home_y

    def get_status(self, eventtime):
        return {
            'z_offset_state': self.z_offset_state,
            'z_offset_id': self.z_offset_id,
            'z_offset_result': self.z_offset_result,
            'z_offset_error': self.z_offset_error,
            'z_offset_physical_x': self.z_offset_physical_x,
            'z_offset_physical_y': self.z_offset_physical_y,
            'z_offset_contact_mode': self.z_offset_contact_mode,
            'z_offset_predicted_nozzle_contact_z': self.z_offset_predicted_nozzle_contact_z,
            'z_offset_commanded_floor_z': self.z_offset_commanded_floor_z,
            'z_offset_raw_probe_trigger_z': self.z_offset_raw_probe_trigger_z,
            'z_offset_sample_count': self.z_offset_sample_count,
            'z_offset_accepted_count': self.z_offset_accepted_count,
            'z_offset_range': self.z_offset_range,
            'z_offset_stddev': self.z_offset_stddev,
            'bootstrap_sim_id': self.bootstrap_sim_id,
            'bootstrap_sim_state': self.bootstrap_sim_state,
            'bootstrap_sim_result': self.bootstrap_sim_result,
            'bootstrap_sim_error': self.bootstrap_sim_error,
            'bootstrap_sim_envelope_mm': self.bootstrap_sim_envelope_mm,
            'bootstrap_sim_raw_probe_trigger_z': self.bootstrap_sim_raw_probe_trigger_z,
            'bootstrap_sim_commanded_floor_z': self.bootstrap_sim_commanded_floor_z,
            'axis_twist_id': self.axis_twist_id,
            'axis_twist_method': self.axis_twist_method,
            'axis_twist_current_axis': self.axis_twist_current_axis,
            'axis_twist_sample_index': self.axis_twist_sample_index,
            'axis_twist_sample_total': self.axis_twist_sample_total,
            'axis_twist_x_state': self.axis_twist_x_state,
            'axis_twist_x_result': self.axis_twist_x_result,
            'axis_twist_x_error': self.axis_twist_x_error,
            'axis_twist_y_state': self.axis_twist_y_state,
            'axis_twist_y_result': self.axis_twist_y_result,
            'axis_twist_y_error': self.axis_twist_y_error,
            'auto_calibrate_id': self.auto_calibrate_id,
            'auto_calibrate_state': self.auto_calibrate_state,
            'auto_calibrate_stage': self.auto_calibrate_stage,
            'auto_calibrate_error': self.auto_calibrate_error,
            'auto_calibrate_result': self.auto_calibrate_result,
        }

    # ------------------------------------------------------------------
    # NEBULAOS_Z_OFFSET_CALIBRATE
    # ------------------------------------------------------------------
    cmd_z_offset_calibrate_help = (
        "Calibrate the BLTouch Z-offset via the local secondary load cell "
        "- for a manual calibration, use pristine upstream PROBE_CALIBRATE "
        "directly. SIMULATE_BOOTSTRAP=1 ENVELOPE=<mm> qualifies the "
        "BOOTSTRAP contact path on an already-calibrated unit without "
        "applying or staging anything - see NEBULAOS_CALIBRATION_STATUS's "
        "bootstrap_sim_* fields")

    def cmd_z_offset_calibrate(self, gcmd):
        """LOAD_CELL only - there is no METHOD=MANUAL passthrough any more
        (see this module's own header comment: call pristine upstream
        PROBE_CALIBRATE directly for a manual run, it needs no wrapper)."""
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no [nebulaos_z_offset_probe] "
                "is configured on this printer - use stock PROBE_CALIBRATE "
                "instead, or add [nebulaos_z_offset_probe] to printer.cfg")
        if not z_offset_probe.get_status(self.reactor.monotonic())['is_calibrated']:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: the load cell has not been "
                "calibrated yet - run LOAD_CELL_CALIBRATE first, or use "
                "stock PROBE_CALIBRATE instead")

        x, y = self._resolve_reference_xy(gcmd)
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, probe_z_offset = probe_obj.get_offsets()

        if gcmd.get_int('SIMULATE_BOOTSTRAP', 0):
            self._cmd_z_offset_calibrate_simulate_bootstrap(
                gcmd, x, y, probe_x_offset, probe_y_offset, z_offset_probe)
            return

        self.z_offset_id += 1
        self.z_offset_state = 'running'
        self.z_offset_result = None
        self.z_offset_error = None
        self.z_offset_physical_x = x
        self.z_offset_physical_y = y

        try:
            # Bounded-descent envelope (ESTABLISHED vs BOOTSTRAP, derived
            # from the CURRENT live probe_z_offset), measurement-quality
            # gates, and repeatability aggregation all live inside this
            # call now (§5-§10, corrected) - measure_probe_nozzle_pair()
            # fails closed with CONTACT_SAFETY_LIMIT_UNQUALIFIED before any
            # nozzle contact motion if the envelope constant this run
            # needs (established_contact_margin_mm or bootstrap_contact_
            # envelope_mm), max_abs_fit_delta, or the repeatability bounds
            # are not configured (not yet hardware-qualified). See
            # nebulaos_probe_pair.py's own header for the exact formulas.
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                probe_z_offset,
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed,
                established_contact_margin_mm=self.established_contact_margin_mm,
                bootstrap_contact_envelope_mm=self.bootstrap_contact_envelope_mm,
                max_abs_fit_delta=self.max_abs_fit_delta,
                min_accepted_samples=self.min_accepted_samples,
                max_repeatability_range=self.max_repeatability_range,
                max_repeatability_stddev=self.max_repeatability_stddev)

            self.z_offset_contact_mode = measurement.contact_mode
            self.z_offset_predicted_nozzle_contact_z = measurement.predicted_nozzle_contact_z
            self.z_offset_commanded_floor_z = measurement.commanded_floor_z
            self.z_offset_raw_probe_trigger_z = measurement.raw_probe_trigger_z
            self.z_offset_sample_count = measurement.repeatability.sample_count
            self.z_offset_accepted_count = measurement.repeatability.accepted_count
            self.z_offset_range = measurement.repeatability.range
            self.z_offset_stddev = measurement.repeatability.stddev

            # §9: a rejected result (bad fit, out-of-envelope, excessive
            # fit delta, or failed repeatability) must NEVER stage a
            # Z-offset, transition to COMPLETE, or call SAVE_CONFIG - the
            # measurement's own `accepted` flag is the single
            # authoritative gate. The toolhead is already lifted clear
            # (measure_probe_nozzle_pair's own final hover()) before this
            # check runs.
            if not measurement.accepted:
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measurement rejected - "
                    "%s (samples: %d taken, %d accepted)" %
                    (measurement.rejection_reason,
                     measurement.repeatability.sample_count,
                     measurement.repeatability.accepted_count))

            new_offset = measurement.probe_z_offset

            if not math.isfinite(new_offset):
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %r is "
                    "not a finite number" % (new_offset,))
            # max_offset_correction_mm bounds the CORRECTION - how far this
            # measurement moves the probe's z_offset from the credible prior
            # it is refining - not the absolute z_offset value itself. A
            # thermal-expansion-driven absolute offset near or past this
            # many mm (e.g. established cold ~1.78mm drifting to ~2.1-2.3mm
            # hot) is real calibration data, not automatically implausible;
            # what would be implausible is THIS RUN disagreeing with the
            # PRIOR by more than max_offset_correction_mm. There is no
            # credible prior in BOOTSTRAP mode (see nebulaos_probe_pair.py's
            # _is_credible_probe_z_offset) - measure_probe_nozzle_pair's own
            # bootstrap_contact_envelope_mm bound already gates plausibility
            # for that case, so this check only applies when refining an
            # ESTABLISHED calibration.
            if measurement.contact_mode == 'established':
                correction = new_offset - probe_z_offset
                if abs(correction) > self.max_offset_correction_mm:
                    raise self.printer.command_error(
                        "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %.5fmm "
                        "implies a %.5fmm correction from the current probe "
                        "z_offset %.5fmm, exceeding "
                        "max_offset_correction_mm=%.5fmm - refusing to "
                        "apply an implausibly large correction"
                        % (new_offset, correction, probe_z_offset,
                           self.max_offset_correction_mm))

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
            # §22 status model: distinguish WHY a run failed where the
            # error message already tells us, rather than collapsing
            # everything to a generic 'error' - a caller polling status
            # can tell "not yet hardware-qualified" apart from "this
            # specific measurement failed quality" apart from "something
            # else broke" without parsing prose.
            text = str(e)
            if 'CONTACT_SAFETY_LIMIT_UNQUALIFIED' in text:
                self.z_offset_state = 'capability_unqualified'
            elif 'measurement rejected' in text:
                self.z_offset_state = 'measurement_quality_failure'
            else:
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
    # NEBULAOS_Z_OFFSET_CALIBRATE SIMULATE_BOOTSTRAP=1 (mission §8: a
    # controlled, simulated virgin state for qualifying the BOOTSTRAP
    # contact envelope without ever touching this unit's real,
    # already-calibrated probe_z_offset)
    # ------------------------------------------------------------------
    def _cmd_z_offset_calibrate_simulate_bootstrap(
            self, gcmd, x, y, probe_x_offset, probe_y_offset, z_offset_probe):
        """Forces probe_z_offset=0.0 into measure_probe_nozzle_pair() for
        THIS call only - _is_credible_probe_z_offset(0.0) is False, so the
        REAL BOOTSTRAP code path (classification, formula, motion, quality
        gates) runs for real, with real nozzle contact - without ever
        reading, mutating, or reapplying this unit's own real,
        already-calibrated probe_offsets.z_offset (never looked up in this
        method at all, unlike the real ESTABLISHED path above). The result
        is reported in its own separate bootstrap_sim_* status namespace
        and is NEVER applied live or staged/SAVE_CONFIG'd - this exists
        purely to physically qualify the bootstrap envelope and exercise
        the bootstrap motion/measurement path on hardware that already has
        a trusted calibration. This is exactly the "controlled temporary/
        simulated virgin state" the mission asks for, with an "exact
        backup/restore" of zero work needed: nothing about the real probe
        is ever touched to begin with, so there is nothing to restore.

        ENVELOPE= is required and always explicit here (never falls back
        to self.bootstrap_contact_envelope_mm) - this command's entire
        purpose is qualifying that value BEFORE it becomes trusted
        production config in the first place; a caller who has already
        set a production default can still pass ENVELOPE=<that value>
        explicitly to re-verify it on a specific unit.
        """
        envelope = gcmd.get_float('ENVELOPE', None)
        if envelope is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE SIMULATE_BOOTSTRAP=1 requires "
                "ENVELOPE=<mm> - this is a qualification aid for "
                "determining bootstrap_contact_envelope_mm itself, so it "
                "never falls back to a config default")

        self.bootstrap_sim_id += 1
        self.bootstrap_sim_state = 'running'
        self.bootstrap_sim_result = None
        self.bootstrap_sim_error = None
        self.bootstrap_sim_envelope_mm = envelope
        self.bootstrap_sim_raw_probe_trigger_z = None
        self.bootstrap_sim_commanded_floor_z = None

        try:
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                0.0,  # forced - see this method's own docstring
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed,
                bootstrap_contact_envelope_mm=envelope,
                max_abs_fit_delta=self.max_abs_fit_delta,
                min_accepted_samples=self.min_accepted_samples,
                max_repeatability_range=self.max_repeatability_range,
                max_repeatability_stddev=self.max_repeatability_stddev)

            self.bootstrap_sim_raw_probe_trigger_z = measurement.raw_probe_trigger_z
            self.bootstrap_sim_commanded_floor_z = measurement.commanded_floor_z

            if measurement.contact_mode != 'bootstrap':
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: internal error - contact_mode=%r, "
                    "expected 'bootstrap' (probe_z_offset=0.0 should "
                    "always classify as BOOTSTRAP)" % (measurement.contact_mode,))

            if not measurement.accepted:
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: measurement rejected - %s "
                    "(samples: %d taken, %d accepted)" %
                    (measurement.rejection_reason,
                     measurement.repeatability.sample_count,
                     measurement.repeatability.accepted_count))

            result = measurement.probe_z_offset
            if not math.isfinite(result):
                raise self.printer.command_error(
                    "SIMULATE_BOOTSTRAP: measured value %r is not a "
                    "finite number" % (result,))
        except Exception as e:
            self.bootstrap_sim_state = 'error'
            self.bootstrap_sim_error = _sanitize_error(e)
            raise

        self.bootstrap_sim_state = 'complete'
        self.bootstrap_sim_result = result
        self.gcode.respond_info(
            "SIMULATE_BOOTSTRAP: measured %.5f mm at X=%.3f Y=%.3f with "
            "ENVELOPE=%.3fmm (commanded_floor_z=%.5f) - NOT applied or "
            "staged, this is a simulation only"
            % (result, x, y, envelope, measurement.commanded_floor_z))

    # ------------------------------------------------------------------
    # NEBULAOS_AUTO_CALIBRATE / NEBULAOS_CALIBRATION_CANCEL (mission
    # §11/§12): a single orchestrated sequence - preflight, home, PID bed,
    # PID hotend, nozzle clean, establish the calibration thermal state,
    # stabilize, localized Z-offset, upstream bed mesh, final validation,
    # ONE persistence transaction (SAVE_CONFIG), restart, post-restart
    # verification. Every real algorithm (PID_CALIBRATE, BED_MESH_
    # CALIBRATE, G28, SAVE_CONFIG) stays pristine upstream, dispatched via
    # gcode.run_script_from_command() exactly like nozzle_clear.py's own
    # _move() helper dispatches G1 - this module owns only the sequencing,
    # the journal, and reading back each stage's own already-existing
    # status fields (self.z_offset_result, etc.) to build the one
    # expected_values dict the post-restart verification checks. Axis
    # Twist is deliberately NOT part of this sequence - manual Axis Twist
    # is separate, owner-initiated upstream AXIS_TWIST_COMPENSATION_
    # CALIBRATE maintenance, not part of automatic calibration.
    # ------------------------------------------------------------------
    def _auto_calibrate_advance(self, journal, stage, now):
        """Advance both the in-memory status (what NEBULAOS_CALIBRATION_
        STATUS reports right now) and the on-disk journal (what a future
        boot reads back after a crash) together, in that order - the
        in-memory field is genuinely for THIS process only; a torn write
        to the journal file would still leave this process's own status
        correct for as long as it keeps running."""
        self.auto_calibrate_stage = stage
        calibration_journal.advance_stage(journal, stage, now)
        calibration_journal.write_journal(journal, path=self.journal_path)

    def _auto_calibrate_check_cancel(self, journal, now):
        if not self._auto_calibrate_cancel_requested:
            return
        self._auto_calibrate_cancel_requested = False
        calibration_journal.mark_cancelled(journal, now)
        calibration_journal.write_journal(journal, path=self.journal_path)
        raise self.printer.command_error(
            "NEBULAOS_AUTO_CALIBRATE: cancelled by NEBULAOS_CALIBRATION_CANCEL")

    cmd_auto_calibrate_help = (
        "Fully automatic calibration: PID (bed+hotend), nozzle clean, "
        "localized Z-offset, upstream bed mesh, one SAVE_CONFIG+restart. "
        "Axis Twist is NOT included - see AXIS_TWIST_COMPENSATION_CALIBRATE")

    def cmd_auto_calibrate(self, gcmd):
        if self.auto_calibrate_state == 'running':
            raise self.printer.command_error(
                "NEBULAOS_AUTO_CALIBRATE: a calibration is already in progress")

        now = time.time()
        self.auto_calibrate_id += 1
        self.auto_calibrate_state = 'running'
        self.auto_calibrate_stage = None
        self.auto_calibrate_error = None
        self.auto_calibrate_result = None
        self._auto_calibrate_cancel_requested = False
        journal = calibration_journal.new_journal(
            self.auto_calibrate_id, 'auto_calibrate', now)
        calibration_journal.write_journal(journal, path=self.journal_path)

        def run(script):
            self.gcode.run_script_from_command(script)

        try:
            # preflight: the same load-cell/reference-point checks
            # cmd_z_offset_calibrate() already performs, run up front so a
            # missing prerequisite is caught before any heating/motion at
            # all rather than partway through the sequence.
            self._auto_calibrate_advance(journal, 'preflight', time.time())
            z_offset_probe = self.printer.lookup_object(
                'nebulaos_z_offset_probe', None)
            if z_offset_probe is None:
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: no [nebulaos_z_offset_probe] "
                    "is configured on this printer")
            if not z_offset_probe.get_status(
                    self.reactor.monotonic())['is_calibrated']:
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: the load cell has not been "
                    "calibrated yet - run LOAD_CELL_CALIBRATE first")
            self._resolve_reference_xy(gcmd)
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'home', time.time())
            run('G28')
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'pid_bed', time.time())
            run('PID_CALIBRATE HEATER=heater_bed TARGET=%.1f'
                % (self.pid_bed_target,))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'pid_hotend', time.time())
            run('PID_CALIBRATE HEATER=extruder TARGET=%.1f'
                % (self.pid_hotend_target,))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'nozzle_clean', time.time())
            run('NEBULAOS_NOZZLE_CLEAN')
            self._auto_calibrate_check_cancel(journal, time.time())

            # z_offset_reference_temp (NOT pid_hotend_target/230C) -
            # root-caused 2026-09-02/03: measuring Z-offset shortly after
            # a long hot dwell biased the load-cell contact reading low
            # by ~0.6mm (nozzle ooze/contamination accumulated during
            # PID-hotend tune + hot nozzle-clean). The bed stays at its
            # intended calibration/mesh temperature (pid_bed_target) -
            # only the nozzle reference for THIS specific measurement is
            # lower. See this module's own __init__ comment for the full
            # evidence and this constant's own header.
            self._auto_calibrate_advance(
                journal, 'establish_thermal_state', time.time())
            run('M140 S%.1f' % (self.pid_bed_target,))
            run('M104 S%.1f' % (self.z_offset_reference_temp,))
            run('M190 S%.1f' % (self.pid_bed_target,))
            run('M109 S%.1f' % (self.z_offset_reference_temp,))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'stabilize', time.time())
            if self.thermal_soak_seconds > 0:
                run('G4 P%d' % (int(self.thermal_soak_seconds * 1000),))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(
                journal, 'localized_z_offset', time.time())
            run('NEBULAOS_Z_OFFSET_CALIBRATE')
            if self.z_offset_state != 'complete' or self.z_offset_result is None:
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: localized Z-offset did not "
                    "complete (state=%r)" % (self.z_offset_state,))
            z_offset_result = self.z_offset_result
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(journal, 'bed_mesh', time.time())
            run('BED_MESH_CALIBRATE PROFILE=%s' % (self.bed_mesh_profile,))
            self._auto_calibrate_check_cancel(journal, time.time())

            self._auto_calibrate_advance(
                journal, 'final_validation', time.time())
            bed_mesh = self.printer.lookup_object('bed_mesh', None)
            mesh_status = (bed_mesh.get_status(self.reactor.monotonic())
                            if bed_mesh is not None else {})
            if not mesh_status.get('profile_name'):
                raise self.printer.command_error(
                    "NEBULAOS_AUTO_CALIBRATE: final validation failed - "
                    "no bed_mesh profile is active after BED_MESH_CALIBRATE")
            expected_values = {
                'bltouch.z_offset': round(z_offset_result, 3),
                'bed_mesh.profile': mesh_status['profile_name'],
            }
            self._auto_calibrate_check_cancel(journal, time.time())

            calibration_journal.mark_commit_requested(
                journal, expected_values, time.time())
            calibration_journal.write_journal(journal, path=self.journal_path)
            self.auto_calibrate_stage = 'commit'
            self.gcode.respond_info(
                "NEBULAOS_AUTO_CALIBRATE: all stages complete (z_offset=%.5f, "
                "bed_mesh profile=%r) - committing with SAVE_CONFIG, "
                "printer will restart"
                % (z_offset_result, mesh_status['profile_name']))
            run('SAVE_CONFIG')
            # SAVE_CONFIG restarts klippy - if we ever reach the next line,
            # SAVE_CONFIG itself refused (e.g. nothing was staged) rather
            # than restarting, which upstream configfile.py treats as a
            # command_error, not a silent return - so this is unreachable
            # in the success path, kept only so a future upstream change
            # in that behavior fails loudly here instead of leaving
            # auto_calibrate_state stuck at 'running' forever.
            raise self.printer.command_error(
                "NEBULAOS_AUTO_CALIBRATE: SAVE_CONFIG returned without "
                "restarting - this should never happen")
        except Exception as e:
            if journal.get('commit_requested'):
                # mark_commit_requested() already ran and was written to
                # disk BEFORE the SAVE_CONFIG call above - in production
                # that call never returns at all (klippy restarts); any
                # exception surfacing from it here (or from anything
                # after it, though nothing runs after it on the success
                # path) must NOT downgrade the already-correct
                # commit_requested/restart_pending/verification_pending
                # journal state back to an error - that would be
                # actively wrong, since the commit truly happened. Only
                # _handle_ready()'s own post-restart verification may
                # ever change this journal's state from here.
                raise
            self.auto_calibrate_state = (
                'cancelled' if 'cancelled by NEBULAOS_CALIBRATION_CANCEL'
                in str(e) else 'error')
            self.auto_calibrate_error = _sanitize_error(e)
            if journal['state'] not in (calibration_journal.STATE_CANCELLED,):
                calibration_journal.mark_error(
                    journal, self.auto_calibrate_error, time.time())
                calibration_journal.write_journal(journal, path=self.journal_path)
            raise

    cmd_calibration_cancel_help = (
        "Request cancellation of an in-progress NEBULAOS_AUTO_CALIBRATE - "
        "takes effect at the next stage boundary, never mid-stage")

    def cmd_calibration_cancel(self, gcmd):
        if self.auto_calibrate_state != 'running':
            raise self.printer.command_error(
                "NEBULAOS_CALIBRATION_CANCEL: no NEBULAOS_AUTO_CALIBRATE "
                "is currently running")
        self._auto_calibrate_cancel_requested = True
        self.gcode.respond_info(
            "NEBULAOS_CALIBRATION_CANCEL: requested - will take effect at "
            "the next stage boundary")

    def _handle_ready(self, *args):
        """klippy:ready - runs on EVERY klippy startup, including the one
        immediately after NEBULAOS_AUTO_CALIBRATE's own SAVE_CONFIG
        restart. Reads the on-disk journal (never the in-memory status,
        which is always fresh/empty in a new process) and, if it shows a
        commit that was requested but never verified, checks the real,
        just-reloaded config against what was expected and records the
        result - the only code path that ever clears verification_pending.
        A journal showing anything else (no file, or an already-verified/
        errored/cancelled state) is left untouched - this only ever acts
        on a genuine "restart just happened, verification still owed"
        case."""
        journal = calibration_journal.read_journal(path=self.journal_path)
        if journal is None or not journal.get('verification_pending'):
            return
        expected = journal.get('expected_values') or {}
        errors = []
        probe_obj = self.printer.lookup_object('probe', None)
        expected_z_offset = expected.get('bltouch.z_offset')
        if expected_z_offset is not None:
            if probe_obj is None:
                errors.append('probe object is not configured')
            else:
                actual = probe_obj.get_offsets()[2]
                if abs(actual - expected_z_offset) > 0.0005:
                    errors.append(
                        'bltouch.z_offset: expected %.5f, found %.5f'
                        % (expected_z_offset, actual))
        expected_profile = expected.get('bed_mesh.profile')
        if expected_profile is not None:
            bed_mesh = self.printer.lookup_object('bed_mesh', None)
            actual_profile = (
                bed_mesh.get_status(self.reactor.monotonic()).get('profile_name')
                if bed_mesh is not None else None)
            if actual_profile != expected_profile:
                errors.append(
                    'bed_mesh.profile: expected %r, found %r'
                    % (expected_profile, actual_profile))
        now = time.time()
        if errors:
            calibration_journal.mark_verification_result(
                journal, success=False, result=None,
                error='; '.join(errors), now=now)
        else:
            calibration_journal.mark_verification_result(
                journal, success=True, result=dict(expected), error=None,
                now=now)
        calibration_journal.write_journal(journal, path=self.journal_path)
        if journal['calibration_id'] == self.auto_calibrate_id or (
                self.auto_calibrate_id == 0):
            self.auto_calibrate_id = journal['calibration_id']
            self.auto_calibrate_state = journal['state']
            self.auto_calibrate_stage = journal['stage']
            self.auto_calibrate_error = journal['error']
            self.auto_calibrate_result = journal['result']

    # ------------------------------------------------------------------
    # NEBULAOS_AXIS_TWIST_CALIBRATE
    # ------------------------------------------------------------------
    cmd_axis_twist_calibrate_help = (
        "Automatic Axis Twist (AXIS=X|Y|BOTH) - HARD BLOCKED pending "
        "remote load-cell contact hardware qualification; use pristine "
        "upstream AXIS_TWIST_COMPENSATION_CALIBRATE for a manual run")

    _REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED_MSG = (
        "REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED: automatic Axis Twist "
        "calibration is hard-blocked - HX711 nozzle contact at a REMOTE "
        "bed point (away from [nebulaos_z_offset_probe]'s own qualified "
        "reference point) has not been hardware-qualified. Zero movement, "
        "zero nozzle contact, and zero compensation/config changes were "
        "made. For a manual calibration, call pristine upstream "
        "AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=%s directly.")

    def cmd_axis_twist_calibrate(self, gcmd):
        """Phase 2 contact-safety mission (§4/§14): this command is now
        ONLY the automatic (load-cell) Axis Twist frontend - there is no
        METHOD=MANUAL passthrough any more (see this module's own header
        comment: call pristine upstream AXIS_TWIST_COMPENSATION_CALIBRATE
        directly for a manual run, it needs no wrapper). The automatic
        path itself is hard-blocked pending hardware qualification of
        remote HX711 nozzle contact: this handler performs ZERO printer
        object lookups and ZERO motion for any AXIS value - it cannot
        accidentally touch hardware even if a future edit here got the
        gating logic below wrong, because there is no hardware-touching
        code left to reach at all.
        """
        # AXIS has no default on purpose - per this project's own rules,
        # ambiguous input must never silently select a (potentially
        # unsafe/unintended) axis, even for a status/error response.
        axis = gcmd.get('AXIS', None)
        if axis is None:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: AXIS=X|Y|BOTH is required")
        axis = axis.upper()
        if axis not in ('X', 'Y', 'BOTH'):
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: unknown AXIS='%s' - "
                "expected X, Y, or BOTH" % (axis,))

        self.axis_twist_id += 1
        self.axis_twist_method = 'LOAD_CELL'
        self.axis_twist_current_axis = None
        msg = self._REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED_MSG % (
            axis if axis != 'BOTH' else 'X (and separately AXIS=Y)')
        if axis in ('X', 'BOTH'):
            self._axis_twist_set_axis_state('X', 'capability_unqualified', None, msg)
        if axis in ('Y', 'BOTH'):
            self._axis_twist_set_axis_state('Y', 'capability_unqualified', None, msg)
        raise self.printer.command_error(
            "NEBULAOS_AXIS_TWIST_CALIBRATE: %s" % (msg,))

    def _axis_twist_set_axis_state(self, axis, state, result, error):
        if axis == 'X':
            self.axis_twist_x_state = state
            self.axis_twist_x_result = result
            self.axis_twist_x_error = error
        else:
            self.axis_twist_y_state = state
            self.axis_twist_y_result = result
            self.axis_twist_y_error = error

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
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: axis_twist X state=%s result=%s "
            "error=%s | Y state=%s result=%s error=%s | active=%s "
            "sample=%s/%s"
            % (status['axis_twist_x_state'], status['axis_twist_x_result'],
               status['axis_twist_x_error'], status['axis_twist_y_state'],
               status['axis_twist_y_result'], status['axis_twist_y_error'],
               status['axis_twist_current_axis'],
               status['axis_twist_sample_index'],
               status['axis_twist_sample_total']))


def load_config(config):
    return NebulaOSCalibration(config)
