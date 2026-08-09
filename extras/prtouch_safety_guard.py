# Movement-path guard - makes it structurally impossible for a non-motion validation
# session (interactive bring-up over SSH, or an automated test) to accidentally trigger
# real stepper motion or heating, by intercepting the exact choke points every motion-
# capable path in this module set funnels through.
#
# NOT installed automatically, anywhere, ever - this is opt-in instrumentation for a human
# (or agent) doing live, zero-motion bring-up on the real printer, not a production safety
# feature of the probe itself. Default posture is "does nothing unless explicitly
# installed" (see install()/guard()) - matches the 2026-08-06 non-motion audit's own
# working constraint (READ_PRES/deal_avgs_prtouch allowed, everything else blocked).
#
# Two choke points cover 100% of this codebase's actual motion-capable paths (verified by
# grep across prtouch_probe.py/prtouch_nozzle.py/prtouch_v2.py/z_compensate.py - there is
# no direct toolhead.move()/manual_stepper call anywhere; every real move goes through one
# of these two):
#   1. PrtouchMCU.start_step_prtouch_cmd.send() - the raw MCU pulse-train command
#      (touch_probe, safe_move_z, clear_nozzle's wipe drag all funnel through this).
#   2. gcode.run_script_from_command() - every G1/G28/G29/BED_MESH_CALIBRATE/SAVE_CONFIG/
#      macro-command invocation in this module set goes through this one method.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import contextlib

#: G-code commands (case-insensitive, matched on the first whitespace-delimited token)
#: that are motion- or heating-capable, or persist config - exactly the task's own
#: "do not execute" list plus the underlying primitives those macros are built from.
BLOCKED_GCODE_PREFIXES = {
    'G0', 'G1', 'G2', 'G3', 'G28', 'G29',
    'BED_MESH_CALIBRATE', 'SAVE_CONFIG', 'Z_OFFSET_APPLY_PROBE',
    'Z_OFFSET_CALIBRATION', 'NOZZLE_CLEAR', 'CRTENSE_NOZZLE_CLEAR', 'SAFE_MOVE_Z',
    'FORCE_MOVE', 'MANUAL_STEPPER', 'SET_HEATER_TEMPERATURE', 'M104', 'M109', 'M140', 'M190',
}

#: Explicitly allowed regardless of the blocklist above - zero-motion, zero-heating reads.
ALLOWED_GCODE_PREFIXES = {'READ_PRES', 'M118', 'RESPOND'}


class MovementBlockedError(Exception):
    """Raised the instant a guarded session attempts a real motion/heating/persistence
    command - never caught internally, always surfaces to the caller immediately."""


def _first_token(script):
    return script.strip().split(None, 1)[0].upper() if script.strip() else ''


def _guarded_gcode_run(original_run, script):
    token = _first_token(script)
    if token in ALLOWED_GCODE_PREFIXES:
        return original_run(script)
    if token in BLOCKED_GCODE_PREFIXES:
        raise MovementBlockedError(
            "movement guard: blocked gcode script %r (command %r is motion/heating/"
            "persistence-capable and physical motion is out of scope right now)"
            % (script, token))
    # Unknown command: fail closed, not open - an un-recognized command could still be a
    # macro that ultimately moves something, and the whole point of this guard is to never
    # let an unexamined path through silently.
    raise MovementBlockedError(
        "movement guard: blocked unrecognized gcode script %r (command %r is not on "
        "either the explicit allow- or block-list - add it deliberately to one before "
        "running this under the guard)" % (script, token))


def _guarded_start_step_send(original_send, args):
    # start_step_prtouch's own field order (see prtouch_mcu.py's format string): oid, dir,
    # send_ms, step_cnt, step_us, acc_ctl_cnt, low_spd_nul, send_step_duty, auto_rtn -
    # step_cnt is index 3. A step_cnt of 0 is the documented "stop"/disarm idiom (see
    # PrtouchMCU.stop()) and moves nothing, so it's allowed through; anything else is a
    # real, physical pulse-train command.
    step_cnt = args[3] if len(args) > 3 else None
    if step_cnt:
        raise MovementBlockedError(
            "movement guard: blocked start_step_prtouch with step_cnt=%r (args=%r) - this "
            "would command real stepper motion" % (step_cnt, args))
    return original_send(args)


@contextlib.contextmanager
def guard(pv2, gcode=None):
    """Installs the guard for the duration of a `with` block, restoring the original,
    unguarded methods afterward even if the block raises. `pv2` is a PRTouchV2 instance
    (or anything exposing .mcu.start_step_prtouch_cmd); `gcode` defaults to pv2.gcode."""
    if gcode is None:
        gcode = pv2.gcode
    cmd = pv2.mcu.start_step_prtouch_cmd
    original_send = cmd.send
    original_run = gcode.run_script_from_command

    cmd.send = lambda args: _guarded_start_step_send(original_send, args)
    gcode.run_script_from_command = lambda script: _guarded_gcode_run(original_run, script)
    try:
        yield
    finally:
        cmd.send = original_send
        gcode.run_script_from_command = original_run
