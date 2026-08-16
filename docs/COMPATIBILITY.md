# Compatibility contract

NebulaOS runs **official, unmodified Klipper** and activates this repository's modules
alongside it. That removes the fork, but it introduces one real risk in its place: the two
halves can drift apart, and neither Klipper, Moonraker, nor Mainsail can detect it — none of
them know this extension set exists.

This document is the contract that closes that gap: what is checked, by whom, when, and what
happens when a check fails.

## The risk, stated concretely

Klipper publishes no API version and offers no stability guarantee for the host-side
interfaces extension code uses. Drift is not hypothetical here — it has already happened
once. Mainline commit `c89393cda` (2026-02-26) renamed `MCU.register_response()` to
`MCU.register_serial_response()`. `prtouch_mcu.py` called the old name at three sites. Against
that Klipper, PRTouch fails at load time.

The important part is what that would have looked like without a gate: no error at config
parse, no warning at startup, and a failure surfacing during a probe descent. This extension
set drives a load-cell probe into a heated bed. An unqualified Klipper underneath it is a
physical-safety change wearing the costume of a version bump.

## Failure policy

**Fail closed, loudly, specifically.** Every check refuses to let Klippy start and names the
specific thing that failed, the value found, the value expected, and what to do about it.

There is no degraded mode and no warn-and-continue path. For a printer, *"did not start, and
said exactly why"* is strictly better than *"started, but the probe is subtly wrong"*.

The one deliberate exception is `klipper.allow_unqualified` in the manifest, which is an
explicit developer opt-in, defaults to `false` in the shipped manifest, and is asserted to be
`false` by the test suite.

## Who enforces what

Three layers, each catching what the others structurally cannot.

| Layer | Mechanism | Catches |
|---|---|---|
| **Moonraker** | `pinned_commit` on the official `Klipper3d/klipper` remote, in `moonraker.conf` | Stops a normal user ever reaching an unqualified Klipper in the first place. The primary defence — it prevents rather than detects. |
| **NebulaOS platform** (`NebulaOS-firmware`, boot-time activation + update supervisor) | Composition integrity, the collision guard, the `c_helper.so` mtime invariant, paired (klipper, extensions) rollback | Everything that is a property of the Klipper checkout or the device, not of this repository |
| **This repository** (`extras/nebulaos_compat.py`) | Manifest + symbol introspection at config load | API drift on a device the platform pre-flight never saw — a hand-updated checkout, a restored backup, a developer install. Klipper knows nothing about NebulaOS, so this **must** be extension code; there is nowhere else to put it. |

## `nebulaos-extensions.json`

The machine-readable manifest at the repository root. Read by `nebulaos_compat.py` at startup,
and by the platform's composition and update steps.

| Key | Meaning |
|---|---|
| `compat_schema_version` | Schema version of this file. `nebulaos_compat.py` refuses a version it does not know rather than interpreting it optimistically — a newer schema may imply a check the running build does not know it should perform. |
| `extensions_version` | Human-readable version of this extension set. |
| `nebulaos_api_level` | This extension set's contract with the NebulaOS **platform**. Deliberately not a claim about Klipper's API — Klipper publishes no such number, and inventing one nobody else maintains would be fiction. |
| `klipper.qualified_commit` | The exact Klipper commit this set was qualified against. |
| `klipper.min_commit` / `max_commit` | Optional compatible ancestry range. Decided with real `git merge-base --is-ancestor`, **never** string comparison — git SHAs carry no ordering, so a string-compared range check would look like it worked and would not. Both `null` today: the qualified commit is the only accepted value. |
| `klipper.allow_unqualified` | Developer opt-in. `false` in the shipped manifest. |
| `required_klipper_symbols` | Klipper APIs this set depends on, as `module:Attr.attr`. Probed against the Klipper actually loaded in the running process. |
| `forbidden_klipper_symbols` | APIs this set has migrated **off**. Their reappearance means the installed Klipper is older than the qualified one, or is not official Klipper. |
| `modules` | Every managed module, with `role`: `runtime` or `test`. A deployment may skip `test` modules; a missing `runtime` module is never legitimate. |
| `sensor_types` | Sensor types this set registers with `[heaters]`, and which module provides each. |
| `composition` | The platform's composition contract — see below. |
| `chelper` | The platform's `c_helper.so` contract — see below. |

## What `nebulaos_compat.py` checks

Run as an ordinary config section, in dependency order — the manifest must be readable before
anything can be checked against it, and its shape understood before its contents are trusted.

1. **Manifest present, parseable, and of a known schema version.** Its absence means a partial
   or hand-assembled deployment, which is exactly the state this gate exists to refuse.
2. **Every declared managed module has a real source file.** A mismatch between manifest and
   tree means composition cannot be trusted — including, potentially, for a safety-relevant
   module.
3. **Every required Klipper symbol exists**, and no forbidden one has come back. This is the
   highest-value check in the file: it is the only one that would have caught the
   `register_response` rename automatically, before any motion. All problems are accumulated
   into one message rather than surfacing one restart at a time.
4. **The installed Klipper commit** is the qualified one, or inside a declared ancestry range.
5. **The platform's `c_helper.so` verdict**, if it published one.
6. **Every declared sensor type is registered**, by force-loading its providing module.

## Section ordering

`[nebulaos_compat]` should be the **first** NebulaOS section in `printer.cfg`, and it must
appear before any `[temperature_sensor]` section using a NebulaOS sensor type.

This is not stylistic. `heaters.setup_sensor()` resolves `sensor_type` against a plain dict
that `add_sensor_factory()` populates, so the providing module must have loaded first.
Klipper's only genuinely order-free bootstrap for this is `klippy/extras/temperature_sensors.cfg`
— an upstream-tracked file NebulaOS deliberately does not patch, because patching it would
dirty the Klipper checkout and make this a fork again.

So the ordering dependency is removed as far as it can be, and made loud where it cannot:

- `nebulaos_temperature_mcu.py` registers its factory from `load_config()`, so a bare
  `[nebulaos_temperature_mcu]` section anywhere is sufficient;
- registration is idempotent and position-independent — the test suite runs all six
  permutations of three independent registration triggers and requires every one to work;
- `nebulaos_compat.py` force-loads each declared provider and then **verifies** the
  registration, so a genuine ordering mistake produces a message naming the sensor type and
  its provider instead of Klipper's bare `Unknown temperature sensor`.

## Handed to the platform (Stage 2)

Two checks are declared here with a fixed interface but deliberately not implemented in this
repository, because neither can be answered from inside it.

### Composition integrity — the collision guard

**Mandatory.** If upstream Klipper ever ships a file at a path this repository also manages,
git replaces the symlink with upstream's regular file **silently** — exit code 0, no warning.
The extension is then shadowed with no error anywhere. The names most exposed are the vendored
community ones: `gcode_shell_command` and `virtual_pins` are exactly the kind of module
mainline could adopt.

By the time Klippy is running, a collision has already happened and this process is importing
upstream's file rather than ours — so the check has to live in the platform, before Klippy
starts. `composition.require_symlink_resolving_inside_source` is `true`, and the platform MUST
verify after composition and after any Klipper update that every destination path is a symlink
resolving inside `source_dir`, refusing to activate otherwise.

### `c_helper.so` mtime invariant

Klipper's `klippy/chelper/__init__.py` decides whether to rebuild its C library by comparing
**mtimes**, not hashes: if the prebuilt target is newer than every source, it returns early and
never invokes gcc. NebulaOS ships a cross-compiled `c_helper.so` and the device has no
toolchain, so a rebuild attempt does not merely take a while — it raises, and Klippy does not
start.

Enforcing this needs firmware build-time information that does not exist inside this
repository, so the platform owns it. The manifest's `chelper` block declares the requirement
and names an optional `platform_result_file`; when set, `nebulaos_compat.py` reads it and
refuses to start on a failing verdict, so the failure appears as a named preflight error
rather than a gcc crash part-way through boot. It is `null` today, meaning the platform owns
this end to end.

## Moving the qualified pin

Advancing to a newer Klipper is a deliberate act, not a side effect of an update. In order:

1. Compose the candidate Klipper with this repository and run the full test suite — it must
   pass, and `git status --porcelain` must be empty in **both** checkouts.
2. Re-verify every entry in `required_klipper_symbols` against the candidate, and add any new
   API the migration comes to depend on.
3. Rebuild and re-ship `c_helper.so` for the candidate, and re-establish the mtime invariant.
4. Re-qualify on real hardware. Static checks cannot clear probe and homing behaviour; that is
   the residual risk this whole architecture concentrates in one place, on purpose.
5. Update `klipper.qualified_commit` and `extensions_version`, then move Moonraker's
   `pinned_commit` to match.

Steps 1–3 are automatable. Step 4 is not, and no amount of green tests substitutes for it.
