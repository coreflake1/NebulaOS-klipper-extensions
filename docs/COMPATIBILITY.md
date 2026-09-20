# Compatibility contract

NebulaOS runs **official, unmodified Klipper** and activates this repository's modules
alongside it. That removes the fork, but it introduces one real risk in its place: the two
halves can drift apart, and neither Klipper, Moonraker, nor Mainsail can detect it — none of
them know this extension set exists.

This document is the contract that closes that gap: what is checked, by whom, when, and what
happens when a check fails.

## The risk, stated concretely

Klipper publishes no API version and offers no stability guarantee for the host-side
interfaces extension code uses. Drift is not hypothetical here — it has already happened once.

> **Historical incident (Phase 1, PRTouch era).** Mainline commit `c89393cda` (2026-02-26)
> renamed `MCU.register_response()` to `MCU.register_serial_response()`. `prtouch_mcu.py`
> called the old name at three sites, so against that Klipper it failed at load time. Both
> `prtouch_mcu.py` and the rest of the PRTouch runtime stack were deleted in Phase 1.8B and no
> longer exist; `MCU.register_serial_response` is today an ordinary entry in this manifest's
> `required_klipper_symbols`. The incident is kept here because it is the reason the symbol
> check exists, not because it describes a current hazard.

The important part is what that would have looked like without a gate: no error at config
parse, no warning at startup, and a failure surfacing during a descent toward the bed. These
modules drive the nozzle into contact with a heated bed for Z-offset and calibration work. An
untested Klipper underneath them is a physical-safety change wearing the costume of a version
bump.

## Failure policy

**Fail closed, loudly, specifically.** Every check refuses to let Klippy start and names the
specific thing that failed, the value found, the value expected, and what to do about it.

There is no degraded mode and no warn-and-continue path. For a printer, *"did not start, and
said exactly why"* is strictly better than *"started, but the probe is subtly wrong"*.

There is no developer opt-out. An earlier design carried one, `klipper.allow_unqualified`,
alongside a pinned `klipper.qualified_commit`; that whole model is **retired**. Neither key
exists in the manifest any more, and the firmware build refuses to produce an image if either
reappears — `NebulaOS-firmware/scripts/build/04-cross-compile-app-stack.sh` fails with
`FATAL: extensions manifest still contains qualified_commit` (likewise `allow_unqualified`) and
otherwise prints `extensions manifest: no global Klipper version gate (correct)`.

## Who enforces what

Three layers, each catching what the others structurally cannot.

| Layer | Mechanism | Catches |
|---|---|---|
| **Moonraker** | an ordinary `update_manager` section on the official `Klipper3d/klipper` remote, with **no** `pinned_commit` — Klipper updates independently of this extension set | Nothing, by design. It is not a gate. The offline safety net is the immutable recovery copy on the rootfs: `nebulaos-recover klipper` restores the exact qualified version. See `NebulaOS-firmware`'s `overlay/etc/nebulaos/moonraker/klipper-pin.conf`. |
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
| *(retired)* `klipper.*` | A `klipper` block once carried `qualified_commit`, `min_commit`/`max_commit` and `allow_unqualified`. **No such block exists today**, and the firmware build fails if one reappears. Klipper and this extension set are independently updateable; the enforced contract is symbol-level, not commit-level. |
| `required_klipper_symbols` | Klipper APIs this set depends on, as `module:Attr.attr`. Probed against the Klipper actually loaded in the running process. |
| `forbidden_klipper_symbols` | APIs this set has migrated **off**. Their reappearance means the installed Klipper is older than the qualified one, or is not official Klipper. |
| `modules` | Every managed module, with `role`: `runtime` or `test`. A deployment may skip `test` modules; a missing `runtime` module is never legitimate. |
| `sensor_types` | Sensor types this set registers with `[heaters]`, and which module provides each. |
| `composition` | The platform's composition contract, and the collision guard this repository re-checks — see below. |
| `chelper` | The platform's `c_helper.so` contract, and where it publishes its verdict — see below. |

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
4. *(No Klipper-commit check.)* There is deliberately no global Klipper version gate: Klipper
   and this extension set update independently, and the firmware build enforces that the
   manifest carries no `qualified_commit`. Compatibility is decided by what the installed
   Klipper actually **provides** — check 3 — not by which commit it claims to be.
5. **Composition integrity** — every runtime module is reachable as a symlink resolving
   inside this repository, so a module silently shadowed by an upstream file is caught
   before any other managed module loads.
6. **The platform's `c_helper.so` verdict.**
7. **Every declared sensor type is registered**, by force-loading its providing module.

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

## Shared with the platform

Two checks are owned by the platform (`NebulaOS-firmware`), because only it can run them
*before* Klippy starts — the one place either failure can be prevented rather than merely
noticed. Both interfaces were declared here first and are wired to a real platform
implementation as of Stage 2, without the manifest shape having to change to do it.

### Composition integrity — the collision guard

**Mandatory.** If upstream Klipper ever ships a file at a path this repository also manages,
git replaces the symlink with upstream's regular file **silently** — exit code 0, no warning.
The extension is then shadowed with no error anywhere. The names most exposed are the vendored
community ones: `gcode_shell_command` and `virtual_pins` are exactly the kind of module
mainline could adopt.

By the time Klippy is running, a collision has already happened — so the authoritative check
lives in the platform, before Klippy starts.
`composition.require_symlink_resolving_inside_source` is `true`, and the platform MUST verify
after composition and after any Klipper update that every destination path is a symlink
resolving inside `source_dir`, refusing to activate otherwise.
`NebulaOS-firmware`'s `/etc/nebulaos-klipper-compose.sh` does exactly that, on every activation
and inside every update transaction, and treats a regular file at a managed path as a hard
error that is never overwritten and never worked around.

`nebulaos_compat.py` re-checks it as a second layer, for the deployments the platform never
saw: a hand-updated checkout, a restored backup, a developer install, a device composed by
older firmware. Because `[nebulaos_compat]` is the first NebulaOS section, this still runs
before any other managed module has been imported. It is skipped entirely when
`require_symlink_resolving_inside_source` is `false` — which is what lets a copy-based
deployment, or the immutable factory-fallback tree where the modules are deliberately real
files, use this same module without misdescribing how it was assembled.

### `c_helper.so` mtime invariant

Klipper's `klippy/chelper/__init__.py` decides whether to rebuild its C library by comparing
**mtimes**, not hashes: if the prebuilt target is newer than every source, it returns early and
never invokes gcc. NebulaOS ships a cross-compiled `c_helper.so` and the device has no
toolchain, so a rebuild attempt does not merely take a while — it raises, and Klippy does not
start.

Enforcing this needs build-time and boot-time information that does not exist inside this
repository, so the platform owns the decision and publishes the result.
`platform_result_file` is `.nebulaos-chelper-verdict.json`, relative to the Klipper checkout —
so the verdict travels with the tree it describes, and a migration that replaces
`apps/klipper` takes any stale verdict with it rather than leaving one behind describing a
tree that no longer exists.

`NebulaOS-firmware` writes it from `/etc/nebulaos-chelper-preflight.sh` at every activation and
inside every update transaction, and bakes one into the immutable `/opt/klipper` copy at build
time (that tree is a read-only squashfs, so nothing can write it there at boot). The build
additionally *enforces* the invariant rather than hoping for it: `cp -r` does not preserve
mtimes and `git read-tree` rewrites them, so without an explicit step the ordering inside a
seed archive would be decided by directory-walk order.

`nebulaos_compat.py` reads the verdict and refuses to start on anything but a pass — including
when the file is absent, since its absence means the platform never checked. Setting it back to
`null` disables this consumer and leaves the invariant entirely to the platform.

## Advancing to a newer Klipper

There is no pin in this manifest to move, but re-qualifying against a newer Klipper is still a
deliberate act rather than something an update does to you. In order:

1. Compose the candidate Klipper with this repository and run the full test suite — it must
   pass, and `git status --porcelain` must be empty in **both** checkouts.
2. Re-verify every entry in `required_klipper_symbols` against the candidate, and add any new
   API the migration comes to depend on.
3. Rebuild and re-ship `c_helper.so` for the candidate, and re-establish the mtime invariant.
4. Re-qualify on real hardware. Static checks cannot clear probe and homing behaviour; that is
   the residual risk this whole architecture concentrates in one place, on purpose.
5. Update `extensions_version`. There is no `klipper.qualified_commit` to update and no
   Moonraker `pinned_commit` to move; what ships is decided by `KLIPPER_PIN` in
   `NebulaOS-firmware/manifests/dependencies.conf`, which is a firmware-side reviewed change.

Steps 1–3 are automatable. Step 4 is not, and no amount of green tests substitutes for it.

## Python extension reload behaviour

Klipper's `FIRMWARE_RESTART` command resets the MCU firmware and re-reads `printer.cfg`, but
it does **not** reimport Python modules. The running Klippy process keeps already-loaded
bytecode in memory. To pick up changes to `.py` extension files, a full service restart is
required:

    /etc/init.d/S55klipper restart

This affects hardware testing of extension code changes: after editing a `.py` file on the
device and running `FIRMWARE_RESTART`, the old code is still executing. Discovered during
Phase 1.8 hardware qualification (gate 11) when a confirmed-correct fix appeared to have no
effect through multiple `FIRMWARE_RESTART` cycles.
