# NebulaOS Klipper extensions

Host-side Klipper extras used by [NebulaOS](https://github.com/coreflake1/NebulaOS), the custom
Linux + Klipper stack for the Creality Ender-3 V3 KE.

**This is not a fork of Klipper.** It contains no copy of Klipper, no patched Klipper core file,
and nothing from `klippy/` outside `extras/`. It is a companion repository to an ordinary,
unmodified checkout of [`Klipper3d/klipper`](https://github.com/Klipper3d/klipper): NebulaOS
installs official Klipper as-is and activates the modules in this repository's `extras/`
directory alongside it. Both checkouts stay content-pristine — `git status --porcelain` is empty
in each of them, always.

If you are looking for the Klipper fork this content used to live in, see
[`coreflake1/NebulaOS-klipper`](https://github.com/coreflake1/NebulaOS-klipper). **That fork is
retired.** It is not the host runtime, not a NebulaOS dependency, and not built or fetched by
anything; NebulaOS runs official upstream Klipper plus this repository. The fork is kept
readable purely as provenance: this repository was seeded from it with `git filter-repo`, so the
development history of every file that moved came with it — `git log` and `git blame` still
work, though commit SHAs are rewritten and the old SHAs remain resolvable only in that original
repository.

## What's in here

[`nebulaos-extensions.json`](nebulaos-extensions.json) is the authoritative inventory — it
declares 34 managed modules, 17 `role: runtime` and 17 `role: test`. This table groups the same
set for readers; if the two ever disagree, the manifest is right.

| Group | Modules |
|---|---|
| Nozzle contact, Z offset, calibration | `z_compensate`, `nozzle_clear`, `nebulaos_z_offset_probe`, `nebulaos_probe_pair`, `nebulaos_calibration`, `nebulaos_calibration_journal` |
| Power-loss recovery | `nebulaos_power_loss_recovery`, `nebulaos_plr_journal` |
| Platform integration | `nebulaos_version`, `nebulaos_compat`, `nebulaos_temperature_mcu` |
| Vendored community modules | `tmcstatus`, `gcode_shell_command`, `virtual_pins`, `calibrate_shaper_config`, `bl24c16f` |
| Offline test infrastructure | `prtouch_test_support` |
| Tests | `extras/test_*.py` (17 modules, `role: test`) |

BLTouch/CR-Touch remains the global Klipper probe — Z homing, bed mesh, and
`probe:z_virtual_endstop` all belong to it. The modules above use the load cell as a secondary
nozzle-contact and calibration reference; none of them registers itself as the global probe.

**On `prtouch_test_support`.** The name is legacy and the module is not a probe. The PRTouch
runtime stack — `prtouch_v2`, `prtouch_probe`, `prtouch_mcu`, `prtouch_nozzle`,
`prtouch_calibration`, `prtouch_units`, `prtouch_safety_guard` — was deleted in Phase 1.8B and
no part of it remains. What survives under that filename is generic offline test scaffolding
(fake config/MCU/toolhead/reactor objects) that seven of the surviving test modules import. It
carries `role: runtime` because `role` is a *composition* attribute, not an "executes on the
printer" one: `runtime` means the file must be present and composed into `klippy/extras/` or
Klippy refuses to start, whereas a deployment may legitimately skip `role: test` modules.
Nothing in `printer.cfg` loads it, and Klipper only imports an extra a config section names.

Every vendored module keeps its original author's copyright and licence header. Where a file
arrived without one, the correct attribution has been researched and added rather than left
blank — see [`VENDORED.md`](VENDORED.md) for the per-file author, licence, upstream source, and
any NebulaOS-side delta. **Nothing community-authored in this repository is presented as
NebulaOS's own work.**

## Layout, and why the tests live next to the modules

```
extras/          every module, including tests
docs/            compatibility contract
nebulaos-extensions.json   machine-readable compatibility manifest
```

The tests are deliberately not in a separate `tests/` directory. Klipper's `klippy/extras/` is a
real Python package named `extras`, and most of these test modules use package-relative imports
(`from . import z_compensate`) to reach the code under test. They only resolve when the test file
sits inside that same package, so they are composed into `klippy/extras/` exactly like the
runtime modules are. `nebulaos-extensions.json` marks them `"role": "test"` so a deployment can
choose not to install them.

Two invocation styles exist, and both are load-bearing:

```sh
# modules using package-relative imports - run from klippy/
python3 -m unittest extras.test_z_compensate_reentrancy_guard -v

# modules using bare imports - run from klippy/extras/
python3 -m unittest test_nebulaos_version -v
```

## How activation works

Klipper's module loader is a filesystem gate, not an import path: `Printer.load_object()`
checks `os.path.exists(<klipper>/klippy/extras/<name>.py)` before it imports anything, so no
`PYTHONPATH`, `.pth` file, or namespace-package trick can supply an extra from elsewhere. But
`os.path.exists()` follows symlinks, and git can be told to ignore untracked paths per-clone.

NebulaOS therefore activates these modules by placing symlinks at
`<klipper>/klippy/extras/<name>.py` pointing into this repository, and listing them in the
Klipper checkout's `.git/info/exclude` — a file that lives inside `.git/`, is never part of any
working tree, and is never cloned or pushed. The result is that both checkouts report a
completely clean `git status`, Moonraker sees an ordinary official Klipper repository with no
anomalies, and the symlinks survive `fetch`, `pull`, `reset --hard`, `checkout`, and
`clean -d -f`.

The symlinks are owned by NebulaOS's platform layer, not by this repository. Creating them is
[`NebulaOS-firmware`](https://github.com/coreflake1/NebulaOS-firmware)'s job, and this repository
is never written to at runtime.

## Compatibility

Klipper offers no API stability guarantee for the host-side interfaces these modules use, so
compatibility is checked rather than assumed. `nebulaos-extensions.json` records the Klipper
commit this extension set was qualified against and the Klipper symbols it depends on, and
`extras/nebulaos_compat.py` verifies both at startup, before Klipper is allowed to control
hardware. A failed check refuses to start, loudly and specifically.

See [`docs/COMPATIBILITY.md`](docs/COMPATIBILITY.md) for the full contract.

## Licence

GPLv3 — see [`LICENSE`](LICENSE). Individual files carry their own copyright headers; those
headers, not this section, are authoritative for the file they appear in.
