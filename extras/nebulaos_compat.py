# nebulaos_compat - startup compatibility preflight for the NebulaOS Klipper extension set
#
# Copyright (C) 2026  NebulaOS contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# ---------------------------------------------------------------------------------------
# Why this exists
# ---------------------------------------------------------------------------------------
#
# NebulaOS runs official, unmodified Klipper and activates the modules in this repository
# alongside it. That buys a great deal - no fork, a clean git tree, Moonraker's own updater
# working as designed - at the cost of one real risk: the two halves can drift apart.
#
# Klipper publishes no API version and offers no stability guarantee for the host-side
# interfaces extension code uses. Drift has already happened once, and it was not theoretical:
# mainline commit c89393cda renamed MCU.register_response() to MCU.register_serial_response(),
# which would have broken PRTouch at load time. Nothing in Klipper, Moonraker, or Mainsail can
# detect that, because none of them know this extension set exists.
#
# So this module is the last gate before Klipper is allowed to drive hardware. It runs as an
# ordinary config section, and it either passes or it stops Klippy.
#
# For a printer, "started up but the probe subtly does not work" is strictly worse than "did
# not start, and said exactly why". Everything here is therefore FAIL-CLOSED: any failed check
# raises config_error with a message naming the specific thing that failed, the value found,
# the value expected, and what to do about it. There is no degraded mode and no warn-and-
# continue path, because a partially-validated load-cell probe still drives a nozzle into a
# bed.
#
# ---------------------------------------------------------------------------------------
# What is checked here, and what is deliberately left to the platform
# ---------------------------------------------------------------------------------------
#
# Checked here, because only extension code can:
#   1. The installed Klipper commit matches the qualified one (or an explicitly allowed range).
#   2. The manifest itself is present, parseable, and of a schema version this code understands.
#   3. Every module the manifest declares has a real source file present in this repository.
#   4. Every Klipper symbol this extension set depends on actually exists on the installed
#      Klipper - and no symbol it must NOT depend on has come back.
#   5. Every sensor_type the manifest declares is registered before any [temperature_sensor]
#      section can ask for it.
#
# NOT checked here, deliberately, and left with a declared interface for the platform layer
# (NebulaOS-firmware) to implement:
#   * Composition integrity - that every composed destination path is a real symlink resolving
#     inside this repository, and not an upstream regular file that silently replaced one.
#     This is a property of the Klipper checkout, which the platform composes and owns; by the
#     time Klippy is running, a collision has already shadowed the module and this process is
#     importing upstream's file, not ours. The manifest's "composition" block is the contract
#     for that check; see COMPOSITION_CONTRACT below and docs/COMPATIBILITY.md.
#   * The c_helper.so mtime invariant - that the shipped prebuilt library is newer than every
#     chelper source, so mainline never attempts a gcc rebuild on a device with no toolchain.
#     Deciding this needs firmware build-time information that does not exist inside this
#     repository. The manifest's "chelper" block declares the requirement and names an optional
#     platform_result_file the platform can write its verdict into; see CHELPER_CONTRACT.
#
# Both are Stage 2 work. They are declared rather than stubbed so that the interface is fixed
# now and the platform can be built against it without renegotiating the manifest.

import json
import logging
import os
import subprocess

# The manifest schema versions this module knows how to read. A manifest declaring anything
# else is refused rather than interpreted optimistically - a newer schema may mean a check
# this build does not know it should be performing.
SUPPORTED_SCHEMA_VERSIONS = (1,)

MANIFEST_FILENAME = 'nebulaos-extensions.json'

REQUIRED_MANIFEST_KEYS = (
    'compat_schema_version', 'extensions_version', 'klipper',
    'required_klipper_symbols', 'modules', 'composition',
)

# Interface notes for the platform layer. Kept here, next to the code that reads the manifest,
# so the contract and its consumer cannot drift apart in separate documents.
COMPOSITION_CONTRACT = """
Platform (NebulaOS-firmware) composition contract, from the manifest's "composition" block:

  source_dir       directory inside this repo holding the modules ("extras")
  destination_dir  directory inside the Klipper checkout they are activated in
                   ("klippy/extras")
  exclude_file     per-clone git ignore list to append managed names to
                   (".git/info/exclude" - inside .git/, never part of any working tree)
  link_type        "symlink"
  marker_file      name of the generation marker recording which extensions commit the
                   current link set reflects (".nebulaos-composed"; itself excluded)
  require_symlink_resolving_inside_source
                   when true, the platform MUST verify, after composition and after any
                   Klipper update, that every destination path is a symlink whose resolved
                   target is inside source_dir - and refuse to activate otherwise.

That last check is the collision guard, and it is mandatory. If upstream Klipper ever ships a
file at a path this repo also manages, git overwrites the symlink with upstream's regular file
silently, exit code 0, no warning. The extension is then shadowed with no error anywhere. The
names most exposed are the vendored community ones - gcode_shell_command and virtual_pins are
exactly the kind of module mainline could adopt.
"""

CHELPER_CONTRACT = """
Platform c_helper.so contract, from the manifest's "chelper" block:

Klipper's klippy/chelper/__init__.py decides whether to rebuild its C library by comparing
MTIMES, not hashes: if the prebuilt target is newer than every source, it returns early and
never invokes gcc. NebulaOS ships a cross-compiled c_helper.so and the device has no
toolchain, so a rebuild attempt is not a slow path - it raises, and Klippy does not start.

  enforced_by          "platform" - the firmware build and the boot-time activation step
  requirement          "prebuilt_so_mtime_newer_than_all_chelper_sources"
  target               path of the prebuilt library, relative to the Klipper checkout
  source_dir           directory whose sources must all be older than the target
  platform_result_file optional path the platform may write its verdict to. When set, this
                       module reads it and refuses to start if the verdict is not a pass, so
                       the failure surfaces as a precise preflight error instead of a gcc
                       crash mid-boot. When null (the default today), this module performs no
                       chelper check at all and the platform owns it end to end.
"""


class CompatibilityError(Exception):
    """Raised only by the pure-python helpers below, so they are testable without a printer.
    load_config() converts it to a Klipper config_error."""


def _repo_root():
    """This repository's root, derived from this file's own location.

    Deliberately os.path.realpath'd: at runtime this file is reached through a symlink placed
    in the Klipper checkout's klippy/extras/, and the manifest lives next to the REAL file in
    the extensions repo, not next to the symlink.
    """
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def load_manifest(path):
    if not os.path.exists(path):
        raise CompatibilityError(
            "compatibility manifest not found at %s. Every NebulaOS Klipper extension "
            "install must carry one; its absence means this is a partial or hand-assembled "
            "deployment, which is exactly the state this check exists to refuse." % (path,))
    try:
        with open(path, 'r') as f:
            manifest = json.load(f)
    except (IOError, OSError) as e:
        raise CompatibilityError(
            "compatibility manifest at %s could not be read: %s" % (path, e))
    except ValueError as e:
        raise CompatibilityError(
            "compatibility manifest at %s is not valid JSON: %s" % (path, e))
    if not isinstance(manifest, dict):
        raise CompatibilityError(
            "compatibility manifest at %s must be a JSON object" % (path,))
    return manifest


def check_manifest_shape(manifest):
    # Missing-key check first, deliberately. A manifest with no compat_schema_version at all
    # is malformed, not "declaring schema None" - reporting the absent key by name is far more
    # actionable than a schema-mismatch message about a value that was never there.
    missing = [k for k in REQUIRED_MANIFEST_KEYS if k not in manifest]
    if missing:
        raise CompatibilityError(
            "compatibility manifest is missing required key(s): %s"
            % (', '.join(sorted(missing)),))
    schema = manifest.get('compat_schema_version')
    if schema not in SUPPORTED_SCHEMA_VERSIONS:
        raise CompatibilityError(
            "compatibility manifest declares schema version %r, but this build of "
            "nebulaos_compat understands only %s. Refusing rather than guessing: a newer "
            "schema may require a check this build does not know it should perform."
            % (schema, ', '.join(str(v) for v in SUPPORTED_SCHEMA_VERSIONS)))


def check_modules_present(manifest, repo_root, include_tests=False):
    """Every module the manifest declares must have a real source file in this repository.

    A missing module means the manifest and the tree disagree, which means composition would
    silently produce a broken link or skip a module entirely - including, potentially, a
    safety-relevant one.
    """
    missing = []
    for entry in manifest.get('modules', ()):
        if not isinstance(entry, dict) or 'path' not in entry:
            raise CompatibilityError(
                "compatibility manifest has a malformed modules entry: %r" % (entry,))
        if entry.get('role') == 'test' and not include_tests:
            continue
        full = os.path.join(repo_root, entry['path'])
        if not os.path.isfile(full):
            missing.append(entry['path'])
    if missing:
        raise CompatibilityError(
            "compatibility manifest declares %d module(s) with no source file present in the "
            "extensions repository at %s: %s. The manifest and the installed tree disagree, "
            "so composition cannot be trusted."
            % (len(missing), repo_root, ', '.join(sorted(missing))))


def _resolve_symbol(spec):
    """Resolve "module:Attr.attr" against the Klipper actually loaded in this process.

    Import errors and missing attributes are both reported as 'absent' rather than raised, so
    the caller can accumulate every problem into one message instead of surfacing them one
    restart at a time.
    """
    if ':' not in spec:
        raise CompatibilityError(
            "malformed required_klipper_symbols entry %r - expected 'module:Attr.attr'"
            % (spec,))
    module_name, dotted = spec.split(':', 1)
    try:
        obj = __import__(module_name, globals(), locals(), ['__name__'])
    except ImportError:
        return None, "module '%s' could not be imported" % (module_name,)
    for part in dotted.split('.'):
        if not hasattr(obj, part):
            return None, "'%s' has no attribute '%s'" % (module_name, dotted)
        obj = getattr(obj, part)
    return obj, None


def check_required_symbols(manifest):
    """Probe the installed Klipper for every API this extension set depends on.

    This is the single highest-value check in the file: it is the only one that would have
    caught, automatically and before any motion, the one real breaking change this migration
    found (register_response -> register_serial_response). A ~20-line introspection loop turns
    a mid-homing failure into a refuse-to-start with a precise message.
    """
    problems = []
    for spec in manifest.get('required_klipper_symbols', ()):
        _obj, why = _resolve_symbol(spec)
        if why is not None:
            problems.append("  required but absent: %s (%s)" % (spec, why))
    for spec in manifest.get('forbidden_klipper_symbols', ()):
        obj, why = _resolve_symbol(spec)
        if why is None and obj is not None:
            problems.append(
                "  present but must not be relied on: %s (this build migrated off it; its "
                "reappearance means the installed Klipper is older than the qualified one, "
                "or is not official Klipper at all)" % (spec,))
    if problems:
        raise CompatibilityError(
            "the installed Klipper does not provide the API this NebulaOS extension set was "
            "built against:\n%s\n"
            "This is API drift, not a configuration mistake. Install the qualified Klipper "
            "commit (see the manifest's klipper.qualified_commit), or update the extension "
            "set to one qualified against this Klipper." % ('\n'.join(problems),))


def _git(repo_dir, *args):
    try:
        out = subprocess.check_output(
            ['git', '-C', repo_dir] + list(args),
            stderr=subprocess.STDOUT, timeout=10)
        return out.decode('utf-8', 'replace').strip()
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        logging.info("nebulaos_compat: git %s failed: %s", ' '.join(args), e)
        return None


def _is_ancestor(repo_dir, maybe_ancestor, descendant):
    try:
        subprocess.check_call(
            ['git', '-C', repo_dir, 'merge-base', '--is-ancestor',
             maybe_ancestor, descendant],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        return True
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False


def check_klipper_commit(manifest, klipper_dir, git_runner=_git,
                         ancestry_check=_is_ancestor):
    """Verify the installed Klipper commit against the manifest.

    Accepts the qualified commit outright. If the manifest declares a min/max range, ancestry
    is decided with real `git merge-base --is-ancestor` calls - never a string comparison,
    because git SHAs carry no ordering whatsoever and comparing them as strings would produce
    a check that looks like it works and does not.
    """
    spec = manifest.get('klipper') or {}
    qualified = spec.get('qualified_commit')
    if not qualified:
        raise CompatibilityError(
            "compatibility manifest declares no klipper.qualified_commit")

    installed = git_runner(klipper_dir, 'rev-parse', 'HEAD')
    if installed is None:
        if spec.get('allow_unqualified'):
            logging.warning(
                "nebulaos_compat: could not determine the installed Klipper commit at %s, "
                "and klipper.allow_unqualified is set - continuing unverified", klipper_dir)
            return None
        raise CompatibilityError(
            "could not determine the installed Klipper commit: `git -C %s rev-parse HEAD` "
            "failed. NebulaOS runs Klipper from a real git checkout precisely so this is "
            "answerable; a non-git or damaged checkout cannot be qualified, so it is refused."
            % (klipper_dir,))

    if installed == qualified:
        return installed

    min_commit = spec.get('min_commit')
    max_commit = spec.get('max_commit')
    if min_commit or max_commit:
        in_range = True
        if min_commit and not ancestry_check(klipper_dir, min_commit, installed):
            in_range = False
        if max_commit and not ancestry_check(klipper_dir, installed, max_commit):
            in_range = False
        if in_range:
            logging.warning(
                "nebulaos_compat: installed Klipper %s is not the qualified commit %s, but "
                "is within the manifest's declared compatible range - continuing",
                installed, qualified)
            return installed

    if spec.get('allow_unqualified'):
        logging.warning(
            "nebulaos_compat: installed Klipper %s is not qualified (expected %s), but "
            "klipper.allow_unqualified is set - continuing", installed, qualified)
        return installed

    raise CompatibilityError(
        "installed Klipper is not the commit this NebulaOS extension set was qualified "
        "against.\n"
        "  installed:  %s\n"
        "  qualified:  %s\n"
        "  checkout:   %s\n"
        "Refusing to start. This extension set drives a load-cell probe into the bed; running "
        "it against an unqualified Klipper is a physical-safety change, not a version "
        "mismatch. Either check out the qualified commit, or - if this is deliberate "
        "development - set klipper.allow_unqualified in %s."
        % (installed, qualified, klipper_dir, MANIFEST_FILENAME))


def check_chelper_verdict(manifest, klipper_dir):
    """Consume the platform's chelper verdict, if it published one.

    The mtime invariant itself is the platform's to enforce (see CHELPER_CONTRACT) - it needs
    firmware build-time information this repository does not have. All this does is refuse to
    start when the platform has already said the invariant is violated, so the failure appears
    as a named preflight error rather than a gcc crash part-way through boot.
    """
    spec = manifest.get('chelper') or {}
    result_file = spec.get('platform_result_file')
    if not result_file:
        return None
    path = result_file if os.path.isabs(result_file) \
        else os.path.join(klipper_dir, result_file)
    if not os.path.exists(path):
        raise CompatibilityError(
            "the manifest declares chelper.platform_result_file %s, but the platform has not "
            "written it. That file is the platform's proof that the prebuilt c_helper.so is "
            "newer than every chelper source; without it, Klipper may attempt a gcc rebuild "
            "on a device with no toolchain and fail mid-boot." % (path,))
    try:
        with open(path, 'r') as f:
            verdict = json.load(f)
    except (IOError, OSError, ValueError) as e:
        raise CompatibilityError(
            "chelper verdict file %s could not be read as JSON: %s" % (path, e))
    if verdict.get('status') != 'ok':
        raise CompatibilityError(
            "the platform reported the chelper prebuilt-library invariant as NOT satisfied: "
            "%r. Klipper would try to rebuild c_helper.so with gcc, which this device has no "
            "toolchain for." % (verdict,))
    return verdict


def check_sensor_types(manifest, printer, config):
    """Guarantee every sensor_type the manifest declares is registered with [heaters].

    This is what makes NebulaOS's own sensor types independent of printer.cfg section order.
    heaters.setup_sensor() resolves sensor_type against a plain dict, so a [temperature_sensor]
    section loaded before the providing module would get Klipper's bare "Unknown temperature
    sensor". Forcing the providing module to load here - from a section the compatibility
    contract already places first - removes that ordering dependency, and the verification
    afterwards turns any remaining gap into a message that names the sensor type and the module
    that should have provided it.
    """
    declared = manifest.get('sensor_types') or ()
    if not declared:
        return []
    registered = []
    pheaters = printer.load_object(config, 'heaters')
    for entry in declared:
        sensor_type = entry.get('sensor_type')
        provider_path = entry.get('provided_by', '')
        module_name = os.path.splitext(os.path.basename(provider_path))[0]
        if module_name:
            printer.load_object(config, module_name)
        if sensor_type not in getattr(pheaters, 'sensor_factories', {}):
            raise CompatibilityError(
                "sensor type '%s' is declared in the compatibility manifest as provided by "
                "%s, but loading that module did not register it with [heaters]. Any "
                "[temperature_sensor] section using this sensor type would fail with "
                "Klipper's own 'Unknown temperature sensor' error."
                % (sensor_type, provider_path or '(unspecified)'))
        registered.append(sensor_type)
    return registered


def run_preflight(manifest_path, klipper_dir, printer=None, config=None,
                  include_tests=False):
    """Run every check this module owns, in dependency order, and return a status dict.

    Order matters: the manifest has to be readable before anything can be checked against it,
    and its shape has to be understood before its contents are trusted.
    """
    manifest = load_manifest(manifest_path)
    check_manifest_shape(manifest)
    repo_root = os.path.dirname(os.path.abspath(manifest_path))
    check_modules_present(manifest, repo_root, include_tests=include_tests)
    check_required_symbols(manifest)
    installed = check_klipper_commit(manifest, klipper_dir)
    check_chelper_verdict(manifest, klipper_dir)
    sensor_types = []
    if printer is not None:
        sensor_types = check_sensor_types(manifest, printer, config)
    return {
        'extensions_version': manifest.get('extensions_version'),
        'nebulaos_api_level': manifest.get('nebulaos_api_level'),
        'compat_schema_version': manifest.get('compat_schema_version'),
        'qualified_klipper_commit': (manifest.get('klipper') or {}).get('qualified_commit'),
        'installed_klipper_commit': installed,
        'managed_module_count': len(manifest.get('modules', ())),
        'registered_sensor_types': sensor_types,
        'composition': manifest.get('composition'),
        'chelper': manifest.get('chelper'),
        'status': 'ok',
    }


class NebulaOSCompat:
    def __init__(self, config):
        self.printer = config.get_printer()
        repo_root = _repo_root()
        self.manifest_path = config.get(
            'manifest', os.path.join(repo_root, MANIFEST_FILENAME))
        # The Klipper checkout root, from this module's runtime location. Note the difference
        # from _repo_root(): this one must NOT be realpath'd - the checkout of interest is the
        # tree Klippy is running out of, reached through the symlink, not the extensions repo
        # the symlink points into.
        self.klipper_dir = config.get(
            'klipper_path',
            os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', '..')))
        try:
            self.status = run_preflight(self.manifest_path, self.klipper_dir,
                                        printer=self.printer, config=config)
        except CompatibilityError as e:
            raise self.printer.config_error("nebulaos_compat: %s" % (e,))
        logging.info(
            "nebulaos_compat: PASS - extensions %s (api level %s), %d managed modules, "
            "Klipper %s",
            self.status['extensions_version'], self.status['nebulaos_api_level'],
            self.status['managed_module_count'],
            self.status['installed_klipper_commit'] or 'unverified')

    def get_status(self, eventtime=None):
        return dict(self.status)


def load_config(config):
    return NebulaOSCompat(config)
