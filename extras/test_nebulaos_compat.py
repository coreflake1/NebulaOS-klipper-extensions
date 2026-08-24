# nebulaos_compat tests - the startup compatibility gate.
#
# The property under test is fail-closed behaviour, so most of these are negative tests: each
# one breaks exactly one thing and asserts that Klippy would refuse to start, with a message
# that names the specific problem rather than a generic failure. A compatibility gate that
# passes when it should fail is worse than no gate at all, because it manufactures confidence.
#
# The real repository's own manifest is exercised too, not just synthetic ones - a manifest
# that drifts out of step with the tree (a module renamed, a test file added and not declared)
# is exactly the failure this file must catch in CI rather than on a printer.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_compat -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import os
import shutil
import tempfile
import unittest

from . import nebulaos_compat


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(
    nebulaos_compat.__file__)))
REAL_MANIFEST = os.path.join(REPO_ROOT, nebulaos_compat.MANIFEST_FILENAME)

QUALIFIED = '58bd67db3ce1be1951c3e4a6d1156a79903d4edc'


def _load_real_manifest():
    with open(REAL_MANIFEST, 'r') as f:
        return json.load(f)


def _fixed_git(sha):
    """A git_runner stand-in that reports one commit, so commit checks are testable without
    a real Klipper checkout."""
    def runner(repo_dir, *args):
        if args[:1] == ('rev-parse',):
            return sha
        return None
    return runner


def _never_ancestor(repo_dir, a, b):
    return False


def _always_ancestor(repo_dir, a, b):
    return True


class _FakeHeaters:
    def __init__(self):
        self.sensor_factories = {}

    def add_sensor_factory(self, sensor_type, factory):
        self.sensor_factories[sensor_type] = factory


class _FakePrinter:
    """Models Printer.load_object()'s two relevant properties: idempotence, and that loading a
    module is what causes its registration side effects to happen."""

    def __init__(self, providers=None):
        self.heaters = _FakeHeaters()
        self.loaded = []
        self.providers = providers or {}

    def load_object(self, config, name):
        self.loaded.append(name)
        if name == 'heaters':
            return self.heaters
        provider = self.providers.get(name)
        if provider is not None:
            provider(self.heaters)
        return object()


class ManifestLoadingTest(unittest.TestCase):
    def test_missing_manifest_is_refused(self):
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.load_manifest('/nonexistent/nebulaos-extensions.json')
        self.assertIn('not found', str(ctx.exception))

    def test_malformed_json_is_refused_with_the_path_named(self):
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            f.write('{not json at all')
            path = f.name
        try:
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.load_manifest(path)
            self.assertIn(path, str(ctx.exception))
        finally:
            os.unlink(path)

    def test_non_object_manifest_is_refused(self):
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            f.write('[1, 2, 3]')
            path = f.name
        try:
            with self.assertRaises(nebulaos_compat.CompatibilityError):
                nebulaos_compat.load_manifest(path)
        finally:
            os.unlink(path)


class ManifestShapeTest(unittest.TestCase):
    def test_the_real_manifest_has_a_supported_schema_and_every_required_key(self):
        nebulaos_compat.check_manifest_shape(_load_real_manifest())

    def test_unknown_schema_version_is_refused_not_guessed_at(self):
        manifest = _load_real_manifest()
        manifest['compat_schema_version'] = 99
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_manifest_shape(manifest)
        self.assertIn('99', str(ctx.exception))

    def test_each_required_key_is_individually_required(self):
        for key in nebulaos_compat.REQUIRED_MANIFEST_KEYS:
            manifest = _load_real_manifest()
            manifest.pop(key, None)
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_manifest_shape(manifest)
            self.assertIn(key, str(ctx.exception))


class ManagedModulePresenceTest(unittest.TestCase):
    def test_the_real_manifest_matches_the_real_tree(self):
        manifest = _load_real_manifest()
        nebulaos_compat.check_modules_present(manifest, REPO_ROOT, include_tests=True)

    def test_the_manifest_declares_every_python_module_in_the_tree(self):
        # The other direction: a module added to extras/ but never declared would never be
        # composed onto a device, and nothing else would notice.
        declared = {e['path'] for e in _load_real_manifest()['modules']}
        on_disk = {
            'extras/' + name
            for name in os.listdir(os.path.join(REPO_ROOT, 'extras'))
            if name.endswith('.py')
        }
        self.assertEqual(on_disk - declared, set(),
                         "module(s) present in extras/ but absent from the manifest")

    def test_a_declared_module_with_no_source_file_is_refused(self):
        manifest = _load_real_manifest()
        manifest['modules'].append({'path': 'extras/not_a_real_module.py',
                                    'role': 'runtime'})
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_modules_present(manifest, REPO_ROOT)
        self.assertIn('not_a_real_module.py', str(ctx.exception))

    def test_a_malformed_modules_entry_is_refused(self):
        manifest = _load_real_manifest()
        manifest['modules'].append('extras/oops_a_bare_string.py')
        with self.assertRaises(nebulaos_compat.CompatibilityError):
            nebulaos_compat.check_modules_present(manifest, REPO_ROOT)

    def test_test_modules_are_skipped_unless_asked_for(self):
        # A deployment may legitimately choose not to install the test modules; a runtime
        # module going missing is never legitimate.
        manifest = _load_real_manifest()
        manifest['modules'].append({'path': 'extras/test_absent_on_device.py',
                                    'role': 'test'})
        nebulaos_compat.check_modules_present(manifest, REPO_ROOT, include_tests=False)
        with self.assertRaises(nebulaos_compat.CompatibilityError):
            nebulaos_compat.check_modules_present(manifest, REPO_ROOT, include_tests=True)


class RequiredSymbolTest(unittest.TestCase):
    def test_every_symbol_the_real_manifest_requires_exists_on_this_klipper(self):
        # Run against the composed tree's REAL Klipper. This is the check that would have
        # caught register_response -> register_serial_response automatically.
        nebulaos_compat.check_required_symbols(_load_real_manifest())

    def test_a_missing_required_api_is_refused_and_named(self):
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = [
            'mcu:MCU.register_serial_response',
            'mcu:MCU.a_method_that_does_not_exist',
        ]
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_required_symbols(manifest)
        msg = str(ctx.exception)
        self.assertIn('a_method_that_does_not_exist', msg)
        self.assertNotIn('register_serial_response', msg)

    def test_the_actual_historical_breakage_would_have_been_caught(self):
        # The real scenario: old extensions (requiring register_response) on new Klipper
        # (58bd67db, which only has register_serial_response). The gate catches the drift.
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = ['mcu:MCU.register_response']
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_required_symbols(manifest)
        self.assertIn('register_response', str(ctx.exception))

    def test_a_forbidden_symbol_that_reappears_is_refused(self):
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = []
        manifest['forbidden_klipper_symbols'] = ['mcu:MCU.register_serial_response']
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_required_symbols(manifest)
        self.assertIn('must not be relied on', str(ctx.exception))

    def test_an_unimportable_module_is_reported_not_raised(self):
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = ['not_a_real_module:Thing.attr']
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_required_symbols(manifest)
        self.assertIn('could not be imported', str(ctx.exception))

    def test_a_malformed_symbol_spec_is_refused(self):
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = ['mcu.MCU.register_response']
        with self.assertRaises(nebulaos_compat.CompatibilityError):
            nebulaos_compat.check_required_symbols(manifest)

    def test_all_problems_are_reported_together_not_one_restart_at_a_time(self):
        manifest = _load_real_manifest()
        manifest['required_klipper_symbols'] = [
            'mcu:MCU.nope_one', 'mcu:MCU.nope_two', 'mcu:MCU.nope_three',
        ]
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_required_symbols(manifest)
        msg = str(ctx.exception)
        for name in ('nope_one', 'nope_two', 'nope_three'):
            self.assertIn(name, msg)


class KlipperCommitTest(unittest.TestCase):
    def test_the_qualified_commit_is_accepted(self):
        manifest = _load_real_manifest()
        got = nebulaos_compat.check_klipper_commit(
            manifest, '/irrelevant', git_runner=_fixed_git(QUALIFIED))
        self.assertEqual(got, QUALIFIED)

    def test_an_unqualified_commit_is_refused_with_both_shas_named(self):
        manifest = _load_real_manifest()
        other = '0' * 40
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_klipper_commit(
                manifest, '/irrelevant', git_runner=_fixed_git(other),
                ancestry_check=_never_ancestor)
        msg = str(ctx.exception)
        self.assertIn(other, msg)
        self.assertIn(QUALIFIED, msg)

    def test_a_commit_inside_a_declared_range_is_accepted_by_real_ancestry(self):
        manifest = _load_real_manifest()
        manifest['klipper']['min_commit'] = 'a' * 40
        manifest['klipper']['max_commit'] = 'b' * 40
        got = nebulaos_compat.check_klipper_commit(
            manifest, '/irrelevant', git_runner=_fixed_git('c' * 40),
            ancestry_check=_always_ancestor)
        self.assertEqual(got, 'c' * 40)

    def test_a_commit_outside_a_declared_range_is_refused(self):
        manifest = _load_real_manifest()
        manifest['klipper']['min_commit'] = 'a' * 40
        with self.assertRaises(nebulaos_compat.CompatibilityError):
            nebulaos_compat.check_klipper_commit(
                manifest, '/irrelevant', git_runner=_fixed_git('c' * 40),
                ancestry_check=_never_ancestor)

    def test_range_membership_never_falls_back_to_string_comparison(self):
        # Git SHAs carry no ordering. A range check that compared them as strings would look
        # like it worked. Here the installed SHA sorts between min and max lexically, but real
        # ancestry says no - and the answer must be no.
        manifest = _load_real_manifest()
        manifest['klipper']['min_commit'] = 'a' * 40
        manifest['klipper']['max_commit'] = 'z' * 40
        with self.assertRaises(nebulaos_compat.CompatibilityError):
            nebulaos_compat.check_klipper_commit(
                manifest, '/irrelevant', git_runner=_fixed_git('m' * 40),
                ancestry_check=_never_ancestor)

    def test_an_undeterminable_commit_is_refused_by_default(self):
        manifest = _load_real_manifest()
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_klipper_commit(
                manifest, '/irrelevant', git_runner=lambda *a, **k: None)
        self.assertIn('rev-parse', str(ctx.exception))

    def test_allow_unqualified_is_an_explicit_opt_in_only(self):
        manifest = _load_real_manifest()
        self.assertFalse(manifest['klipper']['allow_unqualified'],
                         "the shipped manifest must default to refusing an unqualified "
                         "Klipper")
        manifest['klipper']['allow_unqualified'] = True
        got = nebulaos_compat.check_klipper_commit(
            manifest, '/irrelevant', git_runner=_fixed_git('0' * 40),
            ancestry_check=_never_ancestor)
        self.assertEqual(got, '0' * 40)


class ChelperVerdictTest(unittest.TestCase):
    """The mtime invariant is the platform's to enforce; this consumes its published verdict."""

    def test_no_result_file_declared_means_no_check(self):
        # Setting platform_result_file to null hands the invariant to the platform end to
        # end. The shipped manifest no longer does that (Stage 2 wired it to a real path -
        # see the test below), but the escape hatch stays supported and tested, because a
        # deployment whose platform enforces the invariant without publishing a verdict is
        # a legitimate configuration rather than a broken one.
        manifest = _load_real_manifest()
        manifest['chelper']['platform_result_file'] = None
        self.assertIsNone(
            nebulaos_compat.check_chelper_verdict(manifest, '/irrelevant'))

    def test_the_real_manifest_now_names_the_platform_verdict_file(self):
        # Stage 2 wired this to a real path. It stays relative to the Klipper checkout so it
        # travels with the tree the platform composed - a migration that replaces
        # apps/klipper takes the stale verdict with it instead of leaving one behind that
        # describes a tree that no longer exists.
        manifest = _load_real_manifest()
        self.assertEqual(manifest['chelper']['platform_result_file'],
                         '.nebulaos-chelper-verdict.json')
        self.assertFalse(os.path.isabs(manifest['chelper']['platform_result_file']))

    def test_a_declared_but_unwritten_verdict_is_refused(self):
        manifest = _load_real_manifest()
        manifest['chelper']['platform_result_file'] = '/nonexistent/chelper-verdict.json'
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_chelper_verdict(manifest, '/irrelevant')
        self.assertIn('has not written it', str(ctx.exception))

    def test_a_passing_verdict_is_accepted(self):
        tmp = tempfile.mkdtemp()
        try:
            path = os.path.join(tmp, 'chelper-verdict.json')
            with open(path, 'w') as f:
                json.dump({'status': 'ok'}, f)
            manifest = _load_real_manifest()
            manifest['chelper']['platform_result_file'] = path
            self.assertEqual(
                nebulaos_compat.check_chelper_verdict(manifest, '/irrelevant'),
                {'status': 'ok'})
        finally:
            shutil.rmtree(tmp)

    def test_a_failing_verdict_refuses_to_start(self):
        tmp = tempfile.mkdtemp()
        try:
            path = os.path.join(tmp, 'chelper-verdict.json')
            with open(path, 'w') as f:
                json.dump({'status': 'stale', 'newer_source': 'stepcompress.c'}, f)
            manifest = _load_real_manifest()
            manifest['chelper']['platform_result_file'] = path
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_chelper_verdict(manifest, '/irrelevant')
            self.assertIn('gcc', str(ctx.exception))
        finally:
            shutil.rmtree(tmp)


class CompositionIntegrityTest(unittest.TestCase):
    """The collision guard, seen from inside the running process.

    The failure these cover is the one that produces no error anywhere: git replacing a
    managed symlink with an upstream regular file at exit code 0. Every case below asserts
    a refusal, because a composition check that passes on a shadowed module manufactures
    exactly the confidence it exists to withhold.
    """

    def _compose(self, tmp, roles=None, link=True):
        """Build a throwaway (extensions repo, Klipper checkout) pair and compose them the
        way NebulaOS-firmware does. Returns (manifest, klipper_dir, repo_root)."""
        repo_root = os.path.join(tmp, 'ext')
        klipper = os.path.join(tmp, 'klipper')
        os.makedirs(os.path.join(repo_root, 'extras'))
        os.makedirs(os.path.join(klipper, 'klippy', 'extras'))
        modules = roles or [('extras/alpha.py', 'runtime'),
                            ('extras/beta.py', 'runtime'),
                            ('extras/test_gamma.py', 'test')]
        for path, _role in modules:
            with open(os.path.join(repo_root, path), 'w') as f:
                f.write('# %s\n' % (path,))
            if link:
                os.symlink(os.path.join(repo_root, path),
                           os.path.join(klipper, 'klippy', 'extras',
                                        os.path.basename(path)))
        manifest = {
            'composition': {
                'source_dir': 'extras',
                'destination_dir': 'klippy/extras',
                'require_symlink_resolving_inside_source': True,
            },
            'modules': [{'path': p, 'role': r} for p, r in modules],
        }
        return manifest, klipper, repo_root

    def test_a_correctly_composed_tree_passes(self):
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            # Only the two runtime modules are required; the test module may legitimately
            # be skipped by a deployment.
            self.assertEqual(
                nebulaos_compat.check_composition_integrity(manifest, klipper, root), 2)
        finally:
            shutil.rmtree(tmp)

    def test_an_upstream_file_shadowing_a_module_is_refused(self):
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            dest = os.path.join(klipper, 'klippy', 'extras', 'alpha.py')
            os.unlink(dest)
            with open(dest, 'w') as f:
                f.write('# upstream Klipper now ships this\n')
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_composition_integrity(manifest, klipper, root)
            msg = str(ctx.exception)
            self.assertIn('SHADOWED', msg)
            self.assertIn('alpha.py', msg)
        finally:
            shutil.rmtree(tmp)

    def test_a_link_escaping_the_repository_is_refused(self):
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            outsider = os.path.join(tmp, 'outsider.py')
            with open(outsider, 'w') as f:
                f.write('# not ours\n')
            dest = os.path.join(klipper, 'klippy', 'extras', 'beta.py')
            os.unlink(dest)
            os.symlink(outsider, dest)
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_composition_integrity(manifest, klipper, root)
            self.assertIn('outside this repository', str(ctx.exception))
        finally:
            shutil.rmtree(tmp)

    def test_a_dangling_link_is_refused(self):
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            os.unlink(os.path.join(root, 'extras', 'alpha.py'))
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_composition_integrity(manifest, klipper, root)
            self.assertIn('dangling', str(ctx.exception))
        finally:
            shutil.rmtree(tmp)

    def test_a_missing_runtime_module_is_refused(self):
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            os.unlink(os.path.join(klipper, 'klippy', 'extras', 'beta.py'))
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_composition_integrity(manifest, klipper, root)
            self.assertIn('composition is incomplete', str(ctx.exception))
        finally:
            shutil.rmtree(tmp)

    def test_a_missing_test_module_is_tolerated(self):
        # A deployment may legitimately skip test modules; only a missing runtime module is
        # never legitimate.
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            os.unlink(os.path.join(klipper, 'klippy', 'extras', 'test_gamma.py'))
            self.assertEqual(
                nebulaos_compat.check_composition_integrity(manifest, klipper, root), 2)
        finally:
            shutil.rmtree(tmp)

    def test_every_problem_is_reported_at_once(self):
        # One restart per problem is a miserable way to fix a broken deployment.
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp)
            os.unlink(os.path.join(klipper, 'klippy', 'extras', 'alpha.py'))
            os.unlink(os.path.join(klipper, 'klippy', 'extras', 'beta.py'))
            with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
                nebulaos_compat.check_composition_integrity(manifest, klipper, root)
            msg = str(ctx.exception)
            self.assertIn('alpha.py', msg)
            self.assertIn('beta.py', msg)
        finally:
            shutil.rmtree(tmp)

    def test_the_check_is_skipped_when_the_manifest_does_not_require_it(self):
        # A copy-based deployment, or the immutable factory-fallback tree where the modules
        # are deliberately real files, uses this same module and must not be told its own
        # correct layout is broken.
        tmp = tempfile.mkdtemp()
        try:
            manifest, klipper, root = self._compose(tmp, link=False)
            manifest['composition']['require_symlink_resolving_inside_source'] = False
            self.assertIsNone(
                nebulaos_compat.check_composition_integrity(manifest, klipper, root))
        finally:
            shutil.rmtree(tmp)

    def test_the_real_manifest_requires_the_guard(self):
        manifest = _load_real_manifest()
        self.assertTrue(
            manifest['composition']['require_symlink_resolving_inside_source'],
            "the shipped manifest must demand the collision guard - the vendored community "
            "module names are exactly the ones mainline could adopt")


class SensorTypeRegistrationTest(unittest.TestCase):
    def test_declared_sensor_types_are_registered_by_loading_their_provider(self):
        manifest = _load_real_manifest()
        printer = _FakePrinter(providers={
            'nebulaos_temperature_mcu':
                lambda h: h.add_sensor_factory('nebulaos_temperature_mcu', object()),
        })
        got = nebulaos_compat.check_sensor_types(manifest, printer, None)
        self.assertEqual(got, ['nebulaos_temperature_mcu'])
        self.assertIn('nebulaos_temperature_mcu', printer.loaded)

    def test_a_provider_that_fails_to_register_is_refused_and_named(self):
        manifest = _load_real_manifest()
        printer = _FakePrinter()  # loading the provider registers nothing
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            nebulaos_compat.check_sensor_types(manifest, printer, None)
        msg = str(ctx.exception)
        self.assertIn('nebulaos_temperature_mcu', msg)
        self.assertIn('Unknown temperature sensor', msg)


class CompositionContractTest(unittest.TestCase):
    """Stage 2 builds the runtime composition check against this block. These assertions fix
    the interface now so it cannot drift before there is a consumer."""

    def test_the_composition_block_declares_everything_the_platform_needs(self):
        composition = _load_real_manifest()['composition']
        for key in ('source_dir', 'destination_dir', 'exclude_file', 'link_type',
                    'marker_file', 'require_symlink_resolving_inside_source'):
            self.assertIn(key, composition)

    def test_the_collision_guard_is_demanded_not_optional(self):
        composition = _load_real_manifest()['composition']
        self.assertTrue(composition['require_symlink_resolving_inside_source'],
                        "an upstream file colliding with a managed path replaces the symlink "
                        "silently; the platform must be told to verify, not invited to")

    def test_activation_is_by_symlink_as_the_architecture_requires(self):
        self.assertEqual(_load_real_manifest()['composition']['link_type'], 'symlink')

    def test_the_exclude_file_is_inside_dot_git_so_it_never_dirties_the_tree(self):
        exclude = _load_real_manifest()['composition']['exclude_file']
        self.assertTrue(exclude.startswith('.git/'), exclude)


class EndToEndPreflightTest(unittest.TestCase):
    def _run(self, mutate=None, printer=None, compose=True):
        manifest = _load_real_manifest()
        if mutate is not None:
            mutate(manifest)
        tmp = tempfile.mkdtemp()
        try:
            # The manifest must sit at the repo root for the module-presence check to resolve
            # relative paths, so link the real extras/ next to a copy of it.
            os.symlink(os.path.join(REPO_ROOT, 'extras'), os.path.join(tmp, 'extras'))
            path = os.path.join(tmp, nebulaos_compat.MANIFEST_FILENAME)
            with open(path, 'w') as f:
                json.dump(manifest, f)

            # A stand-in Klipper checkout, composed the way NebulaOS-firmware composes a
            # real one. Stage 2 added two checks that are properties of the CHECKOUT rather
            # than of this repository - composition integrity and the platform's chelper
            # verdict - so an end-to-end preflight can no longer be run against a path that
            # does not exist. Building a real composed tree here keeps this test genuinely
            # end to end instead of quietly excusing the two newest checks from it.
            klipper = os.path.join(tmp, 'klipper')
            dest = os.path.join(klipper, 'klippy', 'extras')
            os.makedirs(dest)
            if compose:
                for entry in manifest.get('modules', ()):
                    src = os.path.join(REPO_ROOT, entry['path'])
                    if os.path.isfile(src):
                        os.symlink(src, os.path.join(dest, os.path.basename(entry['path'])))
                verdict = manifest.get('chelper', {}).get('platform_result_file')
                if verdict:
                    with open(os.path.join(klipper, verdict), 'w') as f:
                        json.dump({'status': 'ok'}, f)
            return nebulaos_compat.run_preflight(
                path, klipper, printer=printer, config=None,
                include_tests=True)
        finally:
            shutil.rmtree(tmp)

    def _printer(self):
        return _FakePrinter(providers={
            'nebulaos_temperature_mcu':
                lambda h: h.add_sensor_factory('nebulaos_temperature_mcu', object()),
        })

    def test_a_valid_manifest_passes_and_reports_what_it_verified(self):
        def allow(manifest):
            # The commit check needs a real checkout; exercise it separately (KlipperCommitTest)
            # and let this end-to-end run past it.
            manifest['klipper']['allow_unqualified'] = True
        status = self._run(mutate=allow, printer=self._printer())
        self.assertEqual(status['status'], 'ok')
        self.assertEqual(status['qualified_klipper_commit'], QUALIFIED)
        self.assertEqual(status['registered_sensor_types'], ['nebulaos_temperature_mcu'])
        self.assertGreater(status['managed_module_count'], 25)
        # Every runtime module was verified as a link resolving inside this repository.
        runtime = [e for e in _load_real_manifest()['modules']
                   if e.get('role') != 'test']
        self.assertEqual(status['verified_composed_modules'], len(runtime))

    def test_an_uncomposed_checkout_fails_the_whole_preflight(self):
        # The end-to-end shape of the collision guard: a Klipper checkout where the managed
        # modules are simply not there at all.
        def allow(manifest):
            manifest['klipper']['allow_unqualified'] = True
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            self._run(mutate=allow, printer=self._printer(), compose=False)
        self.assertIn('composition is incomplete', str(ctx.exception))

    def test_a_missing_required_api_fails_the_whole_preflight(self):
        def break_symbols(manifest):
            manifest['klipper']['allow_unqualified'] = True
            manifest['required_klipper_symbols'].append('mcu:MCU.gone_in_a_future_klipper')
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            self._run(mutate=break_symbols, printer=self._printer())
        self.assertIn('gone_in_a_future_klipper', str(ctx.exception))

    def test_a_missing_managed_module_source_fails_the_whole_preflight(self):
        def break_modules(manifest):
            manifest['klipper']['allow_unqualified'] = True
            manifest['modules'].append({'path': 'extras/vanished.py', 'role': 'runtime'})
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            self._run(mutate=break_modules, printer=self._printer())
        self.assertIn('vanished.py', str(ctx.exception))

    def test_an_incompatible_klipper_sha_fails_the_whole_preflight(self):
        # allow_unqualified left at its shipped default of false, and the temp directory is
        # not a git checkout at all - so the commit check cannot resolve a SHA and must refuse.
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            self._run(printer=self._printer())
        self.assertIn('Klipper', str(ctx.exception))

    def test_checks_run_in_dependency_order(self):
        # A manifest that is broken in several ways must report the earliest failure, not a
        # confusing downstream one - shape before contents.
        def break_everything(manifest):
            manifest['compat_schema_version'] = 42
            manifest['modules'].append({'path': 'extras/vanished.py', 'role': 'runtime'})
        with self.assertRaises(nebulaos_compat.CompatibilityError) as ctx:
            self._run(mutate=break_everything, printer=self._printer())
        self.assertIn('schema version', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
