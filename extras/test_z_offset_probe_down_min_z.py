# Regression test for the down_min_z sign convention in touch_probe().
#
# The bug: positive down_min_z (a depth below Z=0) was used as an absolute Z
# coordinate, causing probing to move UPWARD.  The fix negates it before
# clamping to stepper position_min.
#
# These tests extract the z_floor expression from the actual module source and
# evaluate it with concrete values, proving the *behavior* passed into
# phoming.probing_move(), not merely the presence of a minus sign.
#
# Run from klippy/: python3 -m unittest extras.test_z_offset_probe_down_min_z -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import re
import unittest


def _module_source():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, 'nebulaos_z_offset_probe.py')
    with open(path) as f:
        return f.read()


def _extract_method(source, method_name):
    lines = source.splitlines()
    in_method = False
    method_lines = []
    method_indent = None
    for line in lines:
        if 'def %s(' % method_name in line:
            in_method = True
            method_indent = len(line) - len(line.lstrip())
            method_lines.append(line)
            continue
        if in_method:
            if line.strip() == '' or line.strip().startswith('#'):
                method_lines.append(line)
                continue
            indent = len(line) - len(line.lstrip())
            if indent <= method_indent and line.strip():
                break
            method_lines.append(line)
    return '\n'.join(method_lines)


def _extract_z_floor_expr(source):
    """Return the RHS of the 'z_floor = ...' assignment in touch_probe."""
    method = _extract_method(source, 'touch_probe')
    for line in method.splitlines():
        stripped = line.strip()
        if stripped.startswith('z_floor'):
            m = re.match(r'z_floor\s*=\s*(.+)', stripped)
            if m:
                return m.group(1)
    raise AssertionError("z_floor assignment not found in touch_probe")


def _eval_z_floor(position_min, down_min_z):
    """Evaluate the actual z_floor expression from source with given values."""
    expr = _extract_z_floor_expr(_module_source())
    env = {'self': type('S', (), {'_z_min_position': position_min})(),
           'down_min_z': down_min_z}
    return eval(expr, {"__builtins__": {'max': max}}, env)


class DownMinZSignConventionTest(unittest.TestCase):
    """Behavioral proof that down_min_z (positive depth) is correctly negated
    to produce a downward Z target coordinate."""

    def test_case_a_depth_exceeds_position_min(self):
        # position_min=-5, down_min_z=10 -> target must be -5 (clamped)
        z_floor = _eval_z_floor(position_min=-5.0, down_min_z=10)
        self.assertAlmostEqual(z_floor, -5.0)

    def test_case_b_depth_within_position_min(self):
        # position_min=-5, down_min_z=2 -> target must be -2
        z_floor = _eval_z_floor(position_min=-5.0, down_min_z=2)
        self.assertAlmostEqual(z_floor, -2.0)

    def test_case_c_depth_equals_position_min(self):
        # position_min=-5, down_min_z=5 -> target must be -5
        z_floor = _eval_z_floor(position_min=-5.0, down_min_z=5)
        self.assertAlmostEqual(z_floor, -5.0)

    def test_z_floor_always_negative_for_positive_depth(self):
        # Any positive down_min_z with any negative position_min -> negative z_floor
        for depth in (1, 2, 5, 10, 50):
            z_floor = _eval_z_floor(position_min=-5.0, down_min_z=depth)
            self.assertLess(z_floor, 0,
                            "z_floor must be negative for down_min_z=%s" % depth)

    def test_z_floor_never_exceeds_position_min(self):
        for depth in (1, 5, 10, 50):
            z_floor = _eval_z_floor(position_min=-5.0, down_min_z=depth)
            self.assertGreaterEqual(z_floor, -5.0,
                                    "z_floor must respect position_min for depth=%s" % depth)


class ProbeDirectionTest(unittest.TestCase):
    """Prove that from the standard hover height (Z=+3), the probing move is
    always downward — z_floor < current_z."""

    def test_standard_probe_descends(self):
        current_z = 3.0
        z_floor = _eval_z_floor(position_min=-5.0, down_min_z=10)
        self.assertLess(z_floor, current_z,
                        "z_floor=%s must be below hover height=%s" %
                        (z_floor, current_z))

    def test_shallow_probe_still_descends(self):
        current_z = 3.0
        z_floor = _eval_z_floor(position_min=-5.0, down_min_z=1)
        self.assertLess(z_floor, current_z,
                        "even min depth z_floor=%s must be below hover=%s" %
                        (z_floor, current_z))


class SourceDocumentationTest(unittest.TestCase):
    """Verify the parameter semantics are documented in source."""

    def test_down_min_z_sign_documented(self):
        source = _module_source()
        method = _extract_method(source, 'touch_probe')
        self.assertTrue(
            'positive depth' in method.lower() or 'depth below' in method.lower(),
            "touch_probe must document that down_min_z is a positive depth")


class RegressionGuardTest(unittest.TestCase):
    """Structural guard: the z_floor expression must contain negation of
    down_min_z.  This is a backup for the behavioral tests above — if the
    expression changes shape, these catch it."""

    def test_z_floor_expr_contains_negation(self):
        expr = _extract_z_floor_expr(_module_source())
        self.assertIn('-down_min_z', expr,
                      "z_floor expression must negate down_min_z")

    def test_z_floor_expr_does_not_use_bare_down_min_z_as_positive(self):
        expr = _extract_z_floor_expr(_module_source())
        # Remove the negated form, check the bare name doesn't appear as positive
        stripped = expr.replace('-down_min_z', '')
        self.assertNotIn('down_min_z', stripped,
                         "z_floor must not use down_min_z without negation")


if __name__ == '__main__':
    unittest.main()
