"""Keep CPython mocks from hiding unsupported MicroPython math imports."""
import ast
from pathlib import Path
import unittest

ALLOWED = set("sqrt sin cos tan asin acos atan atan2 exp log pow fabs floor ceil trunc fmod copysign degrees radians pi e inf nan isinf isnan isfinite modf frexp ldexp".split())


class MathImports(unittest.TestCase):
    def test_src_math_whitelist(self):
        for path in (Path(__file__).resolve().parents[1] / "src").glob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8-sig"))
            aliases = {}
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module in ("math", "umath"):
                    for name in node.names:
                        self.assertIn(name.name, ALLOWED, "%s:%d" % (path.name, node.lineno))
                if isinstance(node, ast.Import):
                    for name in node.names:
                        if name.name in ("math", "umath"):
                            aliases[name.asname or name.name] = name.name
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in aliases:
                    self.assertIn(node.attr, ALLOWED, "%s:%d" % (path.name, node.lineno))
