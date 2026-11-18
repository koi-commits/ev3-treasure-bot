"""All logs below are fabricated samples, never EV3 measurements."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from suggest_config import patch, read, suggestions

SAMPLE_LOG = """type touch: <class 'bool'> gyro: <class 'coroutine'>
drift 30s = 1.5 deg = 3.0 deg/min
sign: OPPOSITE -> GYRO_POSITIVE_DIRECTION = -1
suggested AXLE_TRACK_MM = 104 * -1.1 = -114.4
BLACK_REFLECTION = 3.0
WHITE_REFLECTION = 21.0
==== RESULT: PASS ====
RGB_EDGE_BRIGHT = 18
SIGNATURES = {
'floor': {'mean': (4,5,4), 'tol': (1,1,1)},
'white': {'mean': (30,30,30), 'tol': (3,3,3)},
'finish': {'mean': (60,5,3), 'tol': (3,2,2)}
}
"""

class TestSampleLogs(unittest.TestCase):
    def test_utf16_sample_diff(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"fabricated-sample.txt"
            p.write_text(SAMPLE_LOG,encoding="utf-16")
            err=io.StringIO()
            with contextlib.redirect_stderr(err):
                c=suggestions(read(p))
            self.assertEqual(c["AXLE_TRACK_MM"],114.4)
            self.assertEqual(c["GYRO_POSITIVE_DIRECTION"],-1)
            self.assertIn("RED_RULE",c)
            for text in ("awaitable: yes","awaitable: no","FAIL","threshold: 12.00"):
                self.assertIn(text,err.getvalue())
            self.assertIn("+AXLE_TRACK_MM = 114.4",patch("AXLE_TRACK_MM = 104\n",c))

    def test_failed_marker_sample(self):
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertNotIn("SIGNATURES",suggestions(SAMPLE_LOG.replace("RESULT: PASS","RESULT: FAIL")))
