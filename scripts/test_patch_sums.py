"""Tests for patch-sums.py on synthetic patches.yml files. Run: python3 -m unittest discover scripts"""
import hashlib
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("patch_sums", Path(__file__).with_name("patch-sums.py"))
ps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps)

YML = """# header comment
patches:
  - path: zephyr/local/a.patch
    sha256sum: {a}
    module: zephyr
    comments: |
      path: not-a-key
      sha256sum: {a}
  # between entries
  - path: zephyr/upstream/b.patch
    sha256sum: '{b}'   # trailing
    module: zephyr
"""


class T(unittest.TestCase):
    def setUp(self):
        self.d = Path(tempfile.mkdtemp())
        z = self.d / "zephyr"
        (z / "patches/zephyr/local").mkdir(parents=True)
        (z / "patches/zephyr/upstream").mkdir(parents=True)
        (z / "patches/zephyr/local/a.patch").write_bytes(b"one\r\ntwo\r\n")
        (z / "patches/zephyr/upstream/b.patch").write_bytes(b"bee\n")
        self.a = hashlib.sha256(b"one\ntwo\n").hexdigest()  # CRLF hashes as LF
        self.b = hashlib.sha256(b"bee\n").hexdigest()
        self.yml = z / "patches.yml"

    def write(self, a, b):
        self.yml.write_text(YML.format(a=a, b=b), newline="\n")

    def test_clean(self):
        self.write(self.a, self.b)
        self.assertEqual(ps.main(["--check", str(self.yml)]), 0)

    def test_check_lists_stale(self):
        self.write("0" * 64, self.b)
        self.assertEqual(ps.main(["--check", str(self.yml)]), 1)
        _, stale = ps.scan(self.yml)
        self.assertEqual([e["path"] for e, _ in stale], ["zephyr/local/a.patch"])

    def test_update_touches_only_sums(self):
        self.write("0" * 64, "f" * 64)
        before = self.yml.read_bytes()
        self.assertEqual(ps.main(["--update", str(self.yml)]), 0)
        after = self.yml.read_bytes()
        # the sums quoted inside the comments block are text, not fields: untouched
        expect = before.decode().replace("    sha256sum: " + "0" * 64, "    sha256sum: " + self.a, 1)
        expect = expect.replace("f" * 64, self.b).encode()
        self.assertEqual(after, expect)
        self.assertNotEqual(before, after)
        self.assertEqual(ps.main(["--check", str(self.yml)]), 0)

    def test_update_noop_leaves_bytes(self):
        self.write(self.a, self.b)
        before = self.yml.read_bytes()
        ps.main(["--update", str(self.yml)])
        self.assertEqual(self.yml.read_bytes(), before)

    def test_missing_file_is_error(self):
        self.write(self.a, self.b)
        (self.d / "zephyr/patches/zephyr/upstream/b.patch").unlink()
        with self.assertRaises(SystemExit):
            ps.scan(self.yml)


if __name__ == "__main__":
    unittest.main()
