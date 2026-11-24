"""Tests for bench-firmware-list.py on the shared crate's fixture image. Run: python3 -m unittest discover scripts"""
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("bfl", Path(__file__).with_name("bench-firmware-list.py"))
bfl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bfl)

DESIGNER = bfl.ROOT.parent / "embarch-study-designer"
FIX = DESIGNER / "tests/fixtures/hil-image"


def identity(segs):
    canon = "[" + ",".join(
        '{"address":%d,"length":%d,"sha256":"%s"}' % (s["address"], s["length"], s["sha256"]) for s in segs
    ) + "]"
    return hashlib.sha256(canon.encode()).hexdigest()


@unittest.skipUnless((FIX / "fixture.hex").is_file(), "no embarch-study-designer sibling")
class T(unittest.TestCase):
    def run_script(self, *args):
        out = Path(tempfile.mkdtemp()) / "release" / "bench-firmware.json"
        rc = bfl.main([*args, "--commit", "abc123", "--out", str(out)])
        return rc, out

    def test_fixture_matches_the_crates_expectation(self):
        rc, out = self.run_script("v1", f"dev-bench:nrf:{FIX / 'fixture.hex'}")
        self.assertEqual(rc, 0)
        doc = json.loads(out.read_text())
        self.assertEqual((doc["schema"], doc["release"]), (1, "v1"))
        (img,) = doc["images"]
        self.assertEqual((img["app"], img["board"], img["version"], img["commit"]), ("dev-bench", "nrf", "v1", "abc123"))
        segs = img["segments"]
        self.assertEqual([(s["address"], s["length"]) for s in segs], [(0x20000, 0x3C), (0x20400, 0x1A)])
        self.assertEqual(img["identity"], identity(segs))

    def test_two_images_sorted_and_elf_agrees_with_hex(self):
        rc, out = self.run_script("v1", f"pd-bench:zeta:{FIX / 'fixture.elf'}", f"dev-bench:alpha:{FIX / 'fixture.hex'}")
        self.assertEqual(rc, 0)
        imgs = json.loads(out.read_text())["images"]
        self.assertEqual([(i["app"], i["board"]) for i in imgs], [("dev-bench", "alpha"), ("pd-bench", "zeta")])
        self.assertEqual(imgs[0]["identity"], imgs[1]["identity"])

    def test_bin_base_and_duplicates(self):
        rc, out = self.run_script("v1", f"a:b:{FIX / 'fixture_a.bin'}@0x20000")
        self.assertEqual(rc, 0)
        self.assertEqual(json.loads(out.read_text())["images"][0]["segments"][0]["address"], 0x20000)
        rc, _ = self.run_script("v1", f"a:b:{FIX / 'fixture.hex'}", f"a:b:{FIX / 'fixture.elf'}")
        self.assertEqual(rc, 2)

    def test_bad_spec_is_an_error(self):
        rc, _ = self.run_script("v1", "no-colons")
        self.assertEqual(rc, 2)


if __name__ == "__main__":
    unittest.main()
