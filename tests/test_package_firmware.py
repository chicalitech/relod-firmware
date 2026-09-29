"""Release packaging contract, using structurally valid ESP32-C6 image fixtures."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import package_firmware as package
from release_manifest import release_key, validate_manifest, version_tuple

COMMIT = "a" * 40


def app_image(**changes):
    identity = dict(version="8.0.0", source_commit=COMMIT,
                    board="sparkfun_esp32c6_thing_plus", hardware_profile="relod-6514-v1",
                    environment="sparkfun_c6", build_type="release", source_state="clean")
    identity.update(changes)
    marker = ("RELOD_RELEASE_IDENTITY_V1|" + "|".join(identity.values()) + "\0").encode()
    # esp_app_desc_t begins the first flash segment (magic 0xabcd5432).
    payload = struct.pack("<I", 0xABCD5432) + bytes(252) + marker
    payload += bytes(-len(payload) % 4)
    header = bytearray(24)
    header[0:2] = bytes([0xE9, 1])
    struct.pack_into("<H", header, 12, 13)
    header[23] = 1
    image = header + struct.pack("<II", 0x42000020, len(payload)) + payload
    checksum = 0xEF
    for value in payload:
        checksum ^= value
    image += bytes((15 - len(image)) % 16) + bytes([checksum])
    return bytes(image + hashlib.sha256(image).digest())


class PackagingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.binary = self.root / "firmware.bin"
        self.binary.write_bytes(app_image())
        self.identity = patch.object(package, "source_identity", return_value=("8.0.0", COMMIT, "clean"))
        self.identity.start()
        self.addCleanup(self.identity.stop)

    def run_package(self, **kwargs):
        return package.package_firmware(self.binary, self.root / "output", self.root,
                                        tag="v8.0.0", **kwargs)

    def test_packages_exact_bytes_with_reproducible_manifest(self):
        manifest = self.run_package(release_notes="USB candidate")
        self.assertEqual(validate_manifest(manifest), manifest)
        self.assertEqual(manifest["binary"]["size"], self.binary.stat().st_size)
        self.assertEqual(manifest["binary"]["sha256"], hashlib.sha256(self.binary.read_bytes()).hexdigest())
        self.assertEqual((self.root / "output/firmware.bin").read_bytes(), self.binary.read_bytes())
        original = (self.root / "output/manifest.json").read_bytes()
        self.run_package(release_notes="USB candidate")
        self.assertEqual((self.root / "output/manifest.json").read_bytes(), original)
        self.assertEqual(json.loads(original), manifest)
        self.assertTrue(release_key(manifest).endswith("/manifest.json"))

    def test_rejects_nonproduction_or_stale_identity(self):
        for changes in ({"environment": "sparkfun_c6_debug"},
                        {"environment": "sparkfun_c6_soldered_test"},
                        {"build_type": "debug"}, {"board": "seeed_xiao_esp32c6"},
                        {"hardware_profile": "other"}, {"source_state": "dirty"},
                        {"source_commit": "b" * 40}, {"version": "8.0.1"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.binary.write_bytes(app_image(**changes))
                self.run_package()

    def test_rejects_wrong_tag(self):
        for tag in ("8.0.1", "v8.0.0-rc1", "vv8.0.0", "08.0.0", "8.0"):
            with self.subTest(tag=tag), self.assertRaises(ValueError):
                package.package_firmware(self.binary, self.root / "output", self.root, tag=tag)

    def test_rejects_dirty_checkout(self):
        with patch.object(package, "source_identity", return_value=("8.0.0", COMMIT, "dirty")):
            with self.assertRaises(ValueError):
                self.run_package()

    def test_missing_binary(self):
        self.binary.unlink()
        with self.assertRaises((ValueError, FileNotFoundError)):
            self.run_package()

    def test_rejects_oversized_and_nonapplication_images(self):
        valid = app_image()
        bad_checksum = bytearray(valid)
        bad_checksum[-33] ^= 1
        bad_chip = bytearray(valid)
        bad_chip[12] = 5
        no_app_descriptor = bytearray(valid)
        no_app_descriptor[32:36] = bytes(4)
        for data in (b"x" * (0x640000 + 1), b"", bytes(4096), valid + bytes(4096),
                     bytes(65536) + valid, bytes(bad_checksum), bytes(bad_chip),
                     bytes(no_app_descriptor), valid[:-1], valid[:-32] + bytes(32)):
            with self.subTest(size=len(data)), self.assertRaises(ValueError):
                self.binary.write_bytes(data)
                self.run_package()

    def test_custom_prefix(self):
        manifest = self.run_package(prefix="test/ota")
        self.assertTrue(manifest["binary"]["key"].startswith("test/ota/releases/"))
        validate_manifest(manifest, prefix="test/ota")

    def test_invalid_manifest_contract(self):
        manifest = self.run_package()
        mutations = [("schema_version", True), ("schema_version", 2), ("version", "8.0"),
                     ("source_commit", COMMIT.upper()), ("board", "xiao"),
                     ("hardware_profile", "other"), ("environment", "sparkfun_c6_debug"),
                     ("release_notes", None)]
        for key, value in mutations:
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_manifest(dict(manifest, **{key: value}))
        for key, value in (("size", True), ("size", 0), ("size", 0x640001),
                           ("sha256", "A" * 64), ("key", "firmware/../firmware.bin")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_manifest(dict(manifest, binary=dict(manifest["binary"], **{key: value})))
        for prefix in ("../firmware", "/firmware", "firmware/", "firmware//ota", "https://s3", "a\\b"):
            with self.subTest(prefix=prefix), self.assertRaises(ValueError):
                validate_manifest(manifest, prefix=prefix)

    def test_strict_numeric_version(self):
        self.assertEqual(version_tuple("4294967295.0.1"), (4294967295, 0, 1))
        for value in ("8.0", "8.0.0-rc1", "v8.0.0", "08.0.0", "8.0.0\n", "-1.0.0", "4294967296.0.0", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                version_tuple(value)


if __name__ == "__main__":
    unittest.main()
