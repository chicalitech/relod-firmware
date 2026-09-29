"""Release safety tests run offline; the fake enforces S3 conditional writes."""
import copy
import hashlib
import io
import json
import re
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from release_manifest import BOARD, ENVIRONMENT, HARDWARE_PROFILE, binary_key, release_key
from publish_firmware import publish_firmware
from promote_firmware import promote_firmware
from release_store import ABSENT, channel_key, read_json


class S3Error(Exception):
    def __init__(self, code):
        self.response = {"Error": {"Code": code}}
        super().__init__(code)


class FakeS3:
    def __init__(self):
        self.objects = {}
        self.puts = []
        self.denied = None
        self.race = None
        self.corrupt = None

    def get_object(self, Bucket, Key):
        if Key not in self.objects:
            raise S3Error("NoSuchKey")
        data = self.objects[Key]
        return {"Body": io.BytesIO(data), "ETag": '"' + hashlib.md5(data).hexdigest() + '"'}

    def put_object(self, Bucket, Key, Body, **kwargs):
        self.puts.append((Key, kwargs))
        if self.denied and self.denied in Key:
            raise S3Error("AccessDenied")
        if self.race and "/channels/" in Key:
            self.objects[Key] = self.race
            self.race = None
        if kwargs.get("IfNoneMatch") == "*" and Key in self.objects:
            raise S3Error("PreconditionFailed")
        if "IfMatch" in kwargs:
            if Key not in self.objects or self.get_object(Bucket, Key)["ETag"] != kwargs["IfMatch"]:
                raise S3Error("PreconditionFailed")
        self.objects[Key] = b"corrupted" if self.corrupt == Key else Body
        return {"ETag": self.get_object(Bucket, Key)["ETag"]}


def candidate(version="8.0.1", commit="a" * 40, data=b"verified application"):
    return {"schema_version": 1, "version": version, "source_commit": commit,
            "board": BOARD, "hardware_profile": HARDWARE_PROFILE, "environment": ENVIRONMENT,
            "binary": {"key": binary_key(version, commit), "size": len(data),
                       "sha256": hashlib.sha256(data).hexdigest()}, "release_notes": "Pilot"}, data


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.s3 = FakeS3()
        self.manifest, self.data = candidate()
        self.key = release_key(self.manifest)

    def publish(self, manifest=None, data=None):
        return publish_firmware(self.s3, "bucket", manifest or self.manifest,
                                self.data if data is None else data)

    def promote(self, channel="test", expected=ABSENT, enabled=True, key=None, evidence="USB pilot SHA checked"):
        return promote_firmware(self.s3, "bucket", key or self.key, channel,
                                expected, evidence, enabled=enabled)

    def test_publish_retry_identical_manifest_last_no_channels(self):
        self.publish()
        self.publish()
        self.assertEqual(self.s3.puts[-1][0], self.key)
        self.assertFalse(any("/channels/" in key for key in self.s3.objects))
        self.assertTrue(all(args["IfNoneMatch"] == "*" for _, args in self.s3.puts))
        upload = next(args for key, args in self.s3.puts if key.endswith("firmware.bin"))
        self.assertEqual(upload["Metadata"]["sha256"], self.manifest["binary"]["sha256"])

    def test_conflicting_version_digest_or_commit_rejected(self):
        self.publish()
        for manifest, data in [candidate(data=b"different"), candidate(commit="b" * 40)]:
            with self.assertRaises(ValueError):
                self.publish(manifest, data)

    def test_stored_binary_corruption_prevents_manifest(self):
        self.s3.corrupt = self.manifest["binary"]["key"]
        with self.assertRaises(ValueError):
            self.publish()
        self.assertNotIn(self.key, self.s3.objects)

    def test_bad_local_binary_does_not_reserve_version(self):
        with self.assertRaises(ValueError):
            self.publish(data=b"bad")
        self.assertEqual(self.s3.objects, {})

    def test_incomplete_or_corrupt_release_cannot_promote(self):
        with self.assertRaises(S3Error):
            self.promote()
        self.publish()
        self.s3.objects[self.manifest["binary"]["key"]] = b"bad"
        with self.assertRaises(ValueError):
            self.promote()
        self.assertNotIn(channel_key("test"), self.s3.objects)

    def test_stable_requires_test_evidence_for_exact_candidate(self):
        self.publish()
        with self.assertRaises(ValueError):
            self.promote("stable")
        self.promote()
        result = self.promote("stable")
        self.assertEqual(result["manifest_key"], self.key)
        self.assertIs(result["enabled"], True)

    def test_missing_evidence_and_non_boolean_rejected(self):
        self.publish()
        for kwargs in [{"evidence": " "}, {"enabled": "false"}, {"enabled": 1}]:
            with self.assertRaises(ValueError):
                self.promote(**kwargs)

    def test_pause_keeps_high_watermark_and_can_resume(self):
        self.publish()
        self.promote()
        key = channel_key("test")
        _, etag = read_json(self.s3, "bucket", key)
        paused = self.promote(expected=etag, enabled=False)
        self.assertEqual(paused["version"], "8.0.1")
        self.assertFalse(paused["enabled"])
        old, data = candidate("8.0.0", "b" * 40)
        self.publish(old, data)
        _, etag = read_json(self.s3, "bucket", key)
        with self.assertRaises(ValueError):
            self.promote(expected=etag, key=release_key(old))
        self.assertTrue(self.promote(expected=etag)["enabled"])

    def test_pause_does_not_need_manifest_or_binary(self):
        self.publish()
        current = self.promote()
        key = channel_key("test")
        _, etag = read_json(self.s3, "bucket", key)
        del self.s3.objects[self.key]
        del self.s3.objects[self.manifest["binary"]["key"]]
        paused = self.promote(expected=etag, enabled=False)
        self.assertFalse(paused["enabled"])
        for field in ("manifest_key", "version", "sha256"):
            self.assertEqual(paused[field], current[field])
        _, etag = read_json(self.s3, "bucket", key)
        with self.assertRaises(S3Error):
            self.promote(expected=etag)

    def test_shared_workflow_exclusion_protects_cross_channel_decisions(self):
        workflow = (Path(__file__).resolve().parents[1] /
                    ".github/workflows/promote-firmware.yml").read_text()
        concurrency = re.search(r"(?m)^concurrency:\n((?:[ \t]+[^\n]*\n)+)", workflow).group(1)
        group = re.search(r"(?m)^  group: (.+)$", concurrency).group(1)
        self.assertNotIn("${{", group)  # Same lock for every channel and pause/enable.
        self.assertIn("cancel-in-progress: false", concurrency)
        # With that exclusion, a completed test pause must be visible to stable.
        self.publish()
        self.promote()
        _, etag = read_json(self.s3, "bucket", channel_key("test"))
        self.promote(expected=etag, enabled=False)
        with self.assertRaises(ValueError):
            self.promote("stable")
        self.assertNotIn(channel_key("stable"), self.s3.objects)

    def test_stale_and_racing_update_never_report_success(self):
        self.publish()
        current = self.promote()
        key = channel_key("test")
        with self.assertRaises(ValueError):
            self.promote()
        _, etag = read_json(self.s3, "bucket", key)
        raced = dict(current, evidence="Another operator", promotion_id="b" * 32)
        race_bytes = json.dumps(raced).encode()
        self.s3.race = race_bytes
        with self.assertRaises(S3Error):
            self.promote(expected=etag, enabled=False)
        self.assertEqual(self.s3.objects[key], race_bytes)
        self.assertEqual(len([k for k in self.s3.objects if "/audits/" in k]), 2)

    def test_iam_denial_leaves_existing_pointer(self):
        self.publish()
        self.promote()
        key = channel_key("test")
        before = self.s3.objects[key]
        _, etag = read_json(self.s3, "bucket", key)
        for denied in ["/audits/", "/channels/"]:
            self.s3.denied = denied
            with self.assertRaises(S3Error):
                self.promote(expected=etag, enabled=False)
            self.assertEqual(self.s3.objects[key], before)

    def test_malformed_existing_pointer_fails_closed(self):
        self.publish()
        current = self.promote()
        key = channel_key("test")
        for field, value in [("enabled", "false"), ("schema_version", True), ("version", "oops")]:
            broken = dict(current, **{field: value})
            self.s3.objects[key] = json.dumps(broken).encode()
            _, etag = read_json(self.s3, "bucket", key)
            with self.assertRaises(ValueError):
                self.promote(expected=etag)


if __name__ == "__main__":
    unittest.main()
