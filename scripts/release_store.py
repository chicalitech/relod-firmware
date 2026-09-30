"""Small S3 primitives shared by publication and promotion (no AWS import at load)."""
import hashlib
import json
from datetime import datetime
from uuid import UUID

from release_manifest import BOARD, HARDWARE_PROFILE, release_key, validate_prefix, version_tuple

ABSENT = "absent"
CHANNELS = ("test", "stable", "legacy")


def json_bytes(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def error_code(error):
    return getattr(error, "response", {}).get("Error", {}).get("Code")


def read_bytes(s3, bucket, key):
    response = s3.get_object(Bucket=bucket, Key=key)
    body = response["Body"]
    try:
        return body.read(), response["ETag"]
    finally:
        body.close()


def read_json(s3, bucket, key):
    data, etag = read_bytes(s3, bucket, key)
    return json.loads(data), etag


def immutable_put(s3, bucket, key, data, content_type, metadata=None):
    try:
        s3.put_object(Bucket=bucket, Key=key, Body=data, ContentType=content_type,
                      IfNoneMatch="*", ServerSideEncryption="AES256", Metadata=metadata or {})
    except Exception as error:
        if error_code(error) != "PreconditionFailed":
            raise
        stored, _ = read_bytes(s3, bucket, key)
        if stored != data:
            raise ValueError(f"immutable object already has different contents: {key}") from error


def verify_binary(manifest, data):
    binary = manifest["binary"]
    if len(data) != binary["size"] or hashlib.sha256(data).hexdigest() != binary["sha256"]:
        raise ValueError("binary bytes do not match manifest size and SHA-256")


def channel_key(channel, prefix="firmware"):
    validate_prefix(prefix)
    if channel not in CHANNELS:
        raise ValueError("unsupported channel")
    return f"{prefix}/channels/{BOARD}/{HARDWARE_PROFILE}/{channel}.json"


def validate_pointer(pointer, prefix="firmware"):
    fields = {"schema_version", "manifest_key", "enabled", "version", "sha256",
              "promotion_id", "evidence", "promoted_at"}
    if not isinstance(pointer, dict) or set(pointer) != fields:
        raise ValueError("invalid channel pointer fields")
    if type(pointer["schema_version"]) is not int or pointer["schema_version"] != 1:
        raise ValueError("invalid pointer schema")
    if type(pointer["enabled"]) is not bool:
        raise ValueError("enabled must be boolean")
    version_tuple(pointer["version"])
    import re
    base = f"{validate_prefix(prefix)}/releases/{BOARD}/{HARDWARE_PROFILE}/{pointer['version']}/"
    if not isinstance(pointer["manifest_key"], str) or not re.fullmatch(re.escape(base) + r"[0-9a-f]{40}/manifest\.json", pointer["manifest_key"]):
        raise ValueError("invalid pointer manifest key")
    if not isinstance(pointer["sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", pointer["sha256"]):
        raise ValueError("invalid pointer SHA-256")
    if not isinstance(pointer["evidence"], str) or not pointer["evidence"].strip() or len(pointer["evidence"]) > 1024:
        raise ValueError("hardware evidence is required")
    if not isinstance(pointer["promotion_id"], str):
        raise ValueError("invalid promotion ID")
    UUID(pointer["promotion_id"])
    if not isinstance(pointer["promoted_at"], str) or not pointer["promoted_at"].endswith("Z"):
        raise ValueError("promotion timestamp must be ISO UTC")
    datetime.fromisoformat(pointer["promoted_at"].replace("Z", "+00:00"))
    return pointer


def load_pointer(s3, bucket, key, prefix):
    try:
        pointer, etag = read_json(s3, bucket, key)
    except Exception as error:
        if error_code(error) == "NoSuchKey":
            return None, ABSENT
        raise
    return validate_pointer(pointer, prefix), etag


def load_release(s3, bucket, manifest_key, prefix):
    # Reject arbitrary S3 reads before fetching the manifest.
    import re
    base = f"{validate_prefix(prefix)}/releases/{BOARD}/{HARDWARE_PROFILE}/"
    if not isinstance(manifest_key, str) or not re.fullmatch(re.escape(base) + r"[0-9]+\.[0-9]+\.[0-9]+/[0-9a-f]{40}/manifest\.json", manifest_key):
        raise ValueError("invalid release manifest key")
    manifest, _ = read_json(s3, bucket, manifest_key)
    if release_key(manifest, prefix) != manifest_key:
        raise ValueError("manifest is not at its canonical immutable key")
    data, _ = read_bytes(s3, bucket, manifest["binary"]["key"])
    verify_binary(manifest, data)
    return manifest
