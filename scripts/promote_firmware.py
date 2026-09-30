"""Verify stored bytes, record intent, and conditionally update one release pointer."""
import argparse
import json
from datetime import datetime, timezone
from uuid import uuid4

from release_manifest import BOARD, HARDWARE_PROFILE, version_tuple
from release_store import (ABSENT, channel_key, immutable_put, json_bytes, load_pointer,
                           load_release, validate_pointer)


def promote_firmware(s3, bucket, manifest_key, channel, expected_etag, evidence,
                     enabled=True, prefix="firmware"):
    if type(enabled) is not bool:
        raise ValueError("enabled must be boolean")
    if not isinstance(evidence, str) or not evidence.strip():
        raise ValueError("hardware evidence is required")
    if not isinstance(expected_etag, str) or not expected_etag:
        raise ValueError("explicit expected ETag or 'absent' is required")
    key = channel_key(channel, prefix)
    current, etag = load_pointer(s3, bucket, key, prefix)
    if expected_etag != etag:
        raise ValueError("channel changed; inspect it and provide its current ETag")
    if enabled:
        manifest = load_release(s3, bucket, manifest_key, prefix)
        version, digest = manifest["version"], manifest["binary"]["sha256"]
        if current:
            old, new = version_tuple(current["version"]), version_tuple(version)
            if new < old:
                raise ValueError("cannot lower the channel version high watermark")
            if new == old and (current["manifest_key"] != manifest_key or current["sha256"] != digest):
                raise ValueError("cannot replace a version with a different artifact")
    else:
        if not current or current["manifest_key"] != manifest_key:
            raise ValueError("pause must retain the current manifest and version high watermark")
        # Emergency disablement must work even when artifact reads fail.
        version, digest = current["version"], current["sha256"]
    tested = None
    if channel == "stable" and enabled:
        tested, _ = load_pointer(s3, bucket, channel_key("test", prefix), prefix)
        if not tested or not tested["enabled"] or tested["manifest_key"] != manifest_key or tested["sha256"] != digest:
            raise ValueError("stable requires the exact candidate enabled in test with hardware evidence")
    pointer = validate_pointer({"schema_version": 1, "manifest_key": manifest_key,
                                "enabled": enabled, "version": version,
                                "sha256": digest, "promotion_id": str(uuid4()),
                                "evidence": evidence.strip(),
                                "promoted_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")}, prefix)
    audit = {"schema_version": 1, "operation": "pointer_update_intent", "channel": channel,
             "expected_etag": expected_etag, "previous": current, "proposed": pointer,
             "tested_pointer": tested}
    audit_key = f"{prefix}/audits/{BOARD}/{HARDWARE_PROFILE}/{channel}/{pointer['promotion_id']}.json"
    immutable_put(s3, bucket, audit_key, json_bytes(audit), "application/json")
    condition = {"IfNoneMatch": "*"} if expected_etag == ABSENT else {"IfMatch": expected_etag}
    s3.put_object(Bucket=bucket, Key=key, Body=json_bytes(pointer), ContentType="application/json",
                  ServerSideEncryption="AES256", **condition)
    return pointer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bucket", required=True)
    parser.add_argument("--prefix", default="firmware")
    parser.add_argument("--manifest-key", required=True)
    # Legacy requires a separately reviewed manual operation, never this CLI/workflow.
    parser.add_argument("--channel", required=True, choices=("test", "stable"))
    parser.add_argument("--expected-etag", required=True, help="current quoted S3 ETag, or absent")
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--pause", action="store_true")
    args = parser.parse_args()
    import boto3
    try:
        pointer = promote_firmware(boto3.client("s3"), args.bucket, args.manifest_key,
                                   args.channel, args.expected_etag, args.evidence,
                                   enabled=not args.pause, prefix=args.prefix)
    except Exception as error:
        parser.exit(1, f"Promotion did not complete: {error}\n")
    print(json.dumps(pointer, sort_keys=True))


if __name__ == "__main__":
    main()
