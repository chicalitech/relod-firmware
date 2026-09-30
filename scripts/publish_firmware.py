"""Publish one packaged candidate immutably; this never changes a channel."""
import argparse
import json
from pathlib import Path

from release_manifest import BOARD, HARDWARE_PROFILE, release_key, validate_manifest
from release_store import immutable_put, json_bytes, read_bytes, verify_binary


def publish_firmware(s3, bucket, manifest, binary, prefix="firmware"):
    validate_manifest(manifest, prefix)
    verify_binary(manifest, binary)
    claim_key = f"{prefix}/versions/{BOARD}/{HARDWARE_PROFILE}/{manifest['version']}.json"
    claim = {"version": manifest["version"], "source_commit": manifest["source_commit"],
             "binary": manifest["binary"]}
    immutable_put(s3, bucket, claim_key, json_bytes(claim), "application/json")
    binary_key = manifest["binary"]["key"]
    immutable_put(s3, bucket, binary_key, binary, "application/octet-stream",
                  {"sha256": manifest["binary"]["sha256"]})
    stored, _ = read_bytes(s3, bucket, binary_key)
    verify_binary(manifest, stored)
    key = release_key(manifest, prefix)
    immutable_put(s3, bucket, key, json_bytes(manifest), "application/json")
    return key


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bucket", required=True)
    parser.add_argument("--prefix", default="firmware")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--binary", required=True, type=Path)
    args = parser.parse_args()
    import boto3
    try:
        key = publish_firmware(boto3.client("s3"), args.bucket,
                               json.loads(args.manifest.read_text(encoding="utf-8")),
                               args.binary.read_bytes(), args.prefix)
    except Exception as error:
        parser.exit(1, f"Publication failed: {error}\n")
    print(key)


if __name__ == "__main__":
    main()
