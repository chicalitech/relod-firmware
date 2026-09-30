"""Shared, dependency-free validation for immutable production release manifests."""
import re

BOARD = "sparkfun_esp32c6_thing_plus"
HARDWARE_PROFILE = "relod-6514-v1"
ENVIRONMENT = "sparkfun_c6"
MAX_BINARY_SIZE = 0x640000


def version_tuple(version):
    if not isinstance(version, str) or not re.fullmatch(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)", version):
        raise ValueError("version must be three canonical unsigned decimal components")
    parts = tuple(int(part) for part in version.split("."))
    if any(part > 0xFFFFFFFF for part in parts):
        raise ValueError("version component exceeds uint32")
    return parts


def validate_prefix(prefix):
    if not isinstance(prefix, str) or not re.fullmatch(r"[A-Za-z0-9_-]+(?:/[A-Za-z0-9_-]+)*", prefix):
        raise ValueError("prefix must be a safe relative slash-separated path")
    return prefix


def binary_key(version, source_commit, prefix="firmware"):
    validate_prefix(prefix)
    version_tuple(version)
    if not isinstance(source_commit, str) or not re.fullmatch(r"[0-9a-f]{40}", source_commit):
        raise ValueError("source_commit must be a full lowercase Git SHA")
    return f"{prefix}/releases/{BOARD}/{HARDWARE_PROFILE}/{version}/{source_commit}/firmware.bin"


def validate_manifest(data, prefix="firmware"):
    validate_prefix(prefix)
    fields = {"schema_version", "version", "source_commit", "board", "hardware_profile", "environment", "binary", "release_notes"}
    if not isinstance(data, dict) or set(data) != fields:
        raise ValueError("manifest fields do not match schema 1")
    if type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise ValueError("unsupported manifest schema")
    expected_key = binary_key(data["version"], data["source_commit"], prefix)
    if (data["board"], data["hardware_profile"], data["environment"]) != (BOARD, HARDWARE_PROFILE, ENVIRONMENT):
        raise ValueError("manifest is not a production SparkFun release")
    binary = data["binary"]
    if not isinstance(binary, dict) or set(binary) != {"key", "size", "sha256"}:
        raise ValueError("invalid binary descriptor")
    if binary["key"] != expected_key:
        raise ValueError("binary key does not match immutable release identity")
    if type(binary["size"]) is not int or not 0 < binary["size"] <= MAX_BINARY_SIZE:
        raise ValueError("binary size does not fit the production OTA slot")
    if not isinstance(binary["sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", binary["sha256"]):
        raise ValueError("invalid binary SHA-256")
    if not isinstance(data["release_notes"], str) or len(data["release_notes"]) > 1024:
        raise ValueError("release_notes must be text of at most 1024 characters")
    return data


def release_key(manifest, prefix="firmware"):
    validate_manifest(manifest, prefix)
    return manifest["binary"]["key"].removesuffix("firmware.bin") + "manifest.json"
