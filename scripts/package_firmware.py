"""Package the exact, clean-source production OTA app; never rebuild during packaging."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

from release_manifest import (BOARD, ENVIRONMENT, HARDWARE_PROFILE, MAX_BINARY_SIZE,
                              binary_key, validate_manifest, version_tuple)

MARKER = b"RELOD_RELEASE_IDENTITY_V1|"


def source_identity(repo):
    repo = Path(repo)
    version = (repo / "relod_sparkfun_v8_0/VERSION").read_text(encoding="utf-8").strip()
    version_tuple(version)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("repository does not have a full source commit")
    status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=repo, text=True)
    return version, commit, "dirty" if status else "clean"


def application_identity(data):
    """Inspect the pinned esptool ESP32 image format, including both checksums.

    ESP32-C6 uses a 24-byte header, chip ID 13, 8-byte segment headers,
    XOR checksum at the next 16-byte boundary minus one, and appended SHA-256.
    The first mapped application segment starts with esp_app_desc_t (256 bytes).
    Bootloaders lack this descriptor; merged images fail exact image length.
    """
    if not 24 <= len(data) <= MAX_BINARY_SIZE or data[0] != 0xE9:
        raise ValueError("not an OTA application image or exceeds OTA slot")
    if not 1 <= data[1] <= 16 or struct.unpack_from("<H", data, 12)[0] != 13 or data[23] != 1:
        raise ValueError("invalid ESP32-C6 image header or missing image digest")
    offset, checksum = 24, 0xEF
    segments = []
    for index in range(data[1]):
        if offset + 8 > len(data):
            raise ValueError("truncated image segment header")
        address, size = struct.unpack_from("<II", data, offset)
        offset += 8
        if size % 4 or offset + size > len(data):
            raise ValueError("invalid image segment size")
        segment = data[offset:offset + size]
        if index == 0 and (not 0x42000000 <= address < 0x43000000 or
                           size < 256 or segment[:4] != struct.pack("<I", 0xABCD5432)):
            raise ValueError("image is not an ESP32-C6 application")
        for value in segment:
            checksum ^= value
        segments.append(segment)
        offset += size
    checksum_offset = offset + (15 - offset) % 16
    end = checksum_offset + 1
    if len(data) != end + 32:
        raise ValueError("truncated, merged, padded, or signed image is not the expected OTA application")
    if any(data[offset:checksum_offset]) or data[checksum_offset] != checksum:
        raise ValueError("image checksum mismatch")
    if hashlib.sha256(data[:end]).digest() != data[end:]:
        raise ValueError("image appended SHA-256 mismatch")
    identities = []
    for segment in segments:
        for match in re.finditer(re.escape(MARKER) + rb"([^\x00]+)\x00", segment):
            try:
                identities.append(match.group(1).decode("ascii").split("|"))
            except UnicodeDecodeError as error:
                raise ValueError("invalid release identity encoding") from error
    if len(identities) != 1 or len(identities[0]) != 7:
        raise ValueError("missing or ambiguous embedded release identity")
    return identities[0]


def package_firmware(binary, output, repo, tag=None, prefix="firmware", release_notes=""):
    version, commit, state = source_identity(repo)
    if state != "clean":
        raise ValueError("release packaging requires a clean committed source tree")
    if tag is not None:
        tag_version = tag[1:] if tag.startswith("v") else tag
        version_tuple(tag_version)
        if tag_version != version:
            raise ValueError("release tag and VERSION disagree")
    binary = Path(binary)
    if binary.stat().st_size > MAX_BINARY_SIZE:
        raise ValueError("binary exceeds OTA slot")
    data = binary.read_bytes()
    expected = [version, commit, BOARD, HARDWARE_PROFILE, ENVIRONMENT, "release", "clean"]
    if application_identity(data) != expected:
        raise ValueError("embedded identity differs from clean production source identity")
    manifest = validate_manifest({
        "schema_version": 1, "version": version, "source_commit": commit,
        "board": BOARD, "hardware_profile": HARDWARE_PROFILE, "environment": ENVIRONMENT,
        "binary": {"key": binary_key(version, commit, prefix), "size": len(data),
                   "sha256": hashlib.sha256(data).hexdigest()},
        "release_notes": release_notes,
    }, prefix)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "firmware.bin").write_bytes(data)
    (output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8", newline="\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--prefix", default="firmware")
    parser.add_argument("--tag", help="numeric release tag, optionally prefixed with v")
    parser.add_argument("--release-notes-file", type=Path)
    args = parser.parse_args()
    try:
        notes = args.release_notes_file.read_text(encoding="utf-8") if args.release_notes_file else ""
        manifest = package_firmware(args.binary, args.output, Path(__file__).resolve().parents[1],
                                    tag=args.tag, prefix=args.prefix, release_notes=notes)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Packaging failed: {error}\n")
    print(json.dumps(manifest, sort_keys=True))


if __name__ == "__main__":
    main()
