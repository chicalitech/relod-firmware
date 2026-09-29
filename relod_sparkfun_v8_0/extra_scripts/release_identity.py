"""Generate build identity in ignored build output; VERSION is the only version source."""
import json
from pathlib import Path
import sys

Import("env")

sys.dont_write_bytecode = True
project = Path(env.subst("$PROJECT_DIR"))
sys.path.insert(0, str(project.parent / "scripts"))
from package_firmware import source_identity
from release_manifest import HARDWARE_PROFILE

version, commit, state = source_identity(project.parent)
board = env.GetProjectOption("board")
environment = env.subst("$PIOENV")
build_type = env.GetProjectOption("build_type", "release")
marker = "|".join(["RELOD_RELEASE_IDENTITY_V1", version, commit, board,
                   HARDWARE_PROFILE, environment, build_type, state])
generated = Path(env.subst("$BUILD_DIR")) / "generated"
generated.mkdir(parents=True, exist_ok=True)
header = generated / "release_identity_generated.h"
content = "#pragma once\n" + "\n".join(
    f"#define {name} {json.dumps(value)}" for name, value in (
        ("RELOD_FIRMWARE_VERSION", version), ("RELOD_SOURCE_COMMIT", commit),
        ("RELOD_HARDWARE_PROFILE", HARDWARE_PROFILE), ("RELOD_BUILD_ENVIRONMENT", environment),
        ("RELOD_RELEASE_MARKER", marker))) + "\n"
if not header.exists() or header.read_text(encoding="utf-8") != content:
    header.write_text(content, encoding="utf-8", newline="\n")
env.Append(CPPPATH=[str(generated)])
