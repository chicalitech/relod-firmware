#pragma once

#include "release_identity_generated.h"

namespace relod {
// Printed during startup so the linker must retain this exact identity in the app.
constexpr char kReleaseIdentity[] = RELOD_RELEASE_MARKER;
}  // namespace relod
