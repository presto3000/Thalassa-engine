#pragma once

#include <filesystem>

#include "thalassa/sim/replay_recording.hpp"

// Disk persistence for replays. Plain binary file holding exactly the bytes
// of ReplayRecording::serialize (magic "HTRP", version, ...). Recommended
// extension: ".thr". Uses std::filesystem/fstream only, so it works headless
// (server) and inside UE5 alike (UE can instead pass its own paths).

namespace thalassa::sim {

// Writes `recording` to `path` (overwrites). Returns false on I/O failure.
[[nodiscard]] bool save_replay_file(const std::filesystem::path& path, const ReplayRecording& recording);

// Reads `path` into `out`. Returns false if the file is missing/unreadable,
// too short to hold a header, or has the wrong magic/version. A file that
// passes the header check but is truncated/corrupt mid-body aborts through
// ByteReader's bounds assert (project policy), so treat files from untrusted
// sources with care.
[[nodiscard]] bool load_replay_file(const std::filesystem::path& path, ReplayRecording& out);

}  // namespace thalassa::sim
