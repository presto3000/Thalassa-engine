#include "thalassa/sim/replay_file.hpp"

#include <fstream>
#include <iterator>

namespace thalassa::sim {

bool save_replay_file(const std::filesystem::path& path, const ReplayRecording& recording) {
    core::serialization::ByteWriter w;
    recording.serialize(w);
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(w.bytes().data()), static_cast<std::streamsize>(w.size()));
    return static_cast<bool>(f);
}

bool load_replay_file(const std::filesystem::path& path, ReplayRecording& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    const std::vector<char> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (raw.size() < 8) return false;  // magic + version
    std::vector<std::byte> bytes(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) bytes[i] = static_cast<std::byte>(raw[i]);
    core::serialization::ByteReader reader(bytes);
    return out.deserialize(reader) && reader.at_end();
}

}  // namespace thalassa::sim
