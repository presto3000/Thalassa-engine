#include "thalassa/sim/nav/nav_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace thalassa::sim::nav {

namespace {
constexpr std::uint32_t kMapMagic = 0x4D4E4854;  // "THNM"
constexpr std::uint32_t kMapVersion = 1;
}  // namespace

std::shared_ptr<const NavMap> NavMap::create(NavGrid base, std::vector<core::Scalar> class_radii) {
    class_radii.erase(std::remove_if(class_radii.begin(), class_radii.end(),
                                     [](core::Scalar r) { return !std::isfinite(r) || !(r > 0.0); }),
                      class_radii.end());
    std::sort(class_radii.begin(), class_radii.end());
    class_radii.erase(std::unique(class_radii.begin(), class_radii.end()), class_radii.end());
    if (class_radii.size() > kMaxClasses - 1) class_radii.resize(kMaxClasses - 1);

    std::shared_ptr<NavMap> map(new NavMap());
    map->radii_.push_back(0.0);
    map->grids_.push_back(std::move(base));
    for (const core::Scalar r : class_radii) {
        map->radii_.push_back(r);
        map->grids_.push_back(map->grids_[0].inflated(r));
    }

    std::uint64_t h = map->grids_[0].hash();
    for (const core::Scalar r : map->radii_) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &r, sizeof(bits));
        for (int i = 0; i < 8; ++i) {
            h ^= (bits >> (8 * i)) & 0xFFu;
            h *= 1099511628211ULL;
        }
    }
    map->hash_ = h;
    return map;
}

std::size_t NavMap::class_for_radius(core::Scalar radius) const noexcept {
    if (!(radius > 0.0)) return 0;  // also catches NaN
    for (std::size_t i = 1; i < radii_.size(); ++i) {
        if (radii_[i] >= radius) return i;
    }
    return radii_.size() - 1;
}

std::vector<std::byte> NavMap::serialize() const {
    std::vector<std::byte> out;
    auto put = [&out](const void* data, std::size_t n) {
        const auto* p = static_cast<const std::byte*>(data);
        out.insert(out.end(), p, p + n);
    };
    const std::uint32_t magic = kMapMagic, version = kMapVersion;
    const auto extra = static_cast<std::uint32_t>(radii_.size() - 1);
    put(&magic, 4);
    put(&version, 4);
    put(&extra, 4);
    for (std::size_t i = 1; i < radii_.size(); ++i) put(&radii_[i], sizeof(core::Scalar));
    const auto grid = grids_[0].serialize();
    out.insert(out.end(), grid.begin(), grid.end());
    return out;
}

std::shared_ptr<const NavMap> NavMap::deserialize(std::span<const std::byte> bytes) {
    std::uint32_t magic = 0, version = 0, extra = 0;
    if (bytes.size() < 12) return nullptr;
    std::memcpy(&magic, bytes.data(), 4);
    std::memcpy(&version, bytes.data() + 4, 4);
    std::memcpy(&extra, bytes.data() + 8, 4);
    bytes = bytes.subspan(12);
    if (magic != kMapMagic || version != kMapVersion || extra > kMaxClasses - 1) return nullptr;
    if (bytes.size() < extra * sizeof(core::Scalar)) return nullptr;

    std::vector<core::Scalar> radii(extra);
    for (std::uint32_t i = 0; i < extra; ++i) {
        std::memcpy(&radii[i], bytes.data() + i * sizeof(core::Scalar), sizeof(core::Scalar));
        if (!std::isfinite(radii[i]) || !(radii[i] > 0.0)) return nullptr;
        if (i > 0 && !(radii[i] > radii[i - 1])) return nullptr;  // must be strictly ascending
    }
    bytes = bytes.subspan(extra * sizeof(core::Scalar));

    auto grid = NavGrid::deserialize(bytes);
    if (!grid) return nullptr;
    return create(std::move(*grid), std::move(radii));
}

}  // namespace thalassa::sim::nav
