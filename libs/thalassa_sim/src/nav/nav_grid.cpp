#include "thalassa/sim/nav/nav_grid.hpp"

#include <cmath>
#include <cstring>

#include "thalassa/core/assert.hpp"

namespace thalassa::sim::nav {

namespace {

constexpr std::uint32_t kMagic = 0x564E4854;  // "THNV"
constexpr std::uint32_t kVersion = 1;
constexpr std::size_t kHeaderBytes = 4 + 4 + 8 + 8 + 8 + 4 + 4;

bool finite(core::Scalar v) noexcept { return std::isfinite(v); }

template <typename T>
void put(std::vector<std::byte>& out, const T& v) {
    const auto* p = reinterpret_cast<const std::byte*>(&v);
    out.insert(out.end(), p, p + sizeof(T));
}

template <typename T>
bool take(std::span<const std::byte>& in, T& v) {
    if (in.size() < sizeof(T)) return false;
    std::memcpy(&v, in.data(), sizeof(T));
    in = in.subspan(sizeof(T));
    return true;
}

}  // namespace

NavGrid::NavGrid(core::Scalar cell_size, core::Scalar origin_x, core::Scalar origin_y, std::int32_t width,
                 std::int32_t height)
    : cell_size_(cell_size), origin_x_(origin_x), origin_y_(origin_y), width_(width), height_(height) {
    THALASSA_ASSERT_MSG(finite(cell_size) && cell_size > 0.0, "NavGrid: cell_size must be finite and > 0");
    THALASSA_ASSERT_MSG(finite(origin_x) && finite(origin_y), "NavGrid: origin must be finite");
    THALASSA_ASSERT_MSG(width >= 1 && width <= kMaxDimension && height >= 1 && height <= kMaxDimension,
                        "NavGrid: dimensions out of range");
    blocked_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

void NavGrid::set_blocked(Cell c, bool is_blocked) noexcept {
    if (!in_bounds(c)) return;
    std::uint8_t& slot = blocked_[index_of(c)];
    const std::uint8_t value = is_blocked ? 1 : 0;
    if (slot != value) {
        slot = value;
        ++revision_;
    }
}

std::size_t NavGrid::block_rect(core::Scalar min_x, core::Scalar min_y, core::Scalar max_x, core::Scalar max_y,
                                bool is_blocked) {
    std::size_t changed = 0;
    for (std::int32_t y = 0; y < height_; ++y) {
        for (std::int32_t x = 0; x < width_; ++x) {
            const core::math::Vec3 c = cell_center(Cell{x, y});
            if (c.x >= min_x && c.x <= max_x && c.y >= min_y && c.y <= max_y) {
                const bool before = blocked(Cell{x, y});
                set_blocked(Cell{x, y}, is_blocked);
                if (before != is_blocked) ++changed;
            }
        }
    }
    return changed;
}

std::size_t NavGrid::block_circle(core::Scalar center_x, core::Scalar center_y, core::Scalar radius,
                                  bool is_blocked) {
    std::size_t changed = 0;
    const core::Scalar r2 = radius * radius;
    for (std::int32_t y = 0; y < height_; ++y) {
        for (std::int32_t x = 0; x < width_; ++x) {
            const core::math::Vec3 c = cell_center(Cell{x, y});
            const core::Scalar dx = c.x - center_x;
            const core::Scalar dy = c.y - center_y;
            if (dx * dx + dy * dy <= r2) {
                const bool before = blocked(Cell{x, y});
                set_blocked(Cell{x, y}, is_blocked);
                if (before != is_blocked) ++changed;
            }
        }
    }
    return changed;
}

Cell NavGrid::to_cell_unclamped(const core::math::Vec3& p) const noexcept {
    // Clamp before the int conversion so absurd coordinates cannot overflow it.
    constexpr core::Scalar kLimit = 1.0e9;
    const core::Scalar fx = std::floor((p.x - origin_x_) / cell_size_);
    const core::Scalar fy = std::floor((p.y - origin_y_) / cell_size_);
    const core::Scalar cx = fx < -kLimit ? -kLimit : (fx > kLimit ? kLimit : fx);
    const core::Scalar cy = fy < -kLimit ? -kLimit : (fy > kLimit ? kLimit : fy);
    return Cell{static_cast<std::int32_t>(cx), static_cast<std::int32_t>(cy)};
}

std::optional<Cell> NavGrid::to_cell(const core::math::Vec3& p) const noexcept {
    const Cell c = to_cell_unclamped(p);
    if (!in_bounds(c)) return std::nullopt;
    return c;
}

core::math::Vec3 NavGrid::cell_center(Cell c, core::Scalar z) const noexcept {
    return core::math::Vec3{origin_x_ + (static_cast<core::Scalar>(c.x) + 0.5) * cell_size_,
                            origin_y_ + (static_cast<core::Scalar>(c.y) + 0.5) * cell_size_, z};
}

std::optional<Cell> NavGrid::nearest_free_cell(Cell from, std::int32_t max_radius_cells) const noexcept {
    if (free(from)) return from;

    std::optional<Cell> best;
    std::int64_t best_d2 = 0;
    std::size_t best_index = 0;

    for (std::int32_t r = 1; r <= max_radius_cells; ++r) {
        // Anything on ring r is at least r away; once we hold a candidate closer than that, stop.
        if (best && best_d2 < static_cast<std::int64_t>(r) * r) break;
        for (std::int32_t dy = -r; dy <= r; ++dy) {
            const bool edge_row = (dy == -r || dy == r);
            const std::int32_t step = edge_row ? 1 : 2 * r;
            for (std::int32_t dx = -r; dx <= r; dx += step) {
                const Cell c{from.x + dx, from.y + dy};
                if (!in_bounds(c) || blocked_[index_of(c)] != 0) continue;
                const std::int64_t d2 = static_cast<std::int64_t>(dx) * dx + static_cast<std::int64_t>(dy) * dy;
                const std::size_t idx = index_of(c);
                if (!best || d2 < best_d2 || (d2 == best_d2 && idx < best_index)) {
                    best = c;
                    best_d2 = d2;
                    best_index = idx;
                }
            }
        }
    }
    return best;
}

std::uint64_t NavGrid::hash() const noexcept {
    std::uint64_t h = 14695981039346656037ULL;
    auto mix_byte = [&h](std::uint8_t b) {
        h ^= b;
        h *= 1099511628211ULL;
    };
    auto mix = [&mix_byte](const void* data, std::size_t n) {
        const auto* p = static_cast<const std::uint8_t*>(data);
        for (std::size_t i = 0; i < n; ++i) mix_byte(p[i]);
    };
    mix(&cell_size_, sizeof(cell_size_));
    mix(&origin_x_, sizeof(origin_x_));
    mix(&origin_y_, sizeof(origin_y_));
    mix(&width_, sizeof(width_));
    mix(&height_, sizeof(height_));
    for (const std::uint8_t b : blocked_) mix_byte(b);
    return h;
}

std::vector<std::byte> NavGrid::serialize() const {
    std::vector<std::byte> out;
    out.reserve(kHeaderBytes + blocked_.size());
    put(out, kMagic);
    put(out, kVersion);
    put(out, cell_size_);
    put(out, origin_x_);
    put(out, origin_y_);
    put(out, width_);
    put(out, height_);
    const auto* cells = reinterpret_cast<const std::byte*>(blocked_.data());
    out.insert(out.end(), cells, cells + blocked_.size());
    return out;
}

std::optional<NavGrid> NavGrid::deserialize(std::span<const std::byte> bytes) {
    std::uint32_t magic = 0, version = 0;
    core::Scalar cell_size = 0, ox = 0, oy = 0;
    std::int32_t w = 0, h = 0;
    if (!take(bytes, magic) || !take(bytes, version) || !take(bytes, cell_size) || !take(bytes, ox) ||
        !take(bytes, oy) || !take(bytes, w) || !take(bytes, h)) {
        return std::nullopt;
    }
    if (magic != kMagic || version != kVersion) return std::nullopt;
    if (!finite(cell_size) || cell_size <= 0.0 || !finite(ox) || !finite(oy)) return std::nullopt;
    if (w < 1 || w > kMaxDimension || h < 1 || h > kMaxDimension) return std::nullopt;
    const std::size_t count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    if (bytes.size() != count) return std::nullopt;  // truncated or trailing bytes

    NavGrid grid(cell_size, ox, oy, w, h);
    for (std::size_t i = 0; i < count; ++i) {
        const auto b = static_cast<std::uint8_t>(bytes[i]);
        if (b > 1) return std::nullopt;
        grid.blocked_[i] = b;
    }
    return grid;
}

bool NavGrid::operator==(const NavGrid& other) const noexcept {
    return cell_size_ == other.cell_size_ && origin_x_ == other.origin_x_ && origin_y_ == other.origin_y_ &&
           width_ == other.width_ && height_ == other.height_ && blocked_ == other.blocked_;
}

}  // namespace thalassa::sim::nav
