#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "thalassa/core/types.hpp"
#include "thalassa/sim/nav/nav_grid.hpp"

// NavMap: everything the simulation needs to route units, immutable once built and shared
// (shared_ptr) by every SimWorld of a match: the server's, each client's confirmed and
// predicted world, replay players. It is MATCH CONFIGURATION, like the rng seed: all
// participants must hold an identical one, which hash() lets them verify.
//
// Contents: the base walkability grid plus one inflated copy per unit-size class. Class 0 is
// always radius 0 (the base grid); further classes come from the radii given at creation
// (sorted, de-duplicated, at most kMaxClasses - 1). A unit with radius r is routed on the
// smallest class whose radius is >= r (the largest class if r exceeds them all).
//
// serialize()/deserialize() make it a self-contained blob (stored inside replay files);
// deserialize() is safe on untrusted bytes.

namespace thalassa::sim::nav {

class NavMap {
public:
    static constexpr std::size_t kMaxClasses = 8;

    // `class_radii`: extra unit sizes (meters). Non-finite, non-positive and duplicate values are dropped.
    [[nodiscard]] static std::shared_ptr<const NavMap> create(NavGrid base, std::vector<core::Scalar> class_radii = {});

    [[nodiscard]] std::size_t class_count() const noexcept { return grids_.size(); }
    [[nodiscard]] core::Scalar class_radius(std::size_t cls) const noexcept { return radii_[cls]; }
    [[nodiscard]] const NavGrid& grid(std::size_t cls) const noexcept { return grids_[cls]; }
    [[nodiscard]] const NavGrid& base() const noexcept { return grids_[0]; }
    [[nodiscard]] std::size_t class_for_radius(core::Scalar radius) const noexcept;

    // Identifies the whole map (base grid + class radii): equal hash <=> same routing behavior.
    [[nodiscard]] std::uint64_t hash() const noexcept { return hash_; }

    [[nodiscard]] std::vector<std::byte> serialize() const;
    [[nodiscard]] static std::shared_ptr<const NavMap> deserialize(std::span<const std::byte> bytes);

private:
    NavMap() = default;
    std::vector<core::Scalar> radii_;  // radii_[0] == 0
    std::vector<NavGrid> grids_;       // grids_[i] = base inflated by radii_[i]
    std::uint64_t hash_ = 0;
};

}  // namespace thalassa::sim::nav
