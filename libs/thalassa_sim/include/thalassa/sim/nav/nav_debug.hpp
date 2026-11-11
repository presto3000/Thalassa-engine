#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/math/vector.hpp"
#include "thalassa/sim/nav/nav_map.hpp"
#include "thalassa/sim/nav/pathfinder.hpp"
#include "thalassa/sim/sim_world.hpp"

// Navigation debug support: READ-ONLY helpers for drawing what pathfinding does.
//
// Nothing here changes the simulation (it only takes const references), so turning debug
// drawing on can never cause a desync. Presentation layers (Unreal's UThalassaNavDebugSubsystem,
// a future native renderer, tests) use the same helpers and the same switch logic.
//
// Two parts:
//   1. NavDebugSettings: one master switch plus one switch per Layer. The master switch
//      short-circuits everything (enabled() is false for every layer when it is off), so a
//      disabled overlay costs one branch. New steps of the plan add layers at the END of
//      the enum so existing hotkey numbers stay stable.
//   2. Data extraction: cells to draw, per-unit routes, a "what would A* do" probe with the
//      explored cells, and summary numbers for a text panel.

namespace thalassa::sim::nav::debug {

enum class Layer : std::uint8_t {
    Grid = 0,       // blocked cells of the base map
    CellCoords,     // cell coordinates (text)
    Paths,          // each moving unit's remaining route and waypoints
    Destinations,   // each unit's order target
    PartialPaths,   // routes that end short of the goal (goal unreachable)
    Explored,       // cells the A* probe expanded (probe_path)
    Inflated,       // extra blocked area added by a unit-size class
    Smoothing,      // raw grid path vs smoothed path (probe_path)
    Panel,          // text panel: map hash, counts, search effort
    kCount
};
inline constexpr std::size_t kLayerCount = static_cast<std::size_t>(Layer::kCount);

// Short stable lowercase name, as used in console commands ("grid", "coords", "paths", ...).
[[nodiscard]] std::string_view layer_name(Layer layer) noexcept;
// Inverse of layer_name; false if unknown.
[[nodiscard]] bool parse_layer(std::string_view name, Layer& out) noexcept;

class NavDebugSettings {
public:
    // Master switch: off by default. Layers keep their own state while it is off.
    void set_master(bool on) noexcept { master_ = on; }
    [[nodiscard]] bool master() const noexcept { return master_; }

    void set_layer(Layer layer, bool on) noexcept;
    void toggle_layer(Layer layer) noexcept { set_layer(layer, !layer_on(layer)); }
    // The layer's own switch, ignoring the master.
    [[nodiscard]] bool layer_on(Layer layer) const noexcept;
    // What drawing code asks: master AND layer.
    [[nodiscard]] bool enabled(Layer layer) const noexcept { return master_ && layer_on(layer); }
    // True if any drawing at all should happen.
    [[nodiscard]] bool any() const noexcept { return master_ && bits_ != 0; }

    // Console-style access by name: "master"/"all" are accepted too. False if the name is unknown.
    bool set_by_name(std::string_view name, bool on) noexcept;
    bool toggle_by_name(std::string_view name) noexcept;

    // Layer switches: everything on / everything off (master untouched).
    void set_all_layers(bool on) noexcept;
    // The panic key: master off.
    void kill() noexcept { master_ = false; }

    // On-screen checklist, one line per layer plus the master:  "[x] 3 paths".
    // Numbers are the hotkey digits (1-based layer index).
    [[nodiscard]] std::vector<std::string> checklist() const;

private:
    static constexpr std::uint32_t kDefaultLayers = (1u << static_cast<unsigned>(Layer::Paths)) |
                                                    (1u << static_cast<unsigned>(Layer::Destinations)) |
                                                    (1u << static_cast<unsigned>(Layer::PartialPaths));
    bool master_ = false;
    std::uint32_t bits_ = kDefaultLayers;
};

// --- data extraction ----------------------------------------------------------------------

// A horizontal run of cells [x_begin, x_end) in row y, so a big blocked area is a few boxes.
struct CellRun {
    std::int32_t y = 0;
    std::int32_t x_begin = 0;
    std::int32_t x_end = 0;
};

// Blocked cells of `grid`, run-length merged per row, rows ascending.
[[nodiscard]] std::vector<CellRun> blocked_runs(const NavGrid& grid);
// Cells blocked in size class `cls` but free in the base map: the area a unit of that size must
// additionally keep out of. Empty for class 0.
[[nodiscard]] std::vector<CellRun> inflated_runs(const NavMap& map, std::size_t cls);

struct UnitRoute {
    core::ecs::Entity entity;
    core::math::Vec3 position;
    core::Scalar radius = 0.0;                      // 0 if the unit has no Radius
    bool has_destination = false;
    core::math::Vec3 destination;
    std::vector<core::math::Vec3> waypoints;        // remaining route, starting at the waypoint being walked to
    bool partial = false;                           // Path::kPartial
    bool truncated = false;                         // Path::kTruncated (more legs will be planned later)
};

// Every unit that has a Destination or a Path, in the world's deterministic iteration order.
[[nodiscard]] std::vector<UnitRoute> collect_routes(const SimWorld& world);

// One-off "what would the planner do" query for the given unit size. Uses its own Pathfinder, so
// it never disturbs the simulation's planner. Returns an empty result (NoPath) if the map is null.
// The result contains the smoothed path, the raw grid path and every explored cell.
[[nodiscard]] PathResult probe_path(const NavMap& map, const core::math::Vec3& from, const core::math::Vec3& to,
                                    core::Scalar unit_radius = 0.0);

struct NavDebugSummary {
    bool has_map = false;
    std::uint64_t map_hash = 0;
    std::int32_t grid_width = 0;
    std::int32_t grid_height = 0;
    core::Scalar cell_size = 0.0;
    std::size_t size_classes = 0;
    std::size_t blocked_cells = 0;
    std::size_t units_with_destination = 0;
    std::size_t units_with_path = 0;
    std::size_t partial_paths = 0;
    std::uint64_t searches = 0;                      // planner queries so far
    // Filled in only when a probe was run and passed to summarize().
    std::uint32_t probe_expansions = 0;
    std::int32_t probe_cost = 0;
    std::size_t probe_explored = 0;
};

[[nodiscard]] NavDebugSummary summarize(const SimWorld& world, const PathResult* probe = nullptr);
// Text for the panel layer, one string per line.
[[nodiscard]] std::vector<std::string> summary_lines(const NavDebugSummary& summary);

}  // namespace thalassa::sim::nav::debug
