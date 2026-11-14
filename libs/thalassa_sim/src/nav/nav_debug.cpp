#include "thalassa/sim/nav/nav_debug.hpp"

#include <cstdio>

#include "thalassa/sim/components.hpp"

namespace thalassa::sim::nav::debug {

namespace {
constexpr const char* kNames[kLayerCount] = {"grid",   "coords",    "paths",   "destinations", "partial",
                                              "explored", "inflated", "smoothing", "panel", "radii",
                                              "stalled"};
static_assert(sizeof(kNames) / sizeof(kNames[0]) == kLayerCount, "add a name for every Layer");

constexpr std::uint32_t bit(Layer l) noexcept { return 1u << static_cast<unsigned>(l); }

std::string fixed(double v, int decimals) {
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

std::vector<CellRun> runs_where(const NavGrid& grid, const NavGrid* subtract_blocked_in) {
    std::vector<CellRun> out;
    for (std::int32_t y = 0; y < grid.height(); ++y) {
        std::int32_t begin = -1;
        for (std::int32_t x = 0; x <= grid.width(); ++x) {
            bool on = false;
            if (x < grid.width()) {
                on = grid.blocked(Cell{x, y}) && (subtract_blocked_in == nullptr || subtract_blocked_in->free(Cell{x, y}));
            }
            if (on && begin < 0) begin = x;
            if (!on && begin >= 0) {
                out.push_back(CellRun{y, begin, x});
                begin = -1;
            }
        }
    }
    return out;
}
}  // namespace

std::string_view layer_name(Layer layer) noexcept {
    const auto i = static_cast<std::size_t>(layer);
    return i < kLayerCount ? std::string_view(kNames[i]) : std::string_view("?");
}

bool parse_layer(std::string_view name, Layer& out) noexcept {
    for (std::size_t i = 0; i < kLayerCount; ++i) {
        if (name == kNames[i]) {
            out = static_cast<Layer>(i);
            return true;
        }
    }
    return false;
}

void NavDebugSettings::set_layer(Layer layer, bool on) noexcept {
    if (static_cast<std::size_t>(layer) >= kLayerCount) return;
    if (on) bits_ |= bit(layer);
    else bits_ &= ~bit(layer);
}

bool NavDebugSettings::layer_on(Layer layer) const noexcept {
    return static_cast<std::size_t>(layer) < kLayerCount && (bits_ & bit(layer)) != 0;
}

void NavDebugSettings::set_all_layers(bool on) noexcept {
    bits_ = on ? ((1u << kLayerCount) - 1u) : 0u;
}

bool NavDebugSettings::set_by_name(std::string_view name, bool on) noexcept {
    if (name == "master") { set_master(on); return true; }
    if (name == "all") { set_all_layers(on); return true; }
    Layer l;
    if (!parse_layer(name, l)) return false;
    set_layer(l, on);
    return true;
}

bool NavDebugSettings::toggle_by_name(std::string_view name) noexcept {
    if (name == "master") { set_master(!master_); return true; }
    Layer l;
    if (!parse_layer(name, l)) return false;
    toggle_layer(l);
    return true;
}

std::vector<std::string> NavDebugSettings::checklist() const {
    std::vector<std::string> lines;
    lines.reserve(kLayerCount + 1);
    lines.push_back(std::string(master_ ? "[x] " : "[ ] ") + "master (nav debug)");
    for (std::size_t i = 0; i < kLayerCount; ++i) {
        const bool on = (bits_ & (1u << i)) != 0;
        lines.push_back(std::string(on ? "[x] " : "[ ] ") + std::to_string(i + 1) + " " + kNames[i]);
    }
    return lines;
}

std::vector<CellRun> blocked_runs(const NavGrid& grid) { return runs_where(grid, nullptr); }

std::vector<CellRun> inflated_runs(const NavMap& map, std::size_t cls) {
    if (cls == 0 || cls >= map.class_count()) return {};
    return runs_where(map.grid(cls), &map.base());
}

std::vector<AgentInfo> collect_agents(const SimWorld& world) {
    using namespace components;
    std::vector<AgentInfo> out;
    const core::ecs::World& w = world.world();
    w.each<Position, Radius>([&](core::ecs::Entity e, const Position& pos, const Radius& radius) {
        if (!(radius.value > 0.0)) return;
        AgentInfo a;
        a.entity = e;
        a.position = pos.value;
        a.radius = radius.value;
        a.has_order = w.has<Destination>(e);
        if (w.has<MoveProgress>(e)) a.stalled_windows = w.get<MoveProgress>(e).stalled_windows;
        out.push_back(a);
    });
    return out;
}

std::vector<UnitRoute> collect_routes(const SimWorld& world) {
    using namespace components;
    std::vector<UnitRoute> out;
    const core::ecs::World& w = world.world();
    w.each<Position>([&](core::ecs::Entity e, const Position& pos) {
        const bool has_dest = w.has<Destination>(e);
        const bool has_path = w.has<Path>(e);
        if (!has_dest && !has_path) return;
        UnitRoute r;
        r.entity = e;
        r.position = pos.value;
        if (w.has<Radius>(e)) r.radius = w.get<Radius>(e).value;
        if (has_dest) {
            r.has_destination = true;
            r.destination = w.get<Destination>(e).target;
        }
        if (has_path) {
            const Path& p = w.get<Path>(e);
            for (std::uint16_t i = p.next; i < p.count && i < Path::kMaxWaypoints; ++i) r.waypoints.push_back(p.points[i]);
            r.partial = (p.flags & Path::kPartial) != 0;
            r.truncated = (p.flags & Path::kTruncated) != 0;
        }
        out.push_back(std::move(r));
    });
    return out;
}

PathResult probe_path(const NavMap& map, const core::math::Vec3& from, const core::math::Vec3& to,
                      core::Scalar unit_radius) {
    Pathfinder finder(map.grid(map.class_for_radius(unit_radius)));
    PathOptions options;
    options.keep_raw = true;
    options.record_explored = true;
    return finder.find_path(from, to, options);
}

NavDebugSummary summarize(const SimWorld& world, const PathResult* probe) {
    NavDebugSummary s;
    const auto& map = world.nav_map();
    if (map != nullptr) {
        s.has_map = true;
        s.map_hash = map->hash();
        s.grid_width = map->base().width();
        s.grid_height = map->base().height();
        s.cell_size = map->base().cell_size();
        s.size_classes = map->class_count();
        for (const CellRun& r : blocked_runs(map->base())) s.blocked_cells += static_cast<std::size_t>(r.x_end - r.x_begin);
    }
    if (world.path_planner() != nullptr) s.searches = world.path_planner()->searches();
    for (const UnitRoute& r : collect_routes(world)) {
        if (r.has_destination) ++s.units_with_destination;
        if (!r.waypoints.empty()) ++s.units_with_path;
        if (r.partial) ++s.partial_paths;
    }
    for (const AgentInfo& a : collect_agents(world)) {
        ++s.agents;
        if (a.stalled_windows > 0) ++s.stalled_agents;
    }
    if (probe != nullptr) {
        s.probe_expansions = probe->expansions;
        s.probe_cost = probe->cost;
        s.probe_explored = probe->explored.size();
    }
    return s;
}

std::vector<std::string> summary_lines(const NavDebugSummary& s) {
    std::vector<std::string> lines;
    if (!s.has_map) {
        lines.push_back("nav: no map (units move in straight lines)");
        return lines;
    }
    char hash[32];
    std::snprintf(hash, sizeof hash, "%016llx", static_cast<unsigned long long>(s.map_hash));
    lines.push_back(std::string("nav map ") + hash);
    lines.push_back("grid " + std::to_string(s.grid_width) + "x" + std::to_string(s.grid_height) + " cells, " +
                    fixed(s.cell_size, 2) + " m, " + std::to_string(s.blocked_cells) + " blocked, " +
                    std::to_string(s.size_classes) + " size classes");
    lines.push_back("units: " + std::to_string(s.units_with_destination) + " ordered, " +
                    std::to_string(s.units_with_path) + " on a path, " + std::to_string(s.partial_paths) + " partial, " +
                    std::to_string(s.agents) + " crowd agents, " + std::to_string(s.stalled_agents) + " stalled");
    lines.push_back("searches so far: " + std::to_string(s.searches));
    if (s.probe_explored != 0 || s.probe_expansions != 0) {
        lines.push_back("probe: " + std::to_string(s.probe_expansions) + " expansions, " +
                        std::to_string(s.probe_explored) + " cells explored, cost " + fixed(s.probe_cost / 10.0, 1) +
                        " cells");
    }
    return lines;
}

}  // namespace thalassa::sim::nav::debug
