#include <filesystem>
#include <fstream>
#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/sim/components.hpp"
#include "thalassa/sim/replay_file.hpp"
#include "thalassa/sim/replay_recording.hpp"

using thalassa::core::ecs::Entity;
using thalassa::core::serialization::ByteReader;
using thalassa::core::serialization::ByteWriter;
using thalassa::sim::Command;
using thalassa::sim::ReplayPlayer;
using thalassa::sim::ReplayRecorder;
using thalassa::sim::ReplayRecording;
using thalassa::sim::components::Health;
using thalassa::sim::components::Team;

namespace {

// Unit-attack scenario: two teams of 8 spawn facing each other, then are
// ordered to converge on the midline and fight until one side is thinned out.
ReplayRecording record_fight(std::uint32_t checkpoint_interval = 30, std::uint64_t ticks = 900) {
    ReplayRecorder rec(/*seed=*/2024, 1.0 / 60.0, checkpoint_interval);

    std::vector<Command> spawns;
    for (int i = 0; i < 8; ++i) {
        const double y = i * 3.0;
        spawns.push_back(Command::spawn_unit({-40.0, y, 0.0}, {}, 0, 100.0, 12.0, 15.0, 0.8));
        spawns.push_back(Command::spawn_unit({40.0, y + 1.0, 0.0}, {}, 1, 100.0, 9.0, 15.0, 0.6));
    }
    rec.step(spawns);

    std::vector<Command> orders;
    rec.sim().world().each<Team>([&](Entity e, const Team& t) {
        orders.push_back(Command::move_to(e, {0.0, 10.0, 0.0}, t.id == 0 ? 8.0 : 7.0));
    });
    rec.step(orders);

    while (rec.current_tick() < ticks) rec.step();
    return rec.finish();
}

std::uint64_t total_health(const thalassa::sim::SimWorld& sim) {
    double sum = 0.0;
    sim.world().each<Health>([&](Entity, const Health& h) { sum += h.current; });
    return static_cast<std::uint64_t>(sum);
}

}  // namespace

TEST_CASE("replay: the fight actually happens (not a trivially empty recording)", "[replay_recording]") {
    const ReplayRecording r = record_fight();
    REQUIRE(r.total_ticks == 900);
    REQUIRE(r.commands.tick_count() == 2);
    REQUIRE(r.checkpoints.size() == 30);

    ReplayPlayer p(r);
    p.seek(1);
    const std::uint64_t start_health = total_health(p.sim());
    REQUIRE(start_health == 16 * 100);
    p.seek(r.total_ticks);
    REQUIRE(total_health(p.sim()) < start_health);  // damage was dealt
}

TEST_CASE("replay: playback reproduces the recorded final state bit-for-bit", "[replay_recording]") {
    const ReplayRecording r = record_fight();
    ReplayPlayer p(r);
    while (p.step_tick()) {}
    REQUIRE(p.finished());
    REQUIRE_FALSE(p.desynced());
    REQUIRE(p.final_checksum_matches());
}

TEST_CASE("replay: file round-trip preserves everything and playback still matches", "[replay_recording]") {
    const ReplayRecording r = record_fight();
    ByteWriter w;
    r.serialize(w);

    ReplayRecording loaded;
    ByteReader reader(w.bytes());
    REQUIRE(loaded.deserialize(reader));
    REQUIRE(reader.at_end());
    REQUIRE(loaded.seed == r.seed);
    REQUIRE(loaded.final_checksum == r.final_checksum);
    REQUIRE(loaded.checkpoints.size() == r.checkpoints.size());

    // Re-serializing yields identical bytes (canonical, order-independent format).
    ByteWriter w2;
    loaded.serialize(w2);
    REQUIRE(w2.bytes() == w.bytes());

    ReplayPlayer p(loaded);
    while (p.step_tick()) {}
    REQUIRE(p.final_checksum_matches());
}

TEST_CASE("replay: bad magic is rejected", "[replay_recording]") {
    ByteWriter w;
    w.write_u32(0xDEADBEEF);
    w.write_u32(1);
    ReplayRecording r;
    ByteReader reader(w.bytes());
    REQUIRE_FALSE(r.deserialize(reader));
}

TEST_CASE("replay: any playback speed ends in the identical state", "[replay_recording]") {
    const ReplayRecording r = record_fight();

    for (const double speed : {0.5, 1.0, 2.0, 4.0, 16.0}) {
        ReplayPlayer p(r);
        p.set_speed(speed);

        int frame = 0;
        while (!p.finished()) {
            p.advance((frame++ % 3 == 0) ? 0.033 : 0.011);
            REQUIRE(frame < 100000);
        }

        INFO("Playback speed: " << speed);
        INFO("Current tick: " << p.current_tick());
        INFO("Expected total ticks: " << r.total_ticks);
        INFO("Desynchronized: " << p.desynced());

        REQUIRE_FALSE(p.desynced());
        REQUIRE(p.final_checksum_matches());
    }
}

TEST_CASE("replay: speed scales ticks per real second; pause and frame-step work", "[replay_recording]") {
    const ReplayRecording r = record_fight();
    ReplayPlayer p(r);

    REQUIRE(p.advance(1.0) == 60);  // 1x: 1 s == 60 ticks
    p.set_speed(2.0);
    REQUIRE(p.advance(1.0) == 120);
    p.set_speed(0.5);
    REQUIRE(p.advance(1.0) == 30);

    const std::uint64_t t = p.current_tick();
    p.pause();
    REQUIRE(p.advance(5.0) == 0);
    REQUIRE(p.current_tick() == t);
    REQUIRE(p.step_tick());  // frame-step while paused
    REQUIRE(p.current_tick() == t + 1);

    p.resume();
    p.set_speed(0.0);
    REQUIRE(p.advance(5.0) == 0);
}

TEST_CASE("replay: seek forwards and backwards lands on the same states", "[replay_recording]") {
    const ReplayRecording r = record_fight();

    ReplayPlayer a(r);
    a.seek(500);
    const std::uint64_t at_500 = thalassa::sim::world_checksum(a.sim());

    a.seek(100);  // backwards -> restart + re-simulate
    REQUIRE(a.current_tick() == 100);
    a.seek(500);
    REQUIRE(thalassa::sim::world_checksum(a.sim()) == at_500);

    a.seek(1'000'000);  // clamped
    REQUIRE(a.finished());
    REQUIRE(a.final_checksum_matches());
}

TEST_CASE("replay: a tampered command is caught by checkpoints as a desync", "[replay_recording]") {
    ReplayRecording r = record_fight();
    // Tamper: add a stray move order at tick 100.
    r.commands.add(100, Command::move_to(Entity{0, 0}, {-30.0, 0.0, 0.0}, 20.0));

    ReplayPlayer p(r);
    while (p.step_tick()) {}
    REQUIRE(p.desynced());
    REQUIRE_FALSE(p.final_checksum_matches());
}

TEST_CASE("replay: submit() applies immediately yet replays identically", "[replay_recording]") {
    ReplayRecorder rec(7, 1.0 / 60.0, 30);

    // Spawn via submit: handles are available right away.
    std::vector<Entity> a, b;
    a = rec.submit(Command::spawn_unit({-10.0, 0.0, 0.0}, {}, 0, 100.0, 10.0, 15.0, 0.5));
    b = rec.submit(Command::spawn_unit({10.0, 0.0, 0.0}, {}, 1, 100.0, 10.0, 15.0, 0.5));
    REQUIRE(a.size() == 1);
    REQUIRE(rec.sim().world().is_alive(a[0]));
    rec.submit(Command::move_to(a[0], {0.0, 0.0, 0.0}, 6.0));
    rec.submit(Command::move_to(b[0], {0.0, 0.0, 0.0}, 6.0));

    for (int i = 0; i < 400; ++i) {
        if (i == 100) rec.submit(Command::move_to(a[0], {-5.0, 5.0, 0.0}, 4.0));
        rec.step();
    }
    const ReplayRecording r = rec.finish();

    ReplayPlayer p(r);
    while (p.step_tick()) {}
    REQUIRE_FALSE(p.desynced());
    REQUIRE(p.final_checksum_matches());
}

TEST_CASE("replay file: save/load round-trip on disk", "[replay_recording]") {
    const ReplayRecording r = record_fight();
    const auto path = std::filesystem::temp_directory_path() / "thalassa_test_fight.thr";

    REQUIRE(thalassa::sim::save_replay_file(path, r));
    ReplayRecording loaded;
    REQUIRE(thalassa::sim::load_replay_file(path, loaded));
    REQUIRE(loaded.final_checksum == r.final_checksum);

    ReplayPlayer p(loaded);
    while (p.step_tick()) {}
    REQUIRE(p.final_checksum_matches());

    std::filesystem::remove(path);
}

TEST_CASE("replay file: missing file and garbage are rejected", "[replay_recording]") {
    ReplayRecording out;
    REQUIRE_FALSE(thalassa::sim::load_replay_file("/nonexistent/dir/nope.thr", out));

    const auto path = std::filesystem::temp_directory_path() / "thalassa_test_garbage.thr";
    {
        std::ofstream f(path, std::ios::binary);
        f << "this is definitely not a replay file";
    }
    REQUIRE_FALSE(thalassa::sim::load_replay_file(path, out));
    std::filesystem::remove(path);
}
