#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/protocol.hpp"
#include "thalassa/net/reliable_channel.hpp"

using namespace thalassa::net;
using thalassa::sim::Command;

namespace {

Command sample_command() {
    return Command::spawn_unit({1.5, -2.5, 3.0}, thalassa::core::PlayerId{7}, 1, 123.0, 14.0, 12.5, 0.75)
        .as_ranged(0.3, 22.0);
}

bool same(const Command& a, const Command& b) {
    return a.type == b.type && a.entity == b.entity && a.position == b.position && a.speed == b.speed &&
           a.owner == b.owner && a.team == b.team && a.max_health == b.max_health &&
           a.attack_damage == b.attack_damage && a.attack_range == b.attack_range &&
           a.attack_cooldown_seconds == b.attack_cooldown_seconds && a.attack_kind == b.attack_kind &&
           a.attack_windup_seconds == b.attack_windup_seconds && a.projectile_speed == b.projectile_speed;
}

}  // namespace

TEST_CASE("protocol: a Command is exactly kCommandWireBytes on the wire and round-trips", "[protocol]") {
    WireWriter w;
    write_command(w, sample_command());
    REQUIRE(w.size() == kCommandWireBytes);

    WireReader r(w.bytes());
    Command out;
    REQUIRE(read_command(r, out));
    REQUIRE(same(out, sample_command()));

    const Command mv = Command::move_to(thalassa::core::ecs::Entity{5, 2}, {9, 8, 7}, 4.5);
    WireWriter w2;
    write_command(w2, mv);
    WireReader r2(w2.bytes());
    REQUIRE(read_command(r2, out));
    REQUIRE(same(out, mv));
}

TEST_CASE("protocol: every message type round-trips", "[protocol]") {
    REQUIRE(decode_hello(encode(HelloMsg{1, 42}))->match_id == 42);

    const auto cc = decode_client_command(encode(ClientCommandMsg{777, sample_command()}));
    REQUIRE(cc);
    REQUIRE(cc->target_tick == 777);
    REQUIRE(same(cc->command, sample_command()));

    WelcomeMsg wm;
    wm.player_id = 3;
    wm.team = 1;
    wm.match_id = 9;
    wm.rng_seed = 0xDEADBEEFCAFEull;
    wm.server_tick = 1234;
    const auto w = decode_welcome(encode(wm));
    REQUIRE(w);
    REQUIRE(w->player_id == 3);
    REQUIRE(w->team == 1);
    REQUIRE(w->rng_seed == 0xDEADBEEFCAFEull);
    REQUIRE(w->server_tick == 1234);

    REQUIRE(decode_reject(encode(RejectMsg{RejectReason::UnknownMatch}))->reason == RejectReason::UnknownMatch);

    BatchMsg b;
    b.up_to_tick = 99;
    for (std::uint64_t i = 0; i < 5; ++i) b.entries.push_back({i, sample_command()});
    const auto db = decode_batch(encode(b));
    REQUIRE(db);
    REQUIRE(db->up_to_tick == 99);
    REQUIRE(db->entries.size() == 5);
    REQUIRE(db->entries[4].tick == 4);

    REQUIRE(peek_type(encode_goodbye()) == MessageType::Goodbye);
}

TEST_CASE("protocol: a full batch fits a reliable channel message", "[protocol]") {
    const std::size_t n = max_batch_entries(ReliableChannel::kMaxMessageBytes);
    REQUIRE(n >= 8);
    BatchMsg b;
    for (std::size_t i = 0; i < n; ++i) b.entries.push_back({i, sample_command()});
    const auto bytes = encode(b);
    REQUIRE(bytes.size() <= ReliableChannel::kMaxMessageBytes);
    REQUIRE(bytes.size() == kBatchHeaderBytes + n * kBatchEntryBytes);
    REQUIRE(decode_batch(bytes)->entries.size() == n);
}

TEST_CASE("protocol: malformed input is rejected, never crashes", "[protocol]") {
    // Empty / unknown type / truncated at every length.
    REQUIRE_FALSE(peek_type({}));
    const std::vector<std::byte> unknown{std::byte{0x7F}, std::byte{0}};
    REQUIRE_FALSE(peek_type(unknown));

    const auto good = encode(ClientCommandMsg{1, sample_command()});
    for (std::size_t len = 0; len < good.size(); ++len) {
        REQUIRE_FALSE(decode_client_command(std::span<const std::byte>(good.data(), len)));
    }
    auto trailing = good;
    trailing.push_back(std::byte{0});
    REQUIRE_FALSE(decode_client_command(trailing));  // trailing garbage

    const auto good_batch = encode(BatchMsg{5, {{1, sample_command()}}});
    for (std::size_t len = 0; len < good_batch.size(); ++len) {
        REQUIRE_FALSE(decode_batch(std::span<const std::byte>(good_batch.data(), len)));
    }

    // A batch claiming 65535 entries must not allocate/loop on that claim.
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Batch));
    w.u64(0);
    w.u16(65535);
    REQUIRE_FALSE(decode_batch(w.bytes()));

    // Wrong decoder for the type.
    REQUIRE_FALSE(decode_hello(good));
}

TEST_CASE("protocol: NaN/inf numbers and bad enums are rejected", "[protocol]") {
    auto cmd = sample_command();
    cmd.position.x = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(decode_client_command(encode(ClientCommandMsg{1, cmd})));

    cmd = sample_command();
    cmd.attack_damage = std::numeric_limits<double>::infinity();
    REQUIRE_FALSE(decode_client_command(encode(ClientCommandMsg{1, cmd})));

    cmd = sample_command();
    cmd.attack_kind = 5;
    REQUIRE_FALSE(decode_client_command(encode(ClientCommandMsg{1, cmd})));

    cmd = sample_command();
    cmd.type = static_cast<Command::Type>(9);
    REQUIRE_FALSE(decode_client_command(encode(ClientCommandMsg{1, cmd})));

    WelcomeMsg wm;
    wm.tick_duration_seconds = 0.0;
    REQUIRE_FALSE(decode_welcome(encode(wm)));

    const std::vector<std::byte> bad_reject{std::byte{11}, std::byte{99}};
    REQUIRE_FALSE(decode_reject(bad_reject));
}
