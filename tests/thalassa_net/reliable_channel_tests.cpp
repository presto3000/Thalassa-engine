#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/loopback_transport.hpp"
#include "thalassa/net/reliable_channel.hpp"

using Catch::Approx;
using thalassa::net::ChannelConfig;
using thalassa::net::ChannelMessage;
using thalassa::net::Datagram;
using thalassa::net::LinkConditions;
using thalassa::net::LoopbackNetwork;
using thalassa::net::ReliableChannel;

namespace {

std::vector<std::byte> bytes_of(std::uint32_t value) {
    std::vector<std::byte> b(4);
    for (int i = 0; i < 4; ++i) b[static_cast<std::size_t>(i)] = static_cast<std::byte>((value >> (8 * i)) & 0xFFu);
    return b;
}

std::uint32_t value_of(const ChannelMessage& m) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(m.payload[static_cast<std::size_t>(i)]) << (8 * i);
    return v;
}

// Two peers connected through a simulated network, advanced in fixed steps.
struct Pair {
    explicit Pair(std::uint64_t seed, const LinkConditions& c, ChannelConfig cfg = {}) : net(seed) {
        net.set_default_conditions(c);
        ta = net.create_endpoint();
        tb = net.create_endpoint();
        a = std::make_unique<ReliableChannel>(*ta, tb->local_id(), 0.0, cfg);
        b = std::make_unique<ReliableChannel>(*tb, ta->local_id(), 0.0, cfg);
    }

    void step(double dt = 0.01) {
        a->update(now);
        b->update(now);
        net.advance(dt);
        now += dt;
        Datagram d;
        while (ta->receive(d)) a->on_datagram(d.payload, now);
        while (tb->receive(d)) b->on_datagram(d.payload, now);
    }

    void run_seconds(double seconds, double dt = 0.01) {
        const double end = now + seconds;
        while (now < end) step(dt);
    }

    LoopbackNetwork net;
    std::unique_ptr<thalassa::net::LoopbackTransport> ta, tb;
    std::unique_ptr<ReliableChannel> a, b;
    double now = 0.0;
};

std::vector<ChannelMessage> drain(ReliableChannel& ch) {
    std::vector<ChannelMessage> out;
    ChannelMessage m;
    while (ch.receive(m)) out.push_back(std::move(m));
    return out;
}

// Sends 1..n reliable numbered messages each way (respecting the window) and
// runs until all arrive, in order, exactly once. Returns false on timeout.
bool exchange_all(Pair& p, std::uint32_t n, double max_seconds, std::vector<std::uint32_t>* got_b = nullptr) {
    std::uint32_t next_a = 0, next_b = 0;
    std::vector<std::uint32_t> at_a, at_b;
    while (p.now < max_seconds) {
        while (next_a < n && p.a->send_reliable(bytes_of(next_a))) ++next_a;
        while (next_b < n && p.b->send_reliable(bytes_of(1000000 + next_b))) ++next_b;
        p.step();
        for (const auto& m : drain(*p.a)) at_a.push_back(value_of(m));
        for (const auto& m : drain(*p.b)) at_b.push_back(value_of(m));
        if (at_a.size() == n && at_b.size() == n && p.a->unacked_reliable_count() == 0 &&
            p.b->unacked_reliable_count() == 0) {
            break;
        }
    }
    if (got_b != nullptr) *got_b = at_b;
    if (at_a.size() != n || at_b.size() != n) return false;
    for (std::uint32_t i = 0; i < n; ++i) {
        if (at_b[i] != i) return false;             // A -> B in order, exactly once
        if (at_a[i] != 1000000 + i) return false;   // B -> A in order, exactly once
    }
    return true;
}

}  // namespace

TEST_CASE("channel: reliable messages arrive in order on a clean link", "[reliable]") {
    Pair p(1, {.latency_seconds = 0.02});
    REQUIRE(exchange_all(p, 300, 60.0));
    REQUIRE(p.a->stats().reliable_retransmits == 0);  // nothing lost, nothing resent
    REQUIRE(p.b->stats().reliable_retransmits == 0);
}

TEST_CASE("channel: survives 30% loss + jitter + duplication, both directions", "[reliable]") {
    Pair p(2, {.latency_seconds = 0.03, .jitter_seconds = 0.02, .loss_probability = 0.3,
               .duplicate_probability = 0.2});
    REQUIRE(exchange_all(p, 1000, 300.0));
    REQUIRE(p.a->stats().reliable_retransmits > 0);  // loss really happened and was repaired
}

TEST_CASE("channel: survives 70% loss", "[reliable]") {
    Pair p(3, {.latency_seconds = 0.02, .loss_probability = 0.7});
    REQUIRE(exchange_all(p, 200, 600.0));
}

TEST_CASE("channel: heavy reordering still yields in-order, exactly-once delivery", "[reliable]") {
    Pair p(4, {.latency_seconds = 0.1, .jitter_seconds = 0.09});
    REQUIRE(exchange_all(p, 500, 120.0));
}

TEST_CASE("channel: unreliable messages are never retransmitted or duplicated", "[reliable]") {
    Pair p(5, {.latency_seconds = 0.01, .loss_probability = 0.4, .duplicate_probability = 0.5});
    for (std::uint32_t i = 0; i < 400; ++i) {
        REQUIRE(p.a->send_unreliable(bytes_of(i)));
        p.step();
    }
    p.run_seconds(2.0);

    std::vector<bool> seen(400, false);
    std::size_t count = 0;
    for (const auto& m : drain(*p.b)) {
        REQUIRE_FALSE(m.reliable);
        const auto v = value_of(m);
        REQUIRE(v < 400);
        REQUIRE_FALSE(seen[v]);  // duplicated datagrams must not deliver twice
        seen[v] = true;
        ++count;
    }
    REQUIRE(count > 150);  // some lost (40% loss, no repair)...
    REQUIRE(count < 400);  // ...and none repaired
}

TEST_CASE("channel: reliable and unreliable can share packets", "[reliable]") {
    Pair p(6, {.latency_seconds = 0.01});
    REQUIRE(p.a->send_reliable(bytes_of(7)));
    REQUIRE(p.a->send_unreliable(bytes_of(8)));
    REQUIRE(p.a->send_reliable(bytes_of(9)));
    p.run_seconds(0.2);

    const auto got = drain(*p.b);
    REQUIRE(got.size() == 3);
    REQUIRE(value_of(got[0]) == 7);
    REQUIRE(got[0].reliable);
    REQUIRE(got[1].reliable);   // reliable messages are written first, in order
    REQUIRE(value_of(got[1]) == 9);
    REQUIRE_FALSE(got[2].reliable);
    REQUIRE(value_of(got[2]) == 8);
}

TEST_CASE("channel: acks stop retransmission and drain the send window", "[reliable]") {
    Pair p(7, {.latency_seconds = 0.02});
    for (std::uint32_t i = 0; i < 50; ++i) REQUIRE(p.a->send_reliable(bytes_of(i)));
    REQUIRE(p.a->unacked_reliable_count() == 50);
    p.run_seconds(1.0);
    REQUIRE(p.a->unacked_reliable_count() == 0);
    REQUIRE(p.a->stats().reliable_retransmits == 0);
}

TEST_CASE("channel: size limit and send window are enforced", "[reliable]") {
    ChannelConfig cfg;
    cfg.max_unacked_reliable = 8;
    Pair p(8, {.loss_probability = 1.0}, cfg);  // nothing is ever acked

    std::vector<std::byte> ok(ReliableChannel::kMaxMessageBytes);
    std::vector<std::byte> too_big(ReliableChannel::kMaxMessageBytes + 1);
    REQUIRE(p.a->send_reliable(ok));
    REQUIRE_FALSE(p.a->send_reliable(too_big));
    REQUIRE_FALSE(p.a->send_unreliable(too_big));

    for (int i = 0; i < 7; ++i) REQUIRE(p.a->send_reliable(bytes_of(1)));
    REQUIRE_FALSE(p.a->send_reliable(bytes_of(1)));  // window of 8 is full
    REQUIRE(p.a->unacked_reliable_count() == 8);
}

TEST_CASE("channel: a window-full sender recovers once acks arrive", "[reliable]") {
    ChannelConfig cfg;
    cfg.max_unacked_reliable = 4;
    Pair p(9, {.latency_seconds = 0.02, .loss_probability = 0.2}, cfg);
    REQUIRE(exchange_all(p, 100, 120.0));  // 100 messages through a window of 4
}

TEST_CASE("channel: large messages of varied sizes arrive intact", "[reliable]") {
    Pair p(10, {.latency_seconds = 0.02, .loss_probability = 0.25, .duplicate_probability = 0.1});
    std::vector<std::vector<std::byte>> sent;
    for (std::size_t i = 0; i < 60; ++i) {
        std::vector<std::byte> m((i * 37) % (ReliableChannel::kMaxMessageBytes + 1));
        for (std::size_t j = 0; j < m.size(); ++j) m[j] = static_cast<std::byte>((i * 31 + j * 7) & 0xFF);
        sent.push_back(m);
    }
    std::size_t next = 0;
    std::vector<ChannelMessage> got;
    while (p.now < 120.0 && got.size() < sent.size()) {
        while (next < sent.size() && p.a->send_reliable(sent[next])) ++next;
        p.step();
        for (auto& m : drain(*p.b)) got.push_back(std::move(m));
    }
    REQUIRE(got.size() == sent.size());
    for (std::size_t i = 0; i < sent.size(); ++i) REQUIRE(got[i].payload == sent[i]);
}

TEST_CASE("channel: many small messages are packed into few datagrams", "[reliable]") {
    Pair p(11, {.latency_seconds = 0.01});
    for (std::uint32_t i = 0; i < 100; ++i) REQUIRE(p.a->send_reliable(bytes_of(i)));
    p.a->update(0.0);  // one call flushes everything due
    // 100 messages * 11 bytes = 1100 bytes: fits in about 1-2 datagrams, not 100.
    REQUIRE(p.a->stats().packets_sent <= 2);
}

TEST_CASE("channel: RTT estimate converges to twice the one-way latency", "[reliable]") {
    Pair p(12, {.latency_seconds = 0.05});
    for (std::uint32_t i = 0; i < 100; ++i) {
        REQUIRE(p.a->send_reliable(bytes_of(i)));
        p.run_seconds(0.2);
    }
    REQUIRE(p.a->stats().smoothed_rtt_seconds == Approx(0.10).margin(0.03));
}

TEST_CASE("channel: timeout fires when the peer goes silent, keepalive prevents false timeouts", "[reliable]") {
    ChannelConfig cfg;
    cfg.timeout_seconds = 2.0;
    cfg.keepalive_seconds = 0.25;

    SECTION("idle but alive: keepalives keep the connection up") {
        Pair p(13, {.latency_seconds = 0.02}, cfg);
        p.run_seconds(10.0);  // no application traffic at all
        REQUIRE_FALSE(p.a->timed_out(p.now));
        REQUIRE_FALSE(p.b->timed_out(p.now));
    }
    SECTION("dead peer: timed out only after the timeout") {
        Pair p(14, {.latency_seconds = 0.02}, cfg);
        p.run_seconds(1.0);
        p.net.set_default_conditions({.loss_probability = 1.0});  // cable pulled
        p.run_seconds(1.0);
        REQUIRE_FALSE(p.a->timed_out(p.now));  // 1 s of silence < 2 s
        p.run_seconds(1.5);
        REQUIRE(p.a->timed_out(p.now));
    }
}

TEST_CASE("channel: malformed datagrams are ignored without changing state", "[reliable]") {
    Pair p(15, {});
    std::vector<std::byte> garbage(40, std::byte{0xFF});
    p.a->on_datagram(garbage, 0.0);
    std::vector<std::byte> truncated(5, std::byte{0});
    p.a->on_datagram(truncated, 0.0);
    p.a->on_datagram({}, 0.0);
    REQUIRE(p.a->stats().packets_received == 0);
    REQUIRE(p.a->stats().packets_discarded == 3);
    ChannelMessage m;
    REQUIRE_FALSE(p.a->receive(m));

    // Still works normally afterwards.
    REQUIRE(p.b->send_reliable(bytes_of(5)));
    p.run_seconds(0.2);
    REQUIRE(drain(*p.a).size() == 1);
}

TEST_CASE("channel: same seed reproduces identical delivery and stats", "[reliable]") {
    const LinkConditions bad{.latency_seconds = 0.03, .jitter_seconds = 0.02, .loss_probability = 0.3,
                             .duplicate_probability = 0.15};
    auto run2 = [&](std::uint64_t seed) {
        Pair p(seed, bad);
        REQUIRE(exchange_all(p, 300, 200.0));
        return std::make_tuple(p.now, p.a->stats().packets_sent, p.b->stats().packets_sent,
                               p.a->stats().reliable_retransmits, p.net.stats().delivered);
    };
    REQUIRE(run2(77) == run2(77));
    REQUIRE(run2(77) != run2(78));
}

TEST_CASE("channel: stress - many seeds of a nasty link all deliver everything in order", "[reliable][stress]") {
    for (std::uint64_t seed = 100; seed < 140; ++seed) {
        Pair p(seed, {.latency_seconds = 0.04, .jitter_seconds = 0.05, .loss_probability = 0.4,
                      .duplicate_probability = 0.3});
        INFO("seed " << seed);
        REQUIRE(exchange_all(p, 150, 400.0));
    }
}
