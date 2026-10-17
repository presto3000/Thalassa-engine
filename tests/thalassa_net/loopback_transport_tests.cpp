#include <cstdint>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/loopback_transport.hpp"

using thalassa::net::Datagram;
using thalassa::net::kMaxDatagramBytes;
using thalassa::net::LinkConditions;
using thalassa::net::LoopbackNetwork;

namespace {

std::vector<std::byte> bytes_of(std::uint32_t value) {
    std::vector<std::byte> b(4);
    for (int i = 0; i < 4; ++i) b[static_cast<std::size_t>(i)] = static_cast<std::byte>((value >> (8 * i)) & 0xFFu);
    return b;
}

std::uint32_t value_of(const Datagram& d) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(d.payload[static_cast<std::size_t>(i)]) << (8 * i);
    return v;
}

std::vector<std::uint32_t> drain(thalassa::net::ITransport& t) {
    std::vector<std::uint32_t> out;
    Datagram d;
    while (t.receive(d)) out.push_back(value_of(d));
    return out;
}

// Sends `count` numbered datagrams a->b, advancing 1 ms between sends, then lets everything land.
std::vector<std::uint32_t> run_traffic(std::uint64_t seed, const LinkConditions& c, std::uint32_t count) {
    LoopbackNetwork net(seed);
    net.set_default_conditions(c);
    auto a = net.create_endpoint();
    auto b = net.create_endpoint();
    std::vector<std::uint32_t> received;
    for (std::uint32_t i = 0; i < count; ++i) {
        REQUIRE(a->send(b->local_id(), bytes_of(i)));
        net.advance(0.001);
        auto got = drain(*b);
        received.insert(received.end(), got.begin(), got.end());
    }
    net.advance(10.0);
    auto got = drain(*b);
    received.insert(received.end(), got.begin(), got.end());
    return received;
}

}  // namespace

TEST_CASE("loopback: endpoints get sequential ids and datagrams carry the sender", "[loopback]") {
    LoopbackNetwork net;
    auto a = net.create_endpoint();
    auto b = net.create_endpoint();
    REQUIRE(a->local_id() == 0);
    REQUIRE(b->local_id() == 1);

    REQUIRE(a->send(b->local_id(), bytes_of(42)));
    net.advance(0.0);

    Datagram d;
    REQUIRE(b->receive(d));
    REQUIRE(d.peer == a->local_id());
    REQUIRE(value_of(d) == 42);
    REQUIRE_FALSE(b->receive(d));
}

TEST_CASE("loopback: nothing arrives before the latency has elapsed", "[loopback]") {
    LoopbackNetwork net;
    net.set_default_conditions({.latency_seconds = 0.1});
    auto a = net.create_endpoint();
    auto b = net.create_endpoint();

    REQUIRE(a->send(b->local_id(), bytes_of(1)));
    net.advance(0.05);
    Datagram d;
    REQUIRE_FALSE(b->receive(d));
    net.advance(0.06);  // total 0.11 s >= 0.1 s
    REQUIRE(b->receive(d));
}

TEST_CASE("loopback: a perfect link preserves order and delivers everything exactly once", "[loopback]") {
    const auto got = run_traffic(1, {}, 500);
    REQUIRE(got.size() == 500);
    for (std::uint32_t i = 0; i < 500; ++i) REQUIRE(got[i] == i);
}

TEST_CASE("loopback: loss drops roughly the configured fraction and never invents data", "[loopback]") {
    const auto got = run_traffic(7, {.loss_probability = 0.3}, 10000);
    REQUIRE(got.size() > 6500);
    REQUIRE(got.size() < 7500);  // ~7000 expected
    // Survivors are still in order on a jitter-free link.
    for (std::size_t i = 1; i < got.size(); ++i) REQUIRE(got[i] > got[i - 1]);
}

TEST_CASE("loopback: jitter reorders datagrams but loses none", "[loopback]") {
    const auto got = run_traffic(3, {.latency_seconds = 0.05, .jitter_seconds = 0.04}, 1000);
    REQUIRE(got.size() == 1000);
    bool reordered = false;
    for (std::size_t i = 1; i < got.size(); ++i) reordered |= got[i] < got[i - 1];
    REQUIRE(reordered);

    std::vector<bool> seen(1000, false);
    for (const auto v : got) {
        REQUIRE(v < 1000);
        REQUIRE_FALSE(seen[v]);  // exactly once
        seen[v] = true;
    }
}

TEST_CASE("loopback: duplication delivers extra copies", "[loopback]") {
    const auto got = run_traffic(5, {.duplicate_probability = 0.5}, 2000);
    REQUIRE(got.size() > 2700);
    REQUIRE(got.size() < 3300);  // ~3000 expected
}

TEST_CASE("loopback: same seed reproduces the exact delivery sequence, different seed does not", "[loopback]") {
    const LinkConditions bad{.latency_seconds = 0.03, .jitter_seconds = 0.03, .loss_probability = 0.2,
                             .duplicate_probability = 0.1};
    const auto first = run_traffic(11, bad, 2000);
    const auto second = run_traffic(11, bad, 2000);
    const auto other = run_traffic(12, bad, 2000);
    REQUIRE(first == second);
    REQUIRE(first != other);
}

TEST_CASE("loopback: links are directional (lossy uplink, clean downlink)", "[loopback]") {
    LoopbackNetwork net(9);
    auto client = net.create_endpoint();
    auto server = net.create_endpoint();
    net.set_link_conditions(client->local_id(), server->local_id(), {.loss_probability = 1.0});

    for (std::uint32_t i = 0; i < 50; ++i) {
        REQUIRE(client->send(server->local_id(), bytes_of(i)));  // accepted, then lost
        REQUIRE(server->send(client->local_id(), bytes_of(i)));
    }
    net.advance(1.0);
    REQUIRE(drain(*server).empty());
    REQUIRE(drain(*client).size() == 50);
    REQUIRE(net.stats().dropped == 50);
}

TEST_CASE("loopback: oversize payloads and unknown peers are rejected immediately", "[loopback]") {
    LoopbackNetwork net;
    auto a = net.create_endpoint();
    auto b = net.create_endpoint();

    std::vector<std::byte> ok(kMaxDatagramBytes);
    std::vector<std::byte> too_big(kMaxDatagramBytes + 1);
    REQUIRE(a->send(b->local_id(), ok));
    REQUIRE_FALSE(a->send(b->local_id(), too_big));
    REQUIRE_FALSE(a->send(99, bytes_of(1)));
    REQUIRE(net.stats().rejected == 2);
    REQUIRE(net.stats().sent == 1);
}

TEST_CASE("loopback: a large burst through a bad link keeps the stats consistent", "[loopback]") {
    LoopbackNetwork net(21);
    net.set_default_conditions({.latency_seconds = 0.02, .jitter_seconds = 0.01, .loss_probability = 0.1,
                               .duplicate_probability = 0.05});
    auto a = net.create_endpoint();
    auto b = net.create_endpoint();
    for (std::uint32_t i = 0; i < 5000; ++i) REQUIRE(a->send(b->local_id(), bytes_of(i)));
    net.advance(5.0);

    const auto got = drain(*b);
    const auto& s = net.stats();
    REQUIRE(s.sent == 5000);
    // Every accepted datagram is either dropped or delivered (+ duplicates of the delivered ones).
    REQUIRE(s.delivered == got.size());
    REQUIRE(s.delivered == s.sent - s.dropped + s.duplicated);
}
