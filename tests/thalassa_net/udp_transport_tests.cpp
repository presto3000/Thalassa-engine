#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <catch2/catch_test_macros.hpp>

#include "thalassa/net/udp_transport.hpp"

using namespace thalassa::net;

namespace {

std::unique_ptr<UdpTransport> make(std::uint16_t port = 0, std::size_t max_peers = 4096) {
    UdpConfig cfg;
    cfg.bind_address = "127.0.0.1";
    cfg.port = port;
    cfg.max_peers = max_peers;
    std::string err;
    auto t = UdpTransport::create(cfg, &err);
    INFO(err);
    REQUIRE(t != nullptr);
    return t;
}

std::vector<std::byte> bytes(std::size_t n, std::uint8_t fill) {
    return std::vector<std::byte>(n, static_cast<std::byte>(fill));
}

// Polls up to ~1 s for one datagram.
bool wait_receive(UdpTransport& t, Datagram& out) {
    for (int i = 0; i < 1000; ++i) {
        if (t.receive(out)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

}  // namespace

TEST_CASE("udp: send and receive over 127.0.0.1, peers learned on first contact", "[net][udp]") {
    auto server = make();
    auto client = make();
    const PeerId to_server = client->add_peer("127.0.0.1", server->local_port());
    REQUIRE(to_server != kInvalidPeer);
    REQUIRE(client->add_peer("127.0.0.1", server->local_port()) == to_server);  // idempotent

    REQUIRE(client->send(to_server, bytes(100, 0xAB)));
    Datagram d;
    REQUIRE(wait_receive(*server, d));
    CHECK(d.payload == bytes(100, 0xAB));
    CHECK(d.peer != kInvalidPeer);
    CHECK(server->peer_name(d.peer) == "127.0.0.1:" + std::to_string(client->local_port()));

    // The server answers to the id it was given.
    REQUIRE(server->send(d.peer, bytes(7, 0x11)));
    Datagram r;
    REQUIRE(wait_receive(*client, r));
    CHECK(r.peer == to_server);
    CHECK(r.payload == bytes(7, 0x11));
    CHECK(server->stats().datagrams_received == 1);
    CHECK(client->stats().bytes_sent == 100);
}

TEST_CASE("udp: receive on an empty socket returns false immediately", "[net][udp]") {
    auto t = make();
    Datagram d;
    CHECK_FALSE(t->receive(d));
}

TEST_CASE("udp: send rejects oversize payloads and unknown peers", "[net][udp]") {
    auto a = make();
    auto b = make();
    const PeerId p = a->add_peer("127.0.0.1", b->local_port());
    CHECK(a->send(p, bytes(kMaxDatagramBytes, 1)));
    CHECK_FALSE(a->send(p, bytes(kMaxDatagramBytes + 1, 1)));
    CHECK_FALSE(a->send(12345, bytes(4, 1)));
    CHECK_FALSE(a->add_peer("256.256.256.256", 1) != kInvalidPeer);
}

TEST_CASE("udp: oversize and empty incoming datagrams are dropped", "[net][udp]") {
    auto server = make();
    const int raw = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(raw >= 0);
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(server->local_port());
    ::inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
    std::vector<char> big(1500, 'x');
    char one = 'k';
    (void)::sendto(raw, big.data(), big.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    (void)::sendto(raw, &one, 0, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    (void)::sendto(raw, &one, 1, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    ::close(raw);

    Datagram d;
    REQUIRE(wait_receive(*server, d));  // skips the two bad ones, delivers the 1-byte one
    CHECK(d.payload.size() == 1);
    CHECK(server->stats().dropped_oversize == 2);
}

TEST_CASE("udp: peer table is capped by evicting the longest-silent peer", "[net][udp]") {
    auto server = make(0, 3);
    std::vector<std::unique_ptr<UdpTransport>> clients;
    for (int i = 0; i < 5; ++i) {
        clients.push_back(make());
        const PeerId p = clients.back()->add_peer("127.0.0.1", server->local_port());
        REQUIRE(clients.back()->send(p, bytes(1, static_cast<std::uint8_t>(i))));
        Datagram d;
        REQUIRE(wait_receive(*server, d));
        CHECK(server->peer_count() <= 3);
    }
    CHECK(server->stats().peers_evicted == 2);
}

TEST_CASE("udp: bind failure reports an error", "[net][udp]") {
    auto a = make();
    UdpConfig cfg;
    cfg.bind_address = "127.0.0.1";
    cfg.port = a->local_port();  // already taken
    std::string err;
    CHECK(UdpTransport::create(cfg, &err) == nullptr);
    CHECK_FALSE(err.empty());

    cfg.bind_address = "not an address";
    cfg.port = 0;
    CHECK(UdpTransport::create(cfg, &err) == nullptr);
}
