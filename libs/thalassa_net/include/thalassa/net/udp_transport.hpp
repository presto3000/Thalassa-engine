#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "thalassa/net/transport.hpp"

// UdpTransport: ITransport over a real, non-blocking UDP socket (IPv4,
// POSIX: Linux/macOS). Same semantics as the loopback: unreliable, unordered,
// polled from the owner's tick, payload <= kMaxDatagramBytes.
//
// Peers. ITransport addresses peers by PeerId, so this class keeps a table
// PeerId <-> (ip, port):
//   - a CLIENT calls add_peer("host", port) once to get the server's PeerId;
//   - a SERVER just receives: the first datagram from an unknown address
//     registers it under a fresh PeerId, and replies use that id.
// PeerId 0 is this endpoint itself (local_id()); remote ids start at 1.
//
// Untrusted input: the source address of a UDP datagram can be forged, so the
// peer table is capped (max_peers); when full, the peer that has been silent
// longest is evicted to make room. Datagrams larger than kMaxDatagramBytes and
// empty ones are dropped. Authentication / anti-spoofing beyond that belongs to
// the session layer, not here.
//
// No threads, no callbacks; the destructor closes the socket.

namespace thalassa::net {

struct UdpConfig {
    std::string bind_address = "0.0.0.0";  // "127.0.0.1" for local-only
    std::uint16_t port = 0;                // 0 = let the OS pick (clients)
    std::size_t max_peers = 4096;
};

struct UdpStats {
    std::uint64_t datagrams_sent = 0;
    std::uint64_t datagrams_received = 0;
    std::uint64_t bytes_sent = 0;
    std::uint64_t bytes_received = 0;
    std::uint64_t send_failures = 0;     // OS refused (buffer full, unreachable...)
    std::uint64_t dropped_oversize = 0;  // too big or empty on receive
    std::uint64_t peers_evicted = 0;
};

class UdpTransport final : public ITransport {
public:
    // Creates and binds the socket. Returns nullptr on failure and, if given,
    // fills *error with a human-readable reason.
    [[nodiscard]] static std::unique_ptr<UdpTransport> create(const UdpConfig& config, std::string* error = nullptr);

    ~UdpTransport() override;
    UdpTransport(const UdpTransport&) = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;

    // Registers a remote address (numeric IPv4 or a hostname) and returns its
    // PeerId, or kInvalidPeer if it cannot be resolved. Adding the same
    // address twice returns the same id.
    [[nodiscard]] PeerId add_peer(const std::string& host, std::uint16_t port);

    // The port actually bound (useful with port = 0).
    [[nodiscard]] std::uint16_t local_port() const noexcept { return local_port_; }

    // Remote address of a known peer as "ip:port" (for logs); empty if unknown.
    [[nodiscard]] std::string peer_name(PeerId peer) const;

    [[nodiscard]] std::size_t peer_count() const noexcept { return by_id_.size(); }
    [[nodiscard]] const UdpStats& stats() const noexcept { return stats_; }

    [[nodiscard]] PeerId local_id() const noexcept override { return 0; }
    [[nodiscard]] bool send(PeerId to, std::span<const std::byte> payload) override;
    [[nodiscard]] bool receive(Datagram& out) override;

private:
    struct Address {
        std::uint32_t ip = 0;    // network byte order
        std::uint16_t port = 0;  // network byte order
        bool operator==(const Address&) const = default;
    };
    struct PeerEntry {
        Address address;
        std::uint64_t last_seen = 0;
    };

    UdpTransport() = default;
    PeerId register_peer(const Address& address);

    int fd_ = -1; // descriptor of the bound UDP socket (IPv4, non-blocking)
    std::uint16_t local_port_ = 0;
    std::size_t max_peers_ = 4096;
    PeerId next_peer_ = 1;
    std::uint64_t clock_ = 0;  // monotonically increasing activity counter
    std::unordered_map<std::uint64_t, PeerId> by_address_;  // key = ip<<16 | port
    std::unordered_map<PeerId, PeerEntry> by_id_;
    std::vector<std::byte> recv_buffer_ = std::vector<std::byte>(2048);
    UdpStats stats_;
};

}  // namespace thalassa::net
