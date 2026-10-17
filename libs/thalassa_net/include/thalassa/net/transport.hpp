#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Transport: the one seam between Thalassa's networking logic and the actual
// wire. Everything above it (reliability, sessions, PredictedClient/MatchHost
// plumbing) talks only to ITransport, so the same code runs over:
//   - LoopbackNetwork (in-process, deterministic, with simulated latency /
//     jitter / loss / duplication) for automated tests, and
//   - a UDP implementation for real play (later part).
// An Unreal adapter over FSocket can implement the same interface.
//
// Semantics are deliberately those of UDP: datagrams are unreliable,
// unordered, may be duplicated, and have a size limit. Reliability lives in a
// layer ABOVE this interface, never inside it. Polling, no threads, no
// callbacks: the owner calls receive() from its own tick.

namespace thalassa::net {

// Identifies the other side of a datagram. For loopback it is the endpoint
// id; a UDP transport maps it to a socket address internally.
using PeerId = std::uint32_t;
inline constexpr PeerId kInvalidPeer = 0xFFFFFFFFu;

// Conservative payload limit that fits a typical 1500-byte Ethernet MTU after
// IP/UDP headers and tunnelling overhead. Larger messages must be fragmented
// by a higher layer (snapshots are delta-compressed for this reason).
inline constexpr std::size_t kMaxDatagramBytes = 1200;

struct Datagram {
    PeerId peer = kInvalidPeer;  // who it came from (receive) / goes to (send)
    std::vector<std::byte> payload;
};

class ITransport {
public:
    virtual ~ITransport() = default;

    // This endpoint's own id (what other peers use to address it).
    [[nodiscard]] virtual PeerId local_id() const noexcept = 0;

    // Queues one datagram for `to`. Returns false if it was rejected right
    // away (payload larger than kMaxDatagramBytes, unknown peer). A true
    // return does NOT mean it will arrive: delivery is unreliable.
    [[nodiscard]] virtual bool send(PeerId to, std::span<const std::byte> payload) = 0;

    // Pops the next received datagram into `out`. Returns false if none is
    // waiting. Call in a loop until it returns false.
    [[nodiscard]] virtual bool receive(Datagram& out) = 0;
};

}  // namespace thalassa::net
