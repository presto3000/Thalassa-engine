#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <vector>

#include "thalassa/core/random.hpp"
#include "thalassa/net/transport.hpp"

// LoopbackNetwork: an in-process "internet" for tests and local development.
// It hands out ITransport endpoints and moves datagrams between them with
// configurable bad-network behaviour, entirely deterministically:
//   - time is SIMULATED: nothing is delivered until the test calls
//     advance(seconds); no wall clock, no threads
//   - every random decision (loss, duplicate, jitter) comes from one seeded
//     Rng, so the same seed + same traffic reproduces the exact same
//     delivery sequence, which makes network bugs replayable
// This is what lets us prove the reliability layer and prediction/reconcile
// code survive packet loss, reordering and duplication without a real network.

namespace thalassa::net {

// Behaviour of one direction of a link. Defaults = a perfect instant link.
struct LinkConditions {
    double latency_seconds = 0.0;            // one-way base delay
    double jitter_seconds = 0.0;             // +/- uniform extra delay; also causes reordering
    double loss_probability = 0.0;           // [0,1] datagram silently dropped
    double duplicate_probability = 0.0;      // [0,1] datagram delivered twice
};

struct NetworkStats {
    std::uint64_t sent = 0;        // accepted by send()
    std::uint64_t rejected = 0;    // refused by send() (too big / unknown peer)
    std::uint64_t dropped = 0;     // lost by the simulated link
    std::uint64_t duplicated = 0;  // extra copies injected
    std::uint64_t delivered = 0;   // moved into a receive queue
};

class LoopbackTransport;

class LoopbackNetwork {
public:
    explicit LoopbackNetwork(std::uint64_t seed = 1) : rng_(seed) {}

    LoopbackNetwork(const LoopbackNetwork&) = delete;
    LoopbackNetwork& operator=(const LoopbackNetwork&) = delete;

    // Creates a new endpoint. Ids are assigned 0, 1, 2... in creation order.
    // The network must outlive every endpoint it created.
    [[nodiscard]] std::unique_ptr<LoopbackTransport> create_endpoint();

    // Conditions applied to every link that has no specific override.
    void set_default_conditions(const LinkConditions& conditions) { default_conditions_ = conditions; }

    // Overrides one direction (from -> to), e.g. a lossy uplink with a clean downlink.
    void set_link_conditions(PeerId from, PeerId to, const LinkConditions& conditions) {
        overrides_[key(from, to)] = conditions;
    }

    // Advances simulated time and delivers every datagram that is due.
    void advance(double seconds);

    [[nodiscard]] double now() const noexcept { return now_; }
    [[nodiscard]] const NetworkStats& stats() const noexcept { return stats_; }

private:
    friend class LoopbackTransport;

    struct InFlight {
        double deliver_at = 0.0;
        std::uint64_t sequence = 0;  // tie-break so equal times keep send order
        PeerId from = kInvalidPeer;
        PeerId to = kInvalidPeer;
        std::vector<std::byte> payload;
    };

    static std::uint64_t key(PeerId from, PeerId to) noexcept {
        return (static_cast<std::uint64_t>(from) << 32) | to;
    }
    [[nodiscard]] const LinkConditions& conditions_for(PeerId from, PeerId to) const;
    [[nodiscard]] bool submit(PeerId from, PeerId to, std::span<const std::byte> payload);
    void enqueue(const LinkConditions& c, PeerId from, PeerId to, std::span<const std::byte> payload, double extra_delay);

    core::Rng rng_;
    double now_ = 0.0;
    std::uint64_t next_sequence_ = 0;
    LinkConditions default_conditions_;
    std::map<std::uint64_t, LinkConditions> overrides_;
    std::vector<InFlight> in_flight_;
    std::vector<std::deque<Datagram>> inboxes_;  // indexed by PeerId
    NetworkStats stats_;
};

// One endpoint of a LoopbackNetwork. Implements ITransport.
class LoopbackTransport final : public ITransport {
public:
    [[nodiscard]] PeerId local_id() const noexcept override { return id_; }
    [[nodiscard]] bool send(PeerId to, std::span<const std::byte> payload) override {
        return network_->submit(id_, to, payload);
    }
    [[nodiscard]] bool receive(Datagram& out) override;

private:
    friend class LoopbackNetwork;
    LoopbackTransport(LoopbackNetwork* network, PeerId id) : network_(network), id_(id) {}

    LoopbackNetwork* network_;
    PeerId id_;
};

}  // namespace thalassa::net
