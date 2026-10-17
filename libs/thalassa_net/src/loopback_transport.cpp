#include "thalassa/net/loopback_transport.hpp"

#include <algorithm>

namespace thalassa::net {

std::unique_ptr<LoopbackTransport> LoopbackNetwork::create_endpoint() {
    const auto id = static_cast<PeerId>(inboxes_.size());
    inboxes_.emplace_back();
    return std::unique_ptr<LoopbackTransport>(new LoopbackTransport(this, id));
}

const LinkConditions& LoopbackNetwork::conditions_for(PeerId from, PeerId to) const {
    const auto it = overrides_.find(key(from, to));
    return it == overrides_.end() ? default_conditions_ : it->second;
}

bool LoopbackNetwork::submit(PeerId from, PeerId to, std::span<const std::byte> payload) {
    if (payload.size() > kMaxDatagramBytes || to >= inboxes_.size()) {
        ++stats_.rejected;
        return false;
    }
    ++stats_.sent;

    const LinkConditions& c = conditions_for(from, to);

    if (c.loss_probability > 0.0 && rng_.next_scalar01() < c.loss_probability) {
        ++stats_.dropped;
        return true;  // accepted by the "network", then lost: exactly like UDP
    }

    enqueue(c, from, to, payload, 0.0);

    if (c.duplicate_probability > 0.0 && rng_.next_scalar01() < c.duplicate_probability) {
        ++stats_.duplicated;
        // The copy takes its own jitter draw, so it may arrive before or after the original.
        enqueue(c, from, to, payload, 0.0);
    }
    return true;
}

void LoopbackNetwork::enqueue(const LinkConditions& c, PeerId from, PeerId to, std::span<const std::byte> payload,
                              double extra_delay) {
    double delay = c.latency_seconds + extra_delay;
    if (c.jitter_seconds > 0.0) {
        delay += (rng_.next_scalar01() * 2.0 - 1.0) * c.jitter_seconds;
    }
    delay = std::max(delay, 0.0);

    InFlight packet;
    packet.deliver_at = now_ + delay;
    packet.sequence = next_sequence_++;
    packet.from = from;
    packet.to = to;
    packet.payload.assign(payload.begin(), payload.end());
    in_flight_.push_back(std::move(packet));
}

void LoopbackNetwork::advance(double seconds) {
    now_ += seconds;

    // Zero-latency links deliver on the next advance() (even advance(0)), never inside send().
    std::vector<InFlight> due;
    std::vector<InFlight> remaining;
    for (InFlight& p : in_flight_) {
        (p.deliver_at <= now_ ? due : remaining).push_back(std::move(p));
    }
    in_flight_ = std::move(remaining);

    // Deterministic delivery order: by arrival time, then send order.
    std::sort(due.begin(), due.end(), [](const InFlight& a, const InFlight& b) {
        return a.deliver_at != b.deliver_at ? a.deliver_at < b.deliver_at : a.sequence < b.sequence;
    });
    for (InFlight& p : due) {
        inboxes_[p.to].push_back(Datagram{p.from, std::move(p.payload)});
        ++stats_.delivered;
    }
}

bool LoopbackTransport::receive(Datagram& out) {
    auto& inbox = network_->inboxes_[id_];
    if (inbox.empty()) {
        return false;
    }
    out = std::move(inbox.front());
    inbox.pop_front();
    return true;
}

}  // namespace thalassa::net
