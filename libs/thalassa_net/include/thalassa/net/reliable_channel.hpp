#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <vector>

#include "thalassa/net/transport.hpp"

// ReliableChannel: reliability on top of an unreliable ITransport, for ONE
// remote peer. It gives the layers above two kinds of messages:
//   - RELIABLE: delivered exactly once, in the order sent, even if datagrams
//     are lost, duplicated or reordered (player commands, the authoritative
//     command stream, handshake messages).
//   - UNRELIABLE: fire and forget, never retransmitted, delivered at most
//     once (snapshots/deltas, where a newer one supersedes a lost one).
//
// How it works (the classic "ack bitfield" scheme):
//   - every datagram carries a packet sequence number plus an ack of the
//     newest packet received from the peer and a 32-bit mask of the 32
//     before it, so acks ride along on normal traffic and survive losing
//     individual ack packets
//   - each reliable message has its own id; the sender keeps it and resends
//     it (with exponential backoff, based on a measured RTT) until a packet
//     that carried it is acked
//   - the receiver delivers reliable messages strictly in id order, buffers
//     early ones, and drops duplicates
//   - duplicate/ancient datagrams are discarded by packet sequence number
//   - an idle channel sends a small keepalive so the peer can detect a dead
//     connection (timed_out)
//
// Time is passed in explicitly (seconds, any monotonic origin), never read
// from a clock, so the channel is deterministic and testable over
// LoopbackNetwork's simulated time.
//
// Usage per tick: feed every datagram from this peer to on_datagram(), call
// update() to flush outgoing data, then drain receive().
// Messages are limited to kMaxMessageBytes; larger payloads need fragmenting
// by a higher layer (snapshot deltas are sized for this).

namespace thalassa::net {

struct ChannelConfig {
    double initial_rto_seconds = 0.2;   // retransmit timeout before any RTT sample exists
    double min_rto_seconds = 0.05;
    double max_rto_seconds = 1.0;
    double keepalive_seconds = 0.5;     // send an empty packet if nothing was sent for this long
    double timeout_seconds = 5.0;       // timed_out() once nothing was received for this long
    std::size_t max_unacked_reliable = 256;  // send window; send_reliable() fails beyond it
};

struct ChannelStats {
    std::uint64_t packets_sent = 0;
    std::uint64_t packets_received = 0;       // accepted
    std::uint64_t packets_discarded = 0;      // duplicate, too old, or malformed
    std::uint64_t reliable_retransmits = 0;   // reliable messages sent more than once
    std::uint64_t unreliable_dropped = 0;     // did not fit in a packet
    double smoothed_rtt_seconds = 0.0;        // 0 until the first sample
};

struct ChannelMessage {
    bool reliable = false;
    std::vector<std::byte> payload;
};

class ReliableChannel {
public:
    static constexpr std::size_t kPacketHeaderBytes = 13;       // seq(4) ack(4) ack_bits(4) count(1)
    static constexpr std::size_t kReliableOverhead = 7;         // kind(1) id(4) len(2)
    static constexpr std::size_t kMaxMessageBytes = kMaxDatagramBytes - kPacketHeaderBytes - kReliableOverhead;

    // `now` = current time in seconds; starts the timeout clock.
    ReliableChannel(ITransport& transport, PeerId peer, double now, ChannelConfig config = {});

    // Queue a reliable message. False if larger than kMaxMessageBytes or the
    // send window is full (too many unacked messages: the peer is slow/gone).
    [[nodiscard]] bool send_reliable(std::span<const std::byte> payload);

    // Queue an unreliable message for the next outgoing packet. False if too large.
    [[nodiscard]] bool send_unreliable(std::span<const std::byte> payload);

    // Feed one datagram received from this peer (the caller demultiplexes by
    // Datagram::peer). Malformed datagrams are ignored.
    void on_datagram(std::span<const std::byte> datagram, double now);

    // Sends whatever is due: new messages, retransmissions, pending acks,
    // keepalive. Call every tick.
    void update(double now);

    // Pops the next delivered message (reliable ones strictly in order).
    [[nodiscard]] bool receive(ChannelMessage& out);

    // True once nothing has been received from the peer for timeout_seconds.
    [[nodiscard]] bool timed_out(double now) const noexcept {
        return now - last_receive_time_ > config_.timeout_seconds;
    }

    [[nodiscard]] std::size_t unacked_reliable_count() const noexcept { return unacked_.size(); }
    [[nodiscard]] const ChannelStats& stats() const noexcept { return stats_; }
    [[nodiscard]] PeerId peer() const noexcept { return peer_; }

private:
    struct Pending {
        std::vector<std::byte> payload;
        double last_sent = 0.0;
        std::uint32_t attempts = 0;  // times sent so far
    };
    struct SentPacket {
        double time = 0.0;
        std::vector<std::uint32_t> reliable_ids;
    };

    [[nodiscard]] double current_rto() const noexcept;
    [[nodiscard]] double retransmit_delay(const Pending& p) const noexcept;
    bool accept_packet_sequence(std::uint32_t seq);
    void process_acks(std::uint32_t ack, std::uint32_t ack_bits, double now);
    void on_packet_acked(std::uint32_t seq, double now);
    void deliver_reliable(std::uint32_t id, std::vector<std::byte> payload);

    ITransport& transport_;
    PeerId peer_;
    ChannelConfig config_;
    ChannelStats stats_;

    // Sending side
    // Packet sequence numbers start at 1: ack value 0 in a header means "nothing received yet"
    // and must never be mistaken for an ack of our own packet 0.
    std::uint32_t next_packet_seq_ = 1;
    std::uint32_t next_message_id_ = 0;
    std::map<std::uint32_t, Pending> unacked_;           // by message id
    std::map<std::uint32_t, SentPacket> sent_packets_;   // by packet seq, for ack bookkeeping
    std::deque<std::vector<std::byte>> unreliable_out_;
    double last_send_time_ = 0.0;
    bool ack_pending_ = false;
    double rtt_ = 0.0;
    bool has_rtt_ = false;

    // Receiving side
    bool has_remote_seq_ = false;
    std::uint32_t remote_seq_ = 0;   // newest packet seq received
    std::uint32_t recv_bits_ = 0;    // bit i set = packet (remote_seq_ - 1 - i) received
    std::uint32_t next_expected_id_ = 0;
    std::map<std::uint32_t, std::vector<std::byte>> reorder_buffer_;
    std::deque<ChannelMessage> incoming_;
    double last_receive_time_ = 0.0;
};

}  // namespace thalassa::net
