#include "thalassa/net/reliable_channel.hpp"

#include <algorithm>

namespace thalassa::net {

namespace {

constexpr std::uint8_t kKindUnreliable = 0;
constexpr std::uint8_t kKindReliable = 1;
constexpr std::size_t kMaxPacketsPerUpdate = 16;
constexpr std::uint32_t kBackoffCap = 3;  // retransmit delay grows up to 8x RTO

void put_u8(std::vector<std::byte>& b, std::uint8_t v) { b.push_back(static_cast<std::byte>(v)); }
void put_u16(std::vector<std::byte>& b, std::uint16_t v) {
    put_u8(b, static_cast<std::uint8_t>(v & 0xFFu));
    put_u8(b, static_cast<std::uint8_t>(v >> 8));
}
void put_u32(std::vector<std::byte>& b, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) put_u8(b, static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
}

// Bounds-checked little-endian reader; any overrun flips ok() to false.
class Reader {
public:
    explicit Reader(std::span<const std::byte> data) : data_(data) {}
    [[nodiscard]] bool ok() const noexcept { return ok_; }
    std::uint8_t u8() {
        if (pos_ + 1 > data_.size()) { ok_ = false; return 0; }
        return static_cast<std::uint8_t>(data_[pos_++]);
    }
    std::uint16_t u16() {
        const std::uint16_t lo = u8();
        const std::uint16_t hi = u8();
        return static_cast<std::uint16_t>(lo | (hi << 8));
    }
    std::uint32_t u32() {
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(u8()) << (8 * i);
        return v;
    }
    std::span<const std::byte> bytes(std::size_t n) {
        if (pos_ + n > data_.size()) { ok_ = false; return {}; }
        const auto s = data_.subspan(pos_, n);
        pos_ += n;
        return s;
    }

private:
    std::span<const std::byte> data_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

std::int32_t seq_diff(std::uint32_t a, std::uint32_t b) { return static_cast<std::int32_t>(a - b); }

}  // namespace

ReliableChannel::ReliableChannel(ITransport& transport, PeerId peer, double now, ChannelConfig config)
    : transport_(transport), peer_(peer), config_(config), last_send_time_(now), last_receive_time_(now) {}

bool ReliableChannel::send_reliable(std::span<const std::byte> payload) {
    if (payload.size() > kMaxMessageBytes) return false;
    if (!unacked_.empty() && next_message_id_ - unacked_.begin()->first >= config_.max_unacked_reliable) {
        return false;  // window full
    }
    Pending p;
    p.payload.assign(payload.begin(), payload.end());
    p.attempts = 0;
    unacked_.emplace(next_message_id_++, std::move(p));
    return true;
}

bool ReliableChannel::send_unreliable(std::span<const std::byte> payload) {
    if (payload.size() > kMaxMessageBytes) return false;
    unreliable_out_.emplace_back(payload.begin(), payload.end());
    return true;
}

double ReliableChannel::current_rto() const noexcept {
    if (!has_rtt_) return config_.initial_rto_seconds;
    return std::clamp(rtt_ * 2.0, config_.min_rto_seconds, config_.max_rto_seconds);
}

double ReliableChannel::retransmit_delay(const Pending& p) const noexcept {
    const std::uint32_t shift = std::min(p.attempts > 0 ? p.attempts - 1 : 0u, kBackoffCap);
    return std::min(current_rto() * static_cast<double>(1u << shift), config_.max_rto_seconds);
}

void ReliableChannel::update(double now) {
    for (std::size_t round = 0; round < kMaxPacketsPerUpdate; ++round) {
        // Reliable messages that are new or whose retransmit timer expired.
        std::vector<std::uint32_t> due;
        for (const auto& [id, p] : unacked_) {
            if (p.attempts == 0 || now - p.last_sent >= retransmit_delay(p)) due.push_back(id);
        }
        const bool keepalive_due = now - last_send_time_ >= config_.keepalive_seconds;
        if (due.empty() && unreliable_out_.empty() && !ack_pending_ && !keepalive_due) {
            break;
        }

        std::vector<std::byte> packet;
        packet.reserve(kMaxDatagramBytes);
        const std::uint32_t seq = next_packet_seq_++;
        put_u32(packet, seq);
        put_u32(packet, has_remote_seq_ ? remote_seq_ : 0u);
        put_u32(packet, recv_bits_);
        put_u8(packet, 0);  // message count, patched below

        std::uint32_t count = 0;
        SentPacket record;
        record.time = now;
        bool all_due_fit = true;

        for (const std::uint32_t id : due) {
            Pending& p = unacked_.at(id);
            if (packet.size() + kReliableOverhead + p.payload.size() > kMaxDatagramBytes || count == 255) {
                all_due_fit = false;
                break;
            }
            put_u8(packet, kKindReliable);
            put_u32(packet, id);
            put_u16(packet, static_cast<std::uint16_t>(p.payload.size()));
            packet.insert(packet.end(), p.payload.begin(), p.payload.end());
            if (p.attempts > 0) ++stats_.reliable_retransmits;
            ++p.attempts;
            p.last_sent = now;
            record.reliable_ids.push_back(id);
            ++count;
        }
        while (!unreliable_out_.empty()) {
            const auto& m = unreliable_out_.front();
            if (packet.size() + 3 + m.size() > kMaxDatagramBytes || count == 255) {
                // Does not fit this packet. If the packet already carries
                // something, leave it for the next round; otherwise it can
                // never fit, so drop it.
                if (count == 0) { unreliable_out_.pop_front(); ++stats_.unreliable_dropped; continue; }
                all_due_fit = false;
                break;
            }
            put_u8(packet, kKindUnreliable);
            put_u16(packet, static_cast<std::uint16_t>(m.size()));
            packet.insert(packet.end(), m.begin(), m.end());
            unreliable_out_.pop_front();
            ++count;
        }
        packet[12] = static_cast<std::byte>(count);

        (void)transport_.send(peer_, packet);  // unreliable by contract; loss is handled by retransmit
        ++stats_.packets_sent;
        last_send_time_ = now;
        ack_pending_ = false;
        sent_packets_.emplace(seq, std::move(record));

        if (all_due_fit) break;
    }
}

bool ReliableChannel::accept_packet_sequence(std::uint32_t seq) {
    if (!has_remote_seq_) {
        has_remote_seq_ = true;
        remote_seq_ = seq;
        recv_bits_ = 0;
        return true;
    }
    const std::int32_t diff = seq_diff(seq, remote_seq_);
    if (diff > 0) {
        recv_bits_ = diff >= 32 ? 0u : (recv_bits_ << diff);
        if (diff <= 32) recv_bits_ |= (1u << (diff - 1));  // the previous newest becomes "diff-1 behind"
        remote_seq_ = seq;
        return true;
    }
    if (diff == 0) return false;  // duplicate of the newest
    const std::uint32_t back = static_cast<std::uint32_t>(-diff);
    if (back > 32) return false;  // too old to track
    const std::uint32_t mask = 1u << (back - 1);
    if ((recv_bits_ & mask) != 0) return false;  // duplicate
    recv_bits_ |= mask;
    return true;
}

void ReliableChannel::on_packet_acked(std::uint32_t seq, double now) {
    const auto it = sent_packets_.find(seq);
    if (it == sent_packets_.end()) return;  // already handled, or forgotten
    const double sample = now - it->second.time;
    if (sample >= 0.0) {
        rtt_ = has_rtt_ ? rtt_ * 0.875 + sample * 0.125 : sample;
        has_rtt_ = true;
        stats_.smoothed_rtt_seconds = rtt_;
    }
    for (const std::uint32_t id : it->second.reliable_ids) unacked_.erase(id);
    sent_packets_.erase(it);
}

void ReliableChannel::process_acks(std::uint32_t ack, std::uint32_t ack_bits, double now) {
    on_packet_acked(ack, now);
    for (std::uint32_t i = 0; i < 32; ++i) {
        if ((ack_bits & (1u << i)) != 0) on_packet_acked(ack - 1u - i, now);
    }
    // Forget bookkeeping for packets far older than anything the peer can still ack.
    while (!sent_packets_.empty() && seq_diff(ack, sent_packets_.begin()->first) > 64) {
        sent_packets_.erase(sent_packets_.begin());
    }
}

void ReliableChannel::deliver_reliable(std::uint32_t id, std::vector<std::byte> payload) {
    if (seq_diff(id, next_expected_id_) < 0) return;  // already delivered: duplicate
    if (id - next_expected_id_ >= config_.max_unacked_reliable * 2) return;  // absurdly far ahead: ignore
    reorder_buffer_.emplace(id, std::move(payload));  // emplace keeps the first copy of a duplicate
    for (auto it = reorder_buffer_.find(next_expected_id_); it != reorder_buffer_.end();
         it = reorder_buffer_.find(next_expected_id_)) {
        incoming_.push_back(ChannelMessage{true, std::move(it->second)});
        reorder_buffer_.erase(it);
        ++next_expected_id_;
    }
}

void ReliableChannel::on_datagram(std::span<const std::byte> datagram, double now) {
    Reader r(datagram);
    const std::uint32_t seq = r.u32();
    const std::uint32_t ack = r.u32();
    const std::uint32_t ack_bits = r.u32();
    const std::uint8_t count = r.u8();
    if (!r.ok()) { ++stats_.packets_discarded; return; }

    // Parse everything first so a malformed packet changes no state at all.
    struct Parsed { std::uint8_t kind; std::uint32_t id; std::span<const std::byte> data; };
    std::vector<Parsed> messages;
    messages.reserve(count);
    for (std::uint8_t i = 0; i < count; ++i) {
        Parsed m{};
        m.kind = r.u8();
        if (m.kind == kKindReliable) m.id = r.u32();
        else if (m.kind != kKindUnreliable) { ++stats_.packets_discarded; return; }
        const std::uint16_t len = r.u16();
        m.data = r.bytes(len);
        if (!r.ok()) { ++stats_.packets_discarded; return; }
        messages.push_back(m);
    }

    if (!accept_packet_sequence(seq)) { ++stats_.packets_discarded; return; }
    ++stats_.packets_received;
    last_receive_time_ = now;

    process_acks(ack, ack_bits, now);

    for (const Parsed& m : messages) {
        std::vector<std::byte> payload(m.data.begin(), m.data.end());
        if (m.kind == kKindReliable) deliver_reliable(m.id, std::move(payload));
        else incoming_.push_back(ChannelMessage{false, std::move(payload)});
    }
    // Only packets that carry messages need an answer; acks for ack-only packets would ping-pong forever.
    if (count > 0) ack_pending_ = true;
}

bool ReliableChannel::receive(ChannelMessage& out) {
    if (incoming_.empty()) return false;
    out = std::move(incoming_.front());
    incoming_.pop_front();
    return true;
}

}  // namespace thalassa::net
