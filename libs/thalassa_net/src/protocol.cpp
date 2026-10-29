#include "thalassa/net/protocol.hpp"

#include <cmath>

namespace thalassa::net {

namespace {

bool finite(double v) { return std::isfinite(v); }

bool finite(const core::math::Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }

void write_vec3(WireWriter& w, const core::math::Vec3& v) {
    w.f64(v.x);
    w.f64(v.y);
    w.f64(v.z);
}

core::math::Vec3 read_vec3(WireReader& r) {
    core::math::Vec3 v;
    v.x = r.f64();
    v.y = r.f64();
    v.z = r.f64();
    return v;
}

// A decoded message must consume the whole buffer: trailing bytes mean it is not ours.
template <typename T>
std::optional<T> finish(const WireReader& r, T value) {
    if (!r.ok() || r.remaining() != 0) return std::nullopt;
    return value;
}

bool expect_type(WireReader& r, MessageType t) { return r.u8() == static_cast<std::uint8_t>(t); }

}  // namespace

void write_command(WireWriter& w, const sim::Command& c) {
    w.u8(static_cast<std::uint8_t>(c.type));
    w.u32(c.entity.index());
    w.u32(c.entity.generation());
    write_vec3(w, c.position);
    w.f64(c.speed);
    w.u32(c.owner.value());
    w.i32(c.team);
    w.f64(c.max_health);
    w.f64(c.attack_damage);
    w.f64(c.attack_range);
    w.f64(c.attack_cooldown_seconds);
    w.i32(c.attack_kind);
    w.f64(c.attack_windup_seconds);
    w.f64(c.projectile_speed);
    w.u32(c.unit_type);
}

bool read_command(WireReader& r, sim::Command& out) {
    sim::Command c;
    const std::uint8_t type = r.u8();
    if (type > static_cast<std::uint8_t>(sim::Command::Type::MoveTo)) return false;
    c.type = static_cast<sim::Command::Type>(type);
    const std::uint32_t index = r.u32();
    const std::uint32_t generation = r.u32();
    c.entity = core::ecs::Entity(index, generation);
    c.position = read_vec3(r);
    c.speed = r.f64();
    c.owner = core::PlayerId{r.u32()};
    c.team = r.i32();
    c.max_health = r.f64();
    c.attack_damage = r.f64();
    c.attack_range = r.f64();
    c.attack_cooldown_seconds = r.f64();
    c.attack_kind = r.i32();
    c.attack_windup_seconds = r.f64();
    c.projectile_speed = r.f64();
    c.unit_type = r.u32();
    if (!r.ok()) return false;

    // Untrusted numbers: NaN/inf would poison the deterministic simulation.
    if (!finite(c.position) || !finite(c.speed) || !finite(c.max_health) || !finite(c.attack_damage) ||
        !finite(c.attack_range) || !finite(c.attack_cooldown_seconds) || !finite(c.attack_windup_seconds) ||
        !finite(c.projectile_speed)) {
        return false;
    }
    if (c.attack_kind != sim::Command::kNoProfile && c.attack_kind != 0 && c.attack_kind != 1) return false;
    out = c;
    return true;
}

std::optional<MessageType> peek_type(std::span<const std::byte> message) {
    if (message.empty()) return std::nullopt;
    switch (static_cast<MessageType>(message[0])) {
        case MessageType::Hello:
        case MessageType::ClientCommand:
        case MessageType::Goodbye:
        case MessageType::Welcome:
        case MessageType::Reject:
        case MessageType::Batch:
            return static_cast<MessageType>(message[0]);
    }
    return std::nullopt;
}

std::vector<std::byte> encode(const HelloMsg& m) {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Hello));
    w.u32(m.version);
    w.u32(m.match_id);
    return w.take();
}

std::vector<std::byte> encode(const ClientCommandMsg& m) {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::ClientCommand));
    w.u64(m.target_tick);
    write_command(w, m.command);
    return w.take();
}

std::vector<std::byte> encode_goodbye() {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Goodbye));
    return w.take();
}

std::vector<std::byte> encode(const WelcomeMsg& m) {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Welcome));
    w.u32(m.player_id);
    w.i32(m.team);
    w.u32(m.match_id);
    w.u64(m.rng_seed);
    w.f64(m.tick_duration_seconds);
    w.u64(m.server_tick);
    return w.take();
}

std::vector<std::byte> encode(const RejectMsg& m) {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Reject));
    w.u8(static_cast<std::uint8_t>(m.reason));
    return w.take();
}

std::vector<std::byte> encode(const BatchMsg& m) {
    WireWriter w;
    w.u8(static_cast<std::uint8_t>(MessageType::Batch));
    w.u64(m.up_to_tick);
    w.u16(static_cast<std::uint16_t>(m.entries.size()));
    for (const BatchEntry& e : m.entries) {
        w.u64(e.tick);
        write_command(w, e.command);
    }
    return w.take();
}

std::optional<HelloMsg> decode_hello(std::span<const std::byte> message) {
    WireReader r(message);
    if (!expect_type(r, MessageType::Hello)) return std::nullopt;
    HelloMsg m;
    m.version = r.u32();
    m.match_id = r.u32();
    return finish(r, m);
}

std::optional<ClientCommandMsg> decode_client_command(std::span<const std::byte> message) {
    WireReader r(message);
    if (!expect_type(r, MessageType::ClientCommand)) return std::nullopt;
    ClientCommandMsg m;
    m.target_tick = r.u64();
    if (!read_command(r, m.command)) return std::nullopt;
    return finish(r, m);
}

std::optional<WelcomeMsg> decode_welcome(std::span<const std::byte> message) {
    WireReader r(message);
    if (!expect_type(r, MessageType::Welcome)) return std::nullopt;
    WelcomeMsg m;
    m.player_id = r.u32();
    m.team = r.i32();
    m.match_id = r.u32();
    m.rng_seed = r.u64();
    m.tick_duration_seconds = r.f64();
    m.server_tick = r.u64();
    if (r.ok() && !(finite(m.tick_duration_seconds) && m.tick_duration_seconds > 0.0)) return std::nullopt;
    return finish(r, m);
}

std::optional<RejectMsg> decode_reject(std::span<const std::byte> message) {
    WireReader r(message);
    if (!expect_type(r, MessageType::Reject)) return std::nullopt;
    const std::uint8_t reason = r.u8();
    if (reason < 1 || reason > 3) return std::nullopt;
    RejectMsg m;
    m.reason = static_cast<RejectReason>(reason);
    return finish(r, m);
}

std::optional<BatchMsg> decode_batch(std::span<const std::byte> message) {
    WireReader r(message);
    if (!expect_type(r, MessageType::Batch)) return std::nullopt;
    BatchMsg m;
    m.up_to_tick = r.u64();
    const std::uint16_t count = r.u16();
    if (!r.ok() || count > max_batch_entries(1500)) return std::nullopt;  // cap before allocating
    m.entries.reserve(count);
    for (std::uint16_t i = 0; i < count; ++i) {
        BatchEntry e;
        e.tick = r.u64();
        if (!read_command(r, e.command)) return std::nullopt;
        m.entries.push_back(e);
    }
    return finish(r, std::move(m));
}

}  // namespace thalassa::net
