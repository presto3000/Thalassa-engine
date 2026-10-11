#include "thalassa/sim/replay_recording.hpp"

#include <algorithm>
#include <cmath>

#include "thalassa/sim/replay.hpp"

namespace thalassa::sim {

namespace {

using core::serialization::ByteReader;
using core::serialization::ByteWriter;

void write_command(ByteWriter& w, const Command& c) {
    // Field-by-field (not write_pod(Command)) so struct padding never leaks
    // indeterminate bytes into the file.
    w.write_pod(static_cast<std::uint8_t>(c.type));
    w.write_pod(c.entity);
    w.write_pod(c.position);
    w.write_pod(c.speed);
    w.write_pod(c.owner);
    w.write_pod(c.team);
    w.write_pod(c.max_health);
    w.write_pod(c.attack_damage);
    w.write_pod(c.attack_range);
    w.write_pod(c.attack_cooldown_seconds);
}

Command read_command(ByteReader& r) {
    Command c;
    c.type = static_cast<Command::Type>(r.read_pod<std::uint8_t>());
    c.entity = r.read_pod<core::ecs::Entity>();
    c.position = r.read_pod<core::math::Vec3>();
    c.speed = r.read_pod<core::Scalar>();
    c.owner = r.read_pod<core::PlayerId>();
    c.team = r.read_pod<std::int32_t>();
    c.max_health = r.read_pod<core::Scalar>();
    c.attack_damage = r.read_pod<core::Scalar>();
    c.attack_range = r.read_pod<core::Scalar>();
    c.attack_cooldown_seconds = r.read_pod<core::Scalar>();
    return c;
}

}  // namespace

std::uint64_t world_checksum(const SimWorld& world) {
    ByteWriter w;
    save_world(world, w);
    std::uint64_t h = 14695981039346656037ULL;
    for (const std::byte b : w.bytes()) {
        h ^= static_cast<std::uint64_t>(b);
        h *= 1099511628211ULL;
    }
    return h;
}

void ReplayRecording::serialize(ByteWriter& w) const {
    w.write_u32(kReplayMagic);
    w.write_u32(kReplayVersion);
    w.write_u64(seed);
    w.write_pod(tick_duration_seconds);
    w.write_u64(total_ticks);
    w.write_u64(final_checksum);
    w.write_u32(checkpoint_interval);
    w.write_u64(checkpoints.size());
    for (const auto& cp : checkpoints) {
        w.write_u64(cp.tick);
        w.write_u64(cp.checksum);
    }
    const std::vector<std::uint64_t> ticks = commands.ticks();  // ascending
    w.write_u64(ticks.size());
    for (const std::uint64_t t : ticks) {
        const std::vector<Command>& cmds = *commands.commands_at(t);
        w.write_u64(t);
        w.write_u64(cmds.size());
        for (const Command& c : cmds) write_command(w, c);
    }
}

bool ReplayRecording::deserialize(ByteReader& r) {
    if (r.read_u32() != kReplayMagic) return false;
    if (r.read_u32() != kReplayVersion) return false;
    seed = r.read_u64();
    tick_duration_seconds = r.read_pod<core::Scalar>();
    total_ticks = r.read_u64();
    final_checksum = r.read_u64();
    checkpoint_interval = r.read_u32();
    const std::uint64_t cp_count = r.read_u64();
    checkpoints.clear();
    checkpoints.reserve(cp_count);
    for (std::uint64_t i = 0; i < cp_count; ++i) {
        ReplayCheckpoint cp;
        cp.tick = r.read_u64();
        cp.checksum = r.read_u64();
        checkpoints.push_back(cp);
    }
    commands = CommandLog{};
    const std::uint64_t tick_count = r.read_u64();
    for (std::uint64_t i = 0; i < tick_count; ++i) {
        const std::uint64_t t = r.read_u64();
        const std::uint64_t n = r.read_u64();
        for (std::uint64_t j = 0; j < n; ++j) commands.add(t, read_command(r));
    }
    return true;
}

ReplayRecorder::ReplayRecorder(std::uint64_t seed, core::Scalar tick_duration_seconds,
                               std::uint32_t checkpoint_interval)
    : sim_(seed) {
    sim_.set_tick_duration_seconds(tick_duration_seconds);
    rec_.seed = seed;
    rec_.tick_duration_seconds = tick_duration_seconds;
    rec_.checkpoint_interval = checkpoint_interval;
}

void ReplayRecorder::step(const std::vector<Command>& commands) {
    const std::uint64_t tick = sim_.ticks_simulated();
    for (const Command& c : commands) rec_.commands.add(tick, c);
    sim_.tick(core::SimTick{tick}, commands);
    if (rec_.checkpoint_interval != 0 && sim_.ticks_simulated() % rec_.checkpoint_interval == 0) {
        rec_.checkpoints.push_back({sim_.ticks_simulated(), world_checksum(sim_)});
    }
}

ReplayRecording ReplayRecorder::finish() const {
    ReplayRecording out = rec_;
    out.total_ticks = sim_.ticks_simulated();
    out.final_checksum = world_checksum(sim_);
    return out;
}

ReplayPlayer::ReplayPlayer(const ReplayRecording& recording) : rec_(recording) { restart(); }

void ReplayPlayer::set_speed(double multiplier) {
    if (!(multiplier > 0.0)) multiplier = 0.0;  // also catches NaN
    speed_ = std::min(multiplier, 64.0);
}

void ReplayPlayer::run_one_tick() {
    const std::uint64_t tick = sim_->ticks_simulated();
    static const std::vector<Command> kNone;
    const std::vector<Command>* cmds = rec_.commands.commands_at(tick);
    sim_->tick(core::SimTick{tick}, cmds != nullptr ? *cmds : kNone);

    if (rec_.checkpoint_interval != 0) {
        const std::uint64_t now = sim_->ticks_simulated();
        if (now % rec_.checkpoint_interval == 0) {
            const std::uint64_t idx = now / rec_.checkpoint_interval - 1;
            if (idx < rec_.checkpoints.size() && rec_.checkpoints[idx].tick == now &&
                rec_.checkpoints[idx].checksum != world_checksum(*sim_)) {
                desynced_ = true;
            }
        }
    }
}

bool ReplayPlayer::step_tick() {
    if (finished()) return false;
    run_one_tick();
    return true;
}

std::uint64_t ReplayPlayer::advance(double real_seconds) {
    if (paused_ || speed_ <= 0.0 || !(real_seconds > 0.0) || finished()) return 0;
    tick_accumulator_ += real_seconds * speed_ / rec_.tick_duration_seconds;
    std::uint64_t ran = 0;
    while (tick_accumulator_ >= 1.0 && !finished()) {
        run_one_tick();
        tick_accumulator_ -= 1.0;
        ++ran;
    }
    if (finished()) tick_accumulator_ = 0.0;
    return ran;
}

void ReplayPlayer::restart() {
    sim_ = std::make_unique<SimWorld>(rec_.seed);
    sim_->set_tick_duration_seconds(rec_.tick_duration_seconds);
    tick_accumulator_ = 0.0;
    desynced_ = false;
}

void ReplayPlayer::seek(std::uint64_t tick) {
    tick = std::min(tick, rec_.total_ticks);
    if (tick < sim_->ticks_simulated()) restart();
    while (sim_->ticks_simulated() < tick) run_one_tick();
}

bool ReplayPlayer::final_checksum_matches() const {
    return finished() && world_checksum(*sim_) == rec_.final_checksum;
}

}  // namespace thalassa::sim
