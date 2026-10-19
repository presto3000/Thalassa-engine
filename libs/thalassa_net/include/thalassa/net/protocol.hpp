#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "thalassa/net/wire.hpp"
#include "thalassa/sim/commands.hpp"

// The game protocol spoken over ReliableChannel between GameClient and
// GameServer. All messages here are sent RELIABLY (exactly once, in order).
//
// Model (see docs/milestone3_networking_design.md): the simulation is
// deterministic, so the server does not stream world state; it streams the
// AUTHORITATIVE COMMAND LOG. A client that applies the same commands on the
// same ticks reproduces the server's world exactly, and predicts ahead of it
// with its own commands (PredictedClient). Bandwidth is tiny and a late
// joiner just replays the history.
//
//   client -> server
//     Hello          join a match (protocol version check)
//     ClientCommand  "run this command on tick T"
//     Goodbye        leaving
//   server -> client
//     Welcome        you are player P on team X; match seed; current tick
//     Reject         join refused (reason)
//     Batch          authoritative commands, plus "every tick < up_to_tick is final"
//
// Decoders return nullopt for ANY malformed message (short, bad enum, NaN
// numbers, trailing bytes): network input is untrusted and must never crash.

namespace thalassa::net {

inline constexpr std::uint32_t kProtocolVersion = 1;

enum class MessageType : std::uint8_t {
    Hello = 1,
    ClientCommand = 2,
    Goodbye = 3,
    Welcome = 10,
    Reject = 11,
    Batch = 12,
};

enum class RejectReason : std::uint8_t {
    BadVersion = 1,
    UnknownMatch = 2,
    ServerFull = 3,
};

struct HelloMsg {
    std::uint32_t version = kProtocolVersion;
    std::uint32_t match_id = 0;
};
struct ClientCommandMsg {
    std::uint64_t target_tick = 0;
    sim::Command command;
};
struct WelcomeMsg {
    std::uint32_t player_id = 0;
    std::int32_t team = 0;
    std::uint32_t match_id = 0;
    std::uint64_t rng_seed = 0;
    double tick_duration_seconds = 1.0 / 60.0;
    std::uint64_t server_tick = 0;  // the server's current tick when it sent this
};
struct RejectMsg {
    RejectReason reason = RejectReason::BadVersion;
};
struct BatchEntry {
    std::uint64_t tick = 0;
    sim::Command command;
};
struct BatchMsg {
    std::uint64_t up_to_tick = 0;  // every tick < up_to_tick is complete: no more commands will ever arrive for it
    std::vector<BatchEntry> entries;
};

// Encoded size of one Command, and the most entries a Batch can carry while
// fitting a ReliableChannel message.
inline constexpr std::size_t kCommandWireBytes = 101;
inline constexpr std::size_t kBatchHeaderBytes = 11;
inline constexpr std::size_t kBatchEntryBytes = 8 + kCommandWireBytes;
[[nodiscard]] constexpr std::size_t max_batch_entries(std::size_t max_message_bytes) {
    return (max_message_bytes - kBatchHeaderBytes) / kBatchEntryBytes;
}

[[nodiscard]] std::optional<MessageType> peek_type(std::span<const std::byte> message);

[[nodiscard]] std::vector<std::byte> encode(const HelloMsg& m);
[[nodiscard]] std::vector<std::byte> encode(const ClientCommandMsg& m);
[[nodiscard]] std::vector<std::byte> encode_goodbye();
[[nodiscard]] std::vector<std::byte> encode(const WelcomeMsg& m);
[[nodiscard]] std::vector<std::byte> encode(const RejectMsg& m);
[[nodiscard]] std::vector<std::byte> encode(const BatchMsg& m);

[[nodiscard]] std::optional<HelloMsg> decode_hello(std::span<const std::byte> message);
[[nodiscard]] std::optional<ClientCommandMsg> decode_client_command(std::span<const std::byte> message);
[[nodiscard]] std::optional<WelcomeMsg> decode_welcome(std::span<const std::byte> message);
[[nodiscard]] std::optional<RejectMsg> decode_reject(std::span<const std::byte> message);
[[nodiscard]] std::optional<BatchMsg> decode_batch(std::span<const std::byte> message);

// Exposed for tests / reuse.
void write_command(WireWriter& w, const sim::Command& c);
[[nodiscard]] bool read_command(WireReader& r, sim::Command& out);  // false on malformed/non-finite

}  // namespace thalassa::net
