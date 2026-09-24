#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include "thalassa/core/assert.hpp"

// ByteWriter/ByteReader: minimal binary serialization primitives used for
// world-state snapshots (save games, replay, and later network resync).
//
// Endianness policy: no byte-swapping is performed. This is consistent
// with the rest of the determinism policy (see
// docs/floating_point_policy.md) — we already assume a fixed, homogeneous
// fleet (same compiler flags, same ISA family) for bit-identical
// simulation, and every platform we currently target (x86-64, ARM64) is
// little-endian. If Thalassa ever needs to support a big-endian target,
// this is the one place that would need byte-swapping added; flagging it
// explicitly here rather than leaving it implicit.
//
// This header intentionally does NOT know about ECS, components, or
// gameplay types — see thalassa/core/ecs/component_pool.hpp for the
// trivially-copyable-component (de)serialization built on top of this, and
// thalassa/sim/replay.hpp for the SimWorld-level snapshot format.

namespace thalassa::core::serialization {

class ByteWriter {
public:
    void write_bytes(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::byte*>(data);
        buffer_.insert(buffer_.end(), bytes, bytes + size);
    }

    template <typename T>
    void write_pod(const T& value) {
        static_assert(std::is_trivially_copyable_v<T>, "write_pod requires a trivially copyable type");
        write_bytes(&value, sizeof(T));
    }

    void write_u32(std::uint32_t value) { write_pod(value); }
    void write_u64(std::uint64_t value) { write_pod(value); }

    // Writes a contiguous span of trivially-copyable values as a single
    // memcpy-equivalent block, prefixed with a u64 element count. This is
    // the fast path ComponentPool<T>::serialize uses for a whole dense
    // array at once — see component_pool.hpp.
    template <typename T>
    void write_pod_array(std::span<const T> values) {
        static_assert(std::is_trivially_copyable_v<T>, "write_pod_array requires a trivially copyable type");
        write_u64(static_cast<std::uint64_t>(values.size()));
        if (!values.empty()) {
            write_bytes(values.data(), values.size() * sizeof(T));
        }
    }

    [[nodiscard]] const std::vector<std::byte>& bytes() const noexcept { return buffer_; }
    [[nodiscard]] std::size_t size() const noexcept { return buffer_.size(); }

private:
    std::vector<std::byte> buffer_;
};

// Reads back a buffer written by ByteWriter. Every read is bounds-checked
// via THALASSA_ASSERT — a truncated/corrupt buffer aborts rather than
// silently reading garbage or invoking UB, consistent with the
// coding-standard rationale for always-on asserts (see assert.hpp): a
// desynced/corrupted replay is worse than a hard, debuggable failure.
class ByteReader {
public:
    explicit ByteReader(std::span<const std::byte> data) noexcept : data_(data) {}

    void read_bytes(void* out, std::size_t size) {
        THALASSA_ASSERT_MSG(offset_ + size <= data_.size(), "ByteReader: read past end of buffer");
        std::memcpy(out, data_.data() + offset_, size);
        offset_ += size;
    }

    template <typename T>
    [[nodiscard]] T read_pod() {
        static_assert(std::is_trivially_copyable_v<T>, "read_pod requires a trivially copyable type");
        T value{};
        read_bytes(&value, sizeof(T));
        return value;
    }

    [[nodiscard]] std::uint32_t read_u32() { return read_pod<std::uint32_t>(); }
    [[nodiscard]] std::uint64_t read_u64() { return read_pod<std::uint64_t>(); }

    // Inverse of write_pod_array: reads the u64 count prefix, then that
    // many T's as a single block.
    template <typename T>
    [[nodiscard]] std::vector<T> read_pod_array() {
        static_assert(std::is_trivially_copyable_v<T>, "read_pod_array requires a trivially copyable type");
        const auto count = read_u64();
        std::vector<T> values(count);
        if (count > 0) {
            read_bytes(values.data(), count * sizeof(T));
        }
        return values;
    }

    [[nodiscard]] bool at_end() const noexcept { return offset_ == data_.size(); }
    [[nodiscard]] std::size_t remaining() const noexcept { return data_.size() - offset_; }

private:
    std::span<const std::byte> data_;
    std::size_t offset_ = 0;
};

}  // namespace thalassa::core::serialization
