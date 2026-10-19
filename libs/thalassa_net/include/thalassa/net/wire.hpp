#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Minimal little-endian wire encoding for network messages.
//
// WireReader is the important half: unlike ByteReader (which asserts, because
// a truncated *local file* is a bug), data from the network is untrusted, so
// every read is bounds-checked and a short/garbled message just flips ok() to
// false. Callers must check ok() once at the end and discard the message if it
// is false. Nothing here can crash on hostile input.

namespace thalassa::net {

class WireWriter {
public:
    void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
    void u16(std::uint16_t v) {
        u8(static_cast<std::uint8_t>(v & 0xFFu));
        u8(static_cast<std::uint8_t>(v >> 8));
    }
    void u32(std::uint32_t v) {
        for (int i = 0; i < 4; ++i) u8(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
    }
    void u64(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) u8(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
    }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }

    [[nodiscard]] const std::vector<std::byte>& bytes() const noexcept { return buf_; }
    [[nodiscard]] std::vector<std::byte> take() noexcept { return std::move(buf_); }
    [[nodiscard]] std::size_t size() const noexcept { return buf_.size(); }

private:
    std::vector<std::byte> buf_;
};

class WireReader {
public:
    explicit WireReader(std::span<const std::byte> data) noexcept : data_(data) {}

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] std::size_t remaining() const noexcept { return data_.size() - pos_; }

    std::uint8_t u8() {
        if (pos_ + 1 > data_.size()) {
            ok_ = false;
            return 0;
        }
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
    std::uint64_t u64() {
        std::uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(u8()) << (8 * i);
        return v;
    }
    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    double f64() { return std::bit_cast<double>(u64()); }

private:
    std::span<const std::byte> data_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

}  // namespace thalassa::net
