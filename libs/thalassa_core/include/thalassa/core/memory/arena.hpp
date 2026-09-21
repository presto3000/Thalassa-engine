#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <utility>
#include <vector>

#include "thalassa/core/config.hpp"

// Linear/arena allocator used throughout thalassa_core/sim for per-frame and
// per-world scratch allocations. No exceptions are thrown on allocation
// failure in the hot-path `allocate()` — it returns nullptr. Callers in hot code must
// check; callers in cold/init code may use `allocate_or_abort()`.

namespace thalassa::core::memory {

// A single contiguous block that bump-allocates. Never frees individual
// allocations — call reset() to reclaim everything at once (typical use:
// reset once per simulation tick for scratch data, or once per world
// lifetime for long-lived arena-backed containers).
class Arena {
public:
    explicit Arena(std::size_t capacity_bytes)
        : capacity_(capacity_bytes),
          buffer_(std::make_unique<std::byte[]>(capacity_bytes)) {}

    Arena(const Arena&) = delete;
    Arena& operator=(const Arena&) = delete;
    Arena(Arena&&) noexcept = default;
    Arena& operator=(Arena&&) noexcept = default;

    // Bump-allocates `size` bytes aligned to `alignment` (must be a power of
    // two). Returns nullptr if the arena is exhausted — no exceptions, no
    // fallback allocation, by design: callers decide how to handle
    // exhaustion (grow a new arena, log-and-abort at init time, etc).
    [[nodiscard]] void* allocate(std::size_t size, std::size_t alignment = alignof(std::max_align_t)) noexcept {
        const std::uintptr_t current = reinterpret_cast<std::uintptr_t>(buffer_.get() + offset_);
        const std::uintptr_t aligned = (current + (alignment - 1)) & ~(alignment - 1);
        const std::size_t padding = static_cast<std::size_t>(aligned - current);

        if (offset_ + padding + size > capacity_) {
            return nullptr;
        }
        offset_ += padding + size;
        return reinterpret_cast<void*>(aligned);
    }

    // Constructs a T in the arena. Aborts the process if the arena is
    // exhausted — intended for initialization-time allocations where
    // running out of arena space indicates a configuration bug, not a
    // recoverable runtime condition.
    template <typename T, typename... Args>
    [[nodiscard]] T* allocate_or_abort(Args&&... args) noexcept {
        void* mem = allocate(sizeof(T), alignof(T));
        if (mem == nullptr) {
            std::abort();
        }
        return std::construct_at(static_cast<T*>(mem), std::forward<Args>(args)...);
    }

    // Reclaims the entire arena for reuse. Does NOT call destructors on
    // objects previously allocated from it — callers are responsible for
    // destructor calls if their allocated types are non-trivial and the
    // caller cares about running them (typical arena usage here is POD
    // component data, where it doesn't matter).
    void reset() noexcept { offset_ = 0; }

    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }
    [[nodiscard]] std::size_t used() const noexcept { return offset_; }
    [[nodiscard]] std::size_t remaining() const noexcept { return capacity_ - offset_; }

private:
    std::size_t capacity_;
    std::size_t offset_ = 0;
    std::unique_ptr<std::byte[]> buffer_;
};

}  // namespace thalassa::core::memory
