#pragma once

#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

#include "thalassa/core/assert.hpp"
#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/serialization/byte_stream.hpp"

// ComponentPool<T>: sparse-set storage for a single component type. See
// docs/ecs_decision.md for why sparse-set over archetype-table storage.
//
// - `dense_` holds component data packed contiguously, in insertion order.
// - `dense_entities_` is a parallel array of which Entity owns each dense
//   slot (needed for swap-and-pop removal and for iteration-with-entity).
// - `sparse_` maps Entity::index() -> dense array index, sized to the
//   largest entity index ever seen (grown lazily, "invalid" slots are a
//   sentinel), giving O(1) has()/get()/erase() without hashing.
//
// Iteration over `dense_`/`dense_entities_` is *always* insertion order,
// which is the deterministic-order guarantee referenced in
// docs/ecs_decision.md: two runs that create entities and add components
// in the same order iterate them in the same order, on any machine.

namespace thalassa::core::ecs {

    template <typename T>
    class ComponentPool {
    public:
        static constexpr std::uint32_t kInvalidDenseIndex = 0xFFFFFFFFu;

        // Inserts or overwrites the component for `e`. Returns a reference to
        // the stored component.
        T& emplace(Entity e, T value = T{}) {
            ensure_sparse_capacity(e.index());
            const std::uint32_t existing = sparse_[e.index()];
            if (existing != kInvalidDenseIndex && dense_entities_[existing] == e) {
                dense_[existing] = std::move(value);
                return dense_[existing];
            }

            const auto new_index = static_cast<std::uint32_t>(dense_.size());
            dense_.push_back(std::move(value));
            dense_entities_.push_back(e);
            sparse_[e.index()] = new_index;
            return dense_.back();
        }

        // Removes the component for `e`, if present. Swap-and-pop keeps the
        // dense array contiguous; this changes the iteration position of
        // whichever entity used to be last, but never their relative order
        // versus entities inserted before them that remain — determinism only
        // requires that *the same sequence of operations* yields the same
        // resulting order, which swap-and-pop satisfies since it's a pure
        // function of the operation sequence, not of timing.
        void erase(Entity e) {
            if (!has(e)) {
                return;
            }
            const std::uint32_t remove_index = sparse_[e.index()];
            const std::uint32_t last_index = static_cast<std::uint32_t>(dense_.size() - 1);
            const Entity last_entity = dense_entities_[last_index];

            dense_[remove_index] = std::move(dense_[last_index]);
            dense_entities_[remove_index] = last_entity;
            sparse_[last_entity.index()] = remove_index;

            dense_.pop_back();
            dense_entities_.pop_back();
            sparse_[e.index()] = kInvalidDenseIndex;
        }

        [[nodiscard]] bool has(Entity e) const noexcept {
            if (e.index() >= sparse_.size()) {
                return false;
            }
            const std::uint32_t dense_index = sparse_[e.index()];
            return dense_index != kInvalidDenseIndex && dense_entities_[dense_index] == e;
        }

        [[nodiscard]] T& get(Entity e) noexcept {
            THALASSA_ASSERT(has(e));
            return dense_[sparse_[e.index()]];
        }

        [[nodiscard]] const T& get(Entity e) const noexcept {
            THALASSA_ASSERT(has(e));
            return dense_[sparse_[e.index()]];
        }

        [[nodiscard]] std::size_t size() const noexcept { return dense_.size(); }
        [[nodiscard]] bool empty() const noexcept { return dense_.empty(); }

        // Direct access for query iteration (see world.hpp). Always insertion
        // order as described above.
        [[nodiscard]] const std::vector<Entity>& entities() const noexcept { return dense_entities_; }
        [[nodiscard]] std::vector<T>& values() noexcept { return dense_; }
        [[nodiscard]] const std::vector<T>& values() const noexcept { return dense_; }

        void clear() noexcept {
            dense_.clear();
            dense_entities_.clear();
            sparse_.clear();
        }

        // --- Serialization -----------------------------------------------
        //
        // Trivially-copyable T (enforced by ByteWriter/ByteReader's own
        // static_asserts) means this is a straight block-copy of the dense
        // arrays — no per-field logic, no risk of a hand-written serializer
        // silently missing a field when someone adds one to T. `sparse_` is
        // NOT written: it's fully derivable from `dense_entities_` and is
        // reconstructed by deserialize(), which both saves space and means
        // the format doesn't depend on `sparse_`'s size (which is really just
        // "highest entity index ever seen in this pool", an implementation
        // detail).
        void serialize(serialization::ByteWriter& writer) const {
            writer.write_pod_array<Entity>(std::span<const Entity>(dense_entities_));
            writer.write_pod_array<T>(std::span<const T>(dense_));
        }

        // Replaces this pool's entire contents with what `reader` holds.
        void deserialize(serialization::ByteReader& reader) {
            clear();
            dense_entities_ = reader.read_pod_array<Entity>();
            dense_ = reader.read_pod_array<T>();
            THALASSA_ASSERT_MSG(dense_entities_.size() == dense_.size(),
                "ComponentPool::deserialize: entity/value count mismatch");

            for (std::uint32_t i = 0; i < dense_entities_.size(); ++i) {
                ensure_sparse_capacity(dense_entities_[i].index());
                sparse_[dense_entities_[i].index()] = i;
            }
        }

    private:
        void ensure_sparse_capacity(std::uint32_t index) {
            if (index >= sparse_.size()) {
                sparse_.resize(std::size_t{ index } + 1, kInvalidDenseIndex);
            }
        }

        std::vector<T> dense_; // components
        std::vector<Entity> dense_entities_; // entities
        std::vector<std::uint32_t> sparse_; // entities to components mapping
    };

}  // namespace thalassa::core::ecs
