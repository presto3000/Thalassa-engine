#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <span>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "thalassa/core/assert.hpp"
#include "thalassa/core/ecs/component_pool.hpp"
#include "thalassa/core/ecs/entity.hpp"
#include "thalassa/core/serialization/byte_stream.hpp"

// World: owns entity lifecycle (create/destroy with generation recycling)
// and a type-erased registry of ComponentPool<T> instances, one per
// component type ever used.
//
// Determinism note: `std::type_index`/`typeid` is used only as a *lookup
// key* to find the right pool for a compile-time-known type T at each call
// site (i.e. `pools_.find(typeid(T))` inside a template function where T is
// fixed by the caller) — we never iterate `pools_` itself in simulation
// logic, so the unspecified relative order of std::type_index values
// between compilers/runs never affects simulation results. See determinism
// rule 3 in docs/floating_point_policy.md.

namespace thalassa::core::ecs {

class World {
public:
    World() = default;
    World(const World&) = delete;
    World& operator=(const World&) = delete;
    World(World&&) noexcept = default;
    World& operator=(World&&) noexcept = default;

    // Creates a new entity. Reuses a freed slot's index with an incremented
    // generation if one is available (keeps the dense entity-index space
    // small and stable, which matters for network delta-encoding, otherwise allocates a new index.
    [[nodiscard]] Entity create_entity() {
        if (!free_indices_.empty()) {
            const Entity::IndexType index = free_indices_.back();
            free_indices_.pop_back();
            alive_[index] = true;
            const Entity::GenerationType generation = generations_[index];
            return Entity{index, generation};
        }

        const auto index = static_cast<Entity::IndexType>(generations_.size());
        generations_.push_back(0);
        alive_.push_back(true);
        return Entity{index, 0};
    }

    // Destroys an entity and removes it from every component pool that
    // holds it. Safe to call on an already-destroyed or default-constructed
    // entity (no-op).
    void destroy_entity(Entity e) {
        if (!is_alive(e)) {
            return;
        }
        for (const auto& erase_fn : erasers_) {
            erase_fn(*this, e);
        }
        alive_[e.index()] = false;
        ++generations_[e.index()];
        free_indices_.push_back(e.index());
    }

    [[nodiscard]] bool is_alive(Entity e) const noexcept {
        return e.is_valid() && e.index() < alive_.size() && alive_[e.index()] &&
               generations_[e.index()] == e.generation();
    }

    [[nodiscard]] std::size_t alive_count() const noexcept {
        return static_cast<std::size_t>(std::count(alive_.begin(), alive_.end(), true));
    }

    // Calls fn(Entity) for every currently-alive entity, in ascending
    // index order. Used by networking code (see thalassa/net/snapshot_delta.hpp)
    // to find entities present in one World but not another; general
    // enough to be useful beyond that, but deliberately not exposed as a
    // query-style each<>() overload since it has nothing to do with
    // component pools.
    template <typename Fn>
    void for_each_alive_entity(Fn&& fn) const {
        for (std::size_t i = 0; i < alive_.size(); ++i) {
            if (alive_[i]) {
                fn(Entity{static_cast<Entity::IndexType>(i), generations_[i]});
            }
        }
    }

    // Forces entity `e` (a specific index AND generation) to exist and be
    // alive in this World, growing the allocator if `e.index()` hasn't
    // been seen here before. Unlike create_entity(), which always assigns
    // the next free slot under this World's own allocation policy, this
    // reproduces an entity identity that was assigned somewhere else
    // (another process's World) — the only legitimate reason to do that
    // is network sync (see thalassa/net/snapshot_delta.hpp), which must
    // reproduce the server's entity ids exactly rather than let a client's
    // own allocator pick different ones. Not intended for gameplay code.
    void ensure_entity(Entity e) {
        if (e.index() >= generations_.size()) {
            const auto old_size = generations_.size();
            generations_.resize(std::size_t{e.index()} + 1, 0);
            alive_.resize(std::size_t{e.index()} + 1, false);
            for (auto i = old_size; i < e.index(); ++i) {
                free_indices_.push_back(static_cast<Entity::IndexType>(i));
            }
        }
        generations_[e.index()] = e.generation();
        if (!alive_[e.index()]) {
            alive_[e.index()] = true;
            std::erase(free_indices_, e.index());
        }
    }

    // --- Component access -------------------------------------------------

    template <typename T>
    T& emplace(Entity e, T value = T{}) {
        THALASSA_ASSERT(is_alive(e));
        return pool<T>().emplace(e, std::move(value));
    }

    template <typename T>
    void remove(Entity e) {
        pool<T>().erase(e);
    }

    template <typename T>
    [[nodiscard]] bool has(Entity e) const {
        auto it = pools_.find(std::type_index(typeid(T)));
        if (it == pools_.end()) {
            return false;
        }
        return static_cast<const TypedErasedPool<T>*>(it->second.get())->value.has(e);
    }

    template <typename T>
    [[nodiscard]] T& get(Entity e) {
        return pool<T>().get(e);
    }

    template <typename T>
    [[nodiscard]] const T& get(Entity e) const {
        auto it = pools_.find(std::type_index(typeid(T)));
        THALASSA_ASSERT(it != pools_.end());
        return static_cast<const TypedErasedPool<T>*>(it->second.get())->value.get(e);
    }

    template <typename T>
    [[nodiscard]] std::size_t component_count() const {
        auto it = pools_.find(std::type_index(typeid(T)));
        return it == pools_.end() ? 0 : static_cast<const TypedErasedPool<T>*>(it->second.get())->value.size();
    }

    // --- Queries ------------------------------------------------------
    //
    // each<T>(fn) iterates ComponentPool<T> in its dense (insertion) order
    // and calls fn(Entity, T&).
    //
    // each<T1, T2>(fn) picks whichever of the requested pools is smallest,
    // iterates its dense array in order, and calls fn(Entity, T1&, T2&)
    // for entities present in both pools. This is deterministic (see class
    // comment) regardless of which pool ends up smaller, because "smaller"
    // is itself a deterministic function of prior operations, and the
    // result set's relative order within the driving pool doesn't depend
    // on which pool happened to be smaller.

    template <typename T, typename Fn>
    void each(Fn&& fn) {
        auto& p = pool<T>();
        auto& entities = p.entities();
        auto& values = p.values();
        for (std::size_t i = 0; i < entities.size(); ++i) {
            fn(entities[i], values[i]);
        }
    }

    // Const overload: calls fn(Entity, const T&). Needed for any const
    // World& context (e.g. code that only holds a const SimWorld&, such as
    // thalassa::net::PredictedClient's confirmed_world()/predicted_world()
    // accessors, which deliberately return const references so callers
    // can't mutate prediction state through them). Uses find_pool<T>()
    // rather than the private, pool-creating pool<T>() — a const method
    // must not have the side effect of creating a pool that isn't there
    // yet, so "no pool yet" here just means "iterate zero entities."
    template <typename T, typename Fn>
    void each(Fn&& fn) const {
        const auto* p = find_pool<T>();
        if (p == nullptr) {
            return;
        }
        const auto& entities = p->entities();
        const auto& values = p->values();
        for (std::size_t i = 0; i < entities.size(); ++i) {
            fn(entities[i], values[i]);
        }
    }

    template <typename T1, typename T2, typename Fn>
    void each(Fn&& fn) {
        auto& p1 = pool<T1>();
        auto& p2 = pool<T2>();
        if (p1.size() <= p2.size()) {
            auto entities = p1.entities();  // copy: pool<T2>() call below may not resize p1, but keep this robust to future growth-during-iteration changes
            for (const Entity e : entities) {
                if (p2.has(e)) {
                    fn(e, p1.get(e), p2.get(e));
                }
            }
        } else {
            auto entities = p2.entities();
            for (const Entity e : entities) {
                if (p1.has(e)) {
                    fn(e, p1.get(e), p2.get(e));
                }
            }
        }
    }

    // Const overload: calls fn(Entity, const T1&, const T2&). See the
    // each<T>() const overload above for why this exists and why it uses
    // find_pool<T>() instead of pool<T>().
    template <typename T1, typename T2, typename Fn>
    void each(Fn&& fn) const {
        const auto* p1 = find_pool<T1>();
        const auto* p2 = find_pool<T2>();
        if (p1 == nullptr || p2 == nullptr) {
            return;
        }
        if (p1->size() <= p2->size()) {
            for (const Entity e : p1->entities()) {
                if (p2->has(e)) {
                    fn(e, p1->get(e), p2->get(e));
                }
            }
        } else {
            for (const Entity e : p2->entities()) {
                if (p1->has(e)) {
                    fn(e, p1->get(e), p2->get(e));
                }
            }
        }
    }

    // Three-component query, same deterministic-driver-pool approach as
    // the two-component overload above: whichever of the three pools is
    // currently smallest drives iteration, and the other two are checked
    // by membership. We hand-write N=1/2/3 rather than a fully generic
    // variadic
    template <typename T1, typename T2, typename T3, typename Fn>
    void each(Fn&& fn) {
        auto& p1 = pool<T1>();
        auto& p2 = pool<T2>();
        auto& p3 = pool<T3>();
        const std::size_t s1 = p1.size();
        const std::size_t s2 = p2.size();
        const std::size_t s3 = p3.size();

        if (s1 <= s2 && s1 <= s3) {
            auto entities = p1.entities();
            for (const Entity e : entities) {
                if (p2.has(e) && p3.has(e)) {
                    fn(e, p1.get(e), p2.get(e), p3.get(e));
                }
            }
        } else if (s2 <= s1 && s2 <= s3) {
            auto entities = p2.entities();
            for (const Entity e : entities) {
                if (p1.has(e) && p3.has(e)) {
                    fn(e, p1.get(e), p2.get(e), p3.get(e));
                }
            }
        } else {
            auto entities = p3.entities();
            for (const Entity e : entities) {
                if (p1.has(e) && p2.has(e)) {
                    fn(e, p1.get(e), p2.get(e), p3.get(e));
                }
            }
        }
    }

    // Const overload: calls fn(Entity, const T1&, const T2&, const T3&).
    // See the each<T>() const overload above for the rationale.
    template <typename T1, typename T2, typename T3, typename Fn>
    void each(Fn&& fn) const {
        const auto* p1 = find_pool<T1>();
        const auto* p2 = find_pool<T2>();
        const auto* p3 = find_pool<T3>();
        if (p1 == nullptr || p2 == nullptr || p3 == nullptr) {
            return;
        }
        const std::size_t s1 = p1->size();
        const std::size_t s2 = p2->size();
        const std::size_t s3 = p3->size();

        if (s1 <= s2 && s1 <= s3) {
            for (const Entity e : p1->entities()) {
                if (p2->has(e) && p3->has(e)) {
                    fn(e, p1->get(e), p2->get(e), p3->get(e));
                }
            }
        } else if (s2 <= s1 && s2 <= s3) {
            for (const Entity e : p2->entities()) {
                if (p1->has(e) && p3->has(e)) {
                    fn(e, p1->get(e), p2->get(e), p3->get(e));
                }
            }
        } else {
            for (const Entity e : p3->entities()) {
                if (p1->has(e) && p2->has(e)) {
                    fn(e, p1->get(e), p2->get(e), p3->get(e));
                }
            }
        }
    }

    // --- Serialization support --------------------------------------------
    //
    // Exposes typed pool access beyond the emplace/get/has/remove surface
    // above, specifically so gameplay code (thalassa_sim) can walk a fixed,
    // explicitly-registered list of component types and serialize each
    // pool without World needing to know what those types are. See
    // thalassa/sim/replay.hpp for the call site.

    // Gets-or-creates the pool for T (same as the private pool<T>(), made
    // available publicly under a clearer name for call sites that aren't
    // "component access" per se — e.g. deserialization, which needs to
    // create a pool before it has any entities in it yet).
    template <typename T>
    [[nodiscard]] ComponentPool<T>& ensure_pool() {
        return pool<T>();
    }

    // Read-only pool lookup that does NOT create the pool if absent
    // (unlike ensure_pool/pool<T>()). Returns nullptr if no entity has
    // ever had a T component in this World. Used by serialization/read
    // paths that want to skip writing empty pools without side effects.
    template <typename T>
    [[nodiscard]] const ComponentPool<T>* find_pool() const {
        auto it = pools_.find(std::type_index(typeid(T)));
        if (it == pools_.end()) {
            return nullptr;
        }
        return &static_cast<const TypedErasedPool<T>*>(it->second.get())->value;
    }

    // --- Entity-allocator serialization -------------------------------
    //
    // Saves/restores exactly the state create_entity()/destroy_entity()
    // maintain (generations_, alive_, free_indices_) so a snapshot
    // restores a World to an identical allocator state — including
    // generation counters — rather than just "the same set of alive
    // entities". That distinction matters because a stale Entity handle
    // from before a save/load must remain correctly stale (or correctly
    // valid) after load, exactly as it would have across a normal
    // destroy/recreate sequence elsewhere in the program.
    //
    // `alive_` is `std::vector<bool>`, whose specialized bit-packed
    // storage isn't a contiguous `bool*` we can block-copy, so it's
    // written one byte per slot rather than through
    // ByteWriter::write_pod_array.

    void save_entity_allocator(serialization::ByteWriter& writer) const {
        writer.write_pod_array<Entity::GenerationType>(std::span<const Entity::GenerationType>(generations_));
        writer.write_u64(alive_.size());
        for (const bool is_alive_flag : alive_) {
            writer.write_pod<std::uint8_t>(is_alive_flag ? 1 : 0);
        }
        writer.write_pod_array<Entity::IndexType>(std::span<const Entity::IndexType>(free_indices_));
    }

    // Rebuilds the entity allocator (and discards all entities/components
    // currently in this World — this is "replace this World's identity
    // wholesale", used right after default-constructing a fresh World
    // during load, not a merge). Component pools are restored separately
    // by the caller via ensure_pool<T>().deserialize() for each
    // explicitly-registered component type, since World doesn't know what
    // those types are — see thalassa/sim/replay.hpp for the call site that
    // drives both in the right order.
    void load_entity_allocator(serialization::ByteReader& reader) {
        pools_.clear();
        erasers_.clear();

        generations_ = reader.read_pod_array<Entity::GenerationType>();

        const auto alive_count = reader.read_u64();
        alive_.assign(alive_count, false);
        for (std::uint64_t i = 0; i < alive_count; ++i) {
            alive_[i] = reader.read_pod<std::uint8_t>() != 0;
        }

        free_indices_ = reader.read_pod_array<Entity::IndexType>();
    }

private:
    // Type-erased pool ownership: a small polymorphic base with a virtual
    // destructor, downcast at known-T call sites only (pool<T>() and the
    // const accessors above). Keeps World's header free of per-component
    // boilerplate outside this file.
    struct ErasedPool {
        virtual ~ErasedPool() = default;
    };
    template <typename T>
    struct TypedErasedPool final : ErasedPool {
        ComponentPool<T> value;
    };

    template <typename T>
    ComponentPool<T>& pool() {
        const std::type_index key(typeid(T));
        auto it = pools_.find(key);
        if (it == pools_.end()) {
            auto new_pool = std::make_unique<TypedErasedPool<T>>();
            auto* raw = new_pool.get();
            pools_.emplace(key, std::move(new_pool));
            erasers_.push_back([](World& world, Entity e) { world.pool<T>().erase(e); });
            return raw->value;
        }
        return static_cast<TypedErasedPool<T>*>(it->second.get())->value;
    }

    std::unordered_map<std::type_index, std::unique_ptr<ErasedPool>> pools_;
    std::vector<void (*)(World&, Entity)> erasers_;

    std::vector<Entity::GenerationType> generations_;
    std::vector<bool> alive_;
    std::vector<Entity::IndexType> free_indices_;
};

}  // namespace thalassa::core::ecs
