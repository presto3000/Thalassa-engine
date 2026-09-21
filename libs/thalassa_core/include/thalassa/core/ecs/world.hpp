#pragma once

#include <algorithm>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "thalassa/core/assert.hpp"
#include "thalassa/core/ecs/component_pool.hpp"
#include "thalassa/core/ecs/entity.hpp"

// World: owns entity lifecycle (create/destroy with generation recycling)
// and a type-erased registry of ComponentPool<T> instances, one per
// component type ever used.
//
// Determinism note: `std::type_index`/`typeid` is used only as a *lookup
// key* to find the right pool for a compile-time-known type T at each call
// site (i.e. `pools_.find(typeid(T))` inside a template function where T is
// fixed by the caller) — never iterate `pools_` itself in simulation
// logic, so the unspecified relative order of std::type_index values
// between compilers/runs never affects simulation results.

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
    // small and stable, which matters for network delta-encoding), otherwise allocates a new index.
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
