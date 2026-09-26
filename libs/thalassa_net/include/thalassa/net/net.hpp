#pragma once

// thalassa_net: server-authoritative networking logic — client-side
// prediction + reconciliation, snapshot/delta compression, entity
// relevance. See docs/milestone3_networking_design.md for the module's
// scope (transport-agnostic; no socket layer) and design rationale.
//
// Convenience umbrella header pulling in all of thalassa_net; translation
// units that only need one piece are still encouraged to include just
// that header to keep build times down.

#include "thalassa/net/predicted_client.hpp"
#include "thalassa/net/relevance.hpp"
#include "thalassa/net/snapshot_delta.hpp"
