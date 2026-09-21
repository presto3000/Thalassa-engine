#include "thalassa/server/headless_app.hpp"

// HeadlessApp is header-only (see headless_app.hpp). This
// translation unit exists so thalassa_server links as a real static
// library from day one; will add multi-instance hosting
// (running several SimWorld/HeadlessApp-like objects per process) and
// networking glue here.

namespace thalassa::server {

// Intentionally empty

}  // namespace thalassa::server
