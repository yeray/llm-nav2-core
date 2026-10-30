#pragma once

#include <string>
#include <string_view>

#include "llm_nav2_core/domain/result.hpp"
#include "llm_nav2_core/domain/types.hpp"
#include "llm_nav2_core/domain/waypoint_registry.hpp"

namespace llm_nav2_core::domain {

class GbnfBuilder {
public:
  // Dynamically builds a strict GBNF grammar from the registered waypoints.
  // Guarantees that:
  // 1) Output is a single-line compact JSON object.
  // 2) If action == "navigate", target MUST be one of the registered waypoints.
  // 3) If action == "stop" or "unknown", target MUST be "none".
  static Result<std::string> build(const WaypointRegistry& registry);
};

class IntentJsonParser {
public:
  // Parses the deterministic JSON produced under GBNF constraints.
  static Result<NavIntent> parse(std::string_view json_str);
};

}  // namespace llm_nav2_core::domain
