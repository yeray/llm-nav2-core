#pragma once

#include <string>
#include <string_view>

#include "llm_nav2_core/domain/result.hpp"
#include "llm_nav2_core/domain/types.hpp"
#include "llm_nav2_core/domain/waypoint_registry.hpp"
#include "llm_nav2_core/ports/intent_extractor.hpp"
#include "llm_nav2_core/ports/navigator.hpp"

namespace llm_nav2_core::domain {

class NavCommandProcessor {
public:
  static Result<NavCommandProcessor> create(
      WaypointRegistry registry,
      ports::IIntentExtractor& extractor,
      ports::INavigator* navigator = nullptr);

  // Processes a natural language command end-to-end:
  // 1) Runs GBNF-constrained LLM inference via IIntentExtractor
  // 2) Parses strict JSON into NavIntent
  // 3) Resolves semantic target into exact Pose2D coordinates
  // 4) Dispatches to INavigator (if attached)
  Result<NavGoal> process(std::string_view user_command);

  [[nodiscard]] const std::string& gbnf_grammar() const noexcept {
    return gbnf_grammar_;
  }

  [[nodiscard]] const WaypointRegistry& registry() const noexcept {
    return registry_;
  }

private:
  NavCommandProcessor(
      WaypointRegistry registry,
      std::string gbnf_grammar,
      ports::IIntentExtractor& extractor,
      ports::INavigator* navigator);

  WaypointRegistry registry_;
  std::string gbnf_grammar_;
  ports::IIntentExtractor& extractor_;
  ports::INavigator* navigator_;
};

}  // namespace llm_nav2_core::domain
