#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "llm_nav2_core/domain/gbnf_builder.hpp"
#include "llm_nav2_core/domain/nav_command_processor.hpp"
#include "llm_nav2_core/domain/waypoint_registry.hpp"

namespace {

using llm_nav2_core::domain::ActionType;
using llm_nav2_core::domain::ErrorCode;
using llm_nav2_core::domain::GbnfBuilder;
using llm_nav2_core::domain::IntentJsonParser;
using llm_nav2_core::domain::NavCommandProcessor;
using llm_nav2_core::domain::NavGoal;
using llm_nav2_core::domain::Result;
using llm_nav2_core::domain::WaypointRegistry;
using llm_nav2_core::ports::IIntentExtractor;
using llm_nav2_core::ports::INavigator;

class FakeIntentExtractor final : public IIntentExtractor {
public:
  std::string next_json_response;
  std::string last_received_grammar;

  Result<std::string> extract_json(
      std::string_view /*user_prompt*/,
      std::string_view gbnf_grammar) override {
    last_received_grammar = std::string(gbnf_grammar);
    return Result<std::string>::Ok(next_json_response);
  }
};

class SpyNavigator final : public INavigator {
public:
  int navigate_calls{0};
  int cancel_calls{0};
  NavGoal last_goal{};

  Result<bool> navigate_to(const NavGoal& goal) override {
    ++navigate_calls;
    last_goal = goal;
    return Result<bool>::Ok(true);
  }

  Result<bool> cancel_navigation() override {
    ++cancel_calls;
    return Result<bool>::Ok(true);
  }
};

void test_yaml_loading_and_lookup() {
  constexpr std::string_view kYaml = R"(
waypoints:
  kitchen:
    x: 3.5
    y: -1.2
    yaw: 1.5708
  charging_dock:
    x: 0.0
    y: 0.0
    yaw: 0.0
)";

  auto reg_res = WaypointRegistry::load_from_yaml_string(kYaml);
  assert(reg_res.is_ok());

  const auto& reg = reg_res.value();
  assert(reg.size() == 2);

  auto kitchen = reg.resolve("kitchen");
  assert(kitchen.is_ok());
  assert(std::abs(kitchen.value().x - 3.5) < 1e-6);
  assert(std::abs(kitchen.value().y - (-1.2)) < 1e-6);

  const auto q = kitchen.value().to_quaternion();
  assert(std::abs(q.z * q.z + q.w * q.w - 1.0) < 1e-6);

  auto missing = reg.resolve("garage");
  assert(missing.is_err());
  assert(missing.error().code == ErrorCode::kWaypointNotFound);
}

void test_dynamic_gbnf_builder() {
  WaypointRegistry empty_reg;
  assert(GbnfBuilder::build(empty_reg).is_err());

  WaypointRegistry reg;
  assert(reg.register_waypoint("kitchen", {1.0, 2.0, 0.0}).is_ok());
  assert(reg.register_waypoint("office", {-1.0, 0.5, 1.57}).is_ok());

  auto gbnf_res = GbnfBuilder::build(reg);
  assert(gbnf_res.is_ok());

  const std::string& gbnf = gbnf_res.value();
  assert(gbnf.find("root ::= nav-cmd | stop-cmd | unknown-cmd") != std::string::npos);
  assert(gbnf.find("waypoint ::= \"\\\"kitchen\\\"\" | \"\\\"office\\\"\"") != std::string::npos);
}

void test_strict_json_parser() {
  auto nav_ok = IntentJsonParser::parse(R"({"action":"navigate","target":"kitchen"})");
  assert(nav_ok.is_ok());
  assert(nav_ok.value().action == ActionType::kNavigate);
  assert(nav_ok.value().target_waypoint == "kitchen");

  auto stop_ok = IntentJsonParser::parse(R"({"action":"stop","target":"none"})");
  assert(stop_ok.is_ok());
  assert(stop_ok.value().action == ActionType::kStop);

  // Reject hallucinated extra fields or invalid combinations
  assert(IntentJsonParser::parse(R"({"action":"navigate","target":"none"})").is_err());
  assert(IntentJsonParser::parse(R"({"action":"stop","target":"kitchen"})").is_err());
  assert(IntentJsonParser::parse(R"({"action":"navigate","target":"kitchen","extra":1})").is_err());
}

void test_end_to_end_command_processor() {
  WaypointRegistry reg;
  assert(reg.register_waypoint("kitchen", {3.5, -1.2, 1.57}).is_ok());

  FakeIntentExtractor extractor;
  SpyNavigator navigator;

  auto proc_res = NavCommandProcessor::create(std::move(reg), extractor, &navigator);
  assert(proc_res.is_ok());
  auto& processor = proc_res.value();

  // 1) Navigate command
  extractor.next_json_response = R"({"action":"navigate","target":"kitchen"})";
  auto goal_res = processor.process("take me to the kitchen please");
  assert(goal_res.is_ok());
  assert(goal_res.value().action == ActionType::kNavigate);
  assert(goal_res.value().waypoint_id == "kitchen");
  assert(std::abs(goal_res.value().pose.x - 3.5) < 1e-6);
  assert(navigator.navigate_calls == 1);
  assert(navigator.cancel_calls == 0);

  // 2) Stop command
  extractor.next_json_response = R"({"action":"stop","target":"none"})";
  auto stop_res = processor.process("stop immediately");
  assert(stop_res.is_ok());
  assert(stop_res.value().action == ActionType::kStop);
  assert(navigator.cancel_calls == 1);

  // 3) Hallucinated waypoint not in registry is rejected before reaching Nav2
  extractor.next_json_response = R"({"action":"navigate","target":"rooftop"})";
  auto bad_res = processor.process("go to the rooftop");
  assert(bad_res.is_err());
  assert(bad_res.error().code == ErrorCode::kWaypointNotFound);
  assert(navigator.navigate_calls == 1);  // Unchanged
}

}  // namespace

int main() {
  test_yaml_loading_and_lookup();
  test_dynamic_gbnf_builder();
  test_strict_json_parser();
  test_end_to_end_command_processor();
  std::cout << "[PASS] All domain core unit tests passed.\n";
  return 0;
}
