#include "llm_nav2_core/domain/nav_command_processor.hpp"

#include "llm_nav2_core/domain/gbnf_builder.hpp"

namespace llm_nav2_core::domain {

NavCommandProcessor::NavCommandProcessor(
    WaypointRegistry registry,
    std::string gbnf_grammar,
    ports::IIntentExtractor& extractor,
    ports::INavigator* navigator)
    : registry_(std::move(registry)),
      gbnf_grammar_(std::move(gbnf_grammar)),
      extractor_(extractor),
      navigator_(navigator) {}

Result<NavCommandProcessor> NavCommandProcessor::create(
    WaypointRegistry registry,
    ports::IIntentExtractor& extractor,
    ports::INavigator* navigator) {
  auto grammar_res = GbnfBuilder::build(registry);
  if (grammar_res.is_err()) {
    return Result<NavCommandProcessor>::Err(
        grammar_res.error().code, grammar_res.error().message);
  }

  return Result<NavCommandProcessor>::Ok(NavCommandProcessor(
      std::move(registry),
      std::move(grammar_res).value(),
      extractor,
      navigator));
}

Result<NavGoal> NavCommandProcessor::process(std::string_view user_command) {
  if (user_command.empty()) {
    return Result<NavGoal>::Err(
        ErrorCode::kInvalidArgument, "User command cannot be empty");
  }

  auto json_res = extractor_.extract_json(user_command, gbnf_grammar_);
  if (json_res.is_err()) {
    return Result<NavGoal>::Err(json_res.error().code, json_res.error().message);
  }

  auto intent_res = IntentJsonParser::parse(json_res.value());
  if (intent_res.is_err()) {
    return Result<NavGoal>::Err(intent_res.error().code, intent_res.error().message);
  }

  const NavIntent& intent = intent_res.value();

  if (intent.action == ActionType::kStop) {
    if (navigator_ != nullptr) {
      auto cancel_res = navigator_->cancel_navigation();
      if (cancel_res.is_err()) {
        return Result<NavGoal>::Err(cancel_res.error().code, cancel_res.error().message);
      }
    }
    return Result<NavGoal>::Ok(NavGoal{ActionType::kStop, "none", Pose2D{}});
  }

  if (intent.action == ActionType::kUnknown) {
    return Result<NavGoal>::Ok(NavGoal{ActionType::kUnknown, "none", Pose2D{}});
  }

  auto pose_res = registry_.resolve(intent.target_waypoint);
  if (pose_res.is_err()) {
    return Result<NavGoal>::Err(pose_res.error().code, pose_res.error().message);
  }

  NavGoal goal{
      ActionType::kNavigate,
      intent.target_waypoint,
      pose_res.value()};

  if (navigator_ != nullptr) {
    auto nav_res = navigator_->navigate_to(goal);
    if (nav_res.is_err()) {
      return Result<NavGoal>::Err(nav_res.error().code, nav_res.error().message);
    }
  }

  return Result<NavGoal>::Ok(std::move(goal));
}

}  // namespace llm_nav2_core::domain
