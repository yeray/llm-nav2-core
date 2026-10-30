#include "llm_nav2_core/domain/gbnf_builder.hpp"

#include <sstream>

namespace llm_nav2_core::domain {

Result<std::string> GbnfBuilder::build(const WaypointRegistry& registry) {
  const auto ids = registry.waypoint_ids();
  if (ids.empty()) {
    return Result<std::string>::Err(
        ErrorCode::kEmptyRegistry,
        "Cannot build GBNF grammar from an empty WaypointRegistry");
  }

  std::ostringstream oss;
  oss << "root ::= nav-cmd | stop-cmd | unknown-cmd\n";
  oss << "nav-cmd ::= \"{\\\"action\\\":\\\"navigate\\\",\\\"target\\\":\" waypoint \"}\"\n";
  oss << "stop-cmd ::= \"{\\\"action\\\":\\\"stop\\\",\\\"target\\\":\\\"none\\\"}\"\n";
  oss << "unknown-cmd ::= \"{\\\"action\\\":\\\"unknown\\\",\\\"target\\\":\\\"none\\\"}\"\n";
  oss << "waypoint ::= ";

  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i > 0) {
      oss << " | ";
    }
    oss << "\"\\\"" << ids[i] << "\\\"\"";
  }
  oss << "\n";

  return Result<std::string>::Ok(oss.str());
}

namespace {

Result<std::string> extract_string_field(std::string_view json, std::string_view key) {
  const std::string pattern = "\"" + std::string(key) + "\":\"";
  const auto pos = json.find(pattern);
  if (pos == std::string_view::npos) {
    return Result<std::string>::Err(
        ErrorCode::kJsonParseError,
        "Missing or malformed field '" + std::string(key) + "' in JSON");
  }
  const auto val_start = pos + pattern.size();
  const auto val_end = json.find('"', val_start);
  if (val_end == std::string_view::npos) {
    return Result<std::string>::Err(
        ErrorCode::kJsonParseError,
        "Unterminated string value for field '" + std::string(key) + "'");
  }
  return Result<std::string>::Ok(std::string(json.substr(val_start, val_end - val_start)));
}

}  // namespace

Result<NavIntent> IntentJsonParser::parse(std::string_view json_str) {
  if (json_str.size() < 2 || json_str.front() != '{' || json_str.back() != '}') {
    return Result<NavIntent>::Err(
        ErrorCode::kJsonParseError, "JSON payload must start with '{' and end with '}'");
  }

  auto action_res = extract_string_field(json_str, "action");
  if (action_res.is_err()) {
    return Result<NavIntent>::Err(action_res.error().code, action_res.error().message);
  }

  auto target_res = extract_string_field(json_str, "target");
  if (target_res.is_err()) {
    return Result<NavIntent>::Err(target_res.error().code, target_res.error().message);
  }

  const std::string& action_str = action_res.value();
  const std::string& target_str = target_res.value();

  // Verify strict canonical formatting: {"action":"<action>","target":"<target>"}
  const std::string canonical =
      "{\"action\":\"" + action_str + "\",\"target\":\"" + target_str + "\"}";
  if (json_str != canonical) {
    return Result<NavIntent>::Err(
        ErrorCode::kJsonParseError,
        "JSON does not match strict canonical GBNF schema");
  }

  NavIntent intent;
  intent.target_waypoint = target_str;

  if (action_str == "navigate") {
    if (target_str.empty() || target_str == "none") {
      return Result<NavIntent>::Err(
          ErrorCode::kJsonParseError,
          "Action 'navigate' requires a non-empty target waypoint other than 'none'");
    }
    intent.action = ActionType::kNavigate;
  } else if (action_str == "stop") {
    if (target_str != "none") {
      return Result<NavIntent>::Err(
          ErrorCode::kJsonParseError, "Action 'stop' must have target 'none'");
    }
    intent.action = ActionType::kStop;
  } else if (action_str == "unknown") {
    if (target_str != "none") {
      return Result<NavIntent>::Err(
          ErrorCode::kJsonParseError, "Action 'unknown' must have target 'none'");
    }
    intent.action = ActionType::kUnknown;
  } else {
    return Result<NavIntent>::Err(
        ErrorCode::kJsonParseError, "Unsupported action type: '" + action_str + "'");
  }

  return Result<NavIntent>::Ok( std::move(intent));
}

}  // namespace llm_nav2_core::domain
