#include "llm_nav2_core/domain/waypoint_registry.hpp"

#include <cctype>
#include <fstream>
#include <sstream>

namespace llm_nav2_core::domain {

namespace {

std::string trim(std::string_view sv) {
  std::size_t start = 0;
  while (start < sv.size() && std::isspace(static_cast<unsigned char>(sv[start]))) {
    ++start;
  }
  std::size_t end = sv.size();
  while (end > start && std::isspace(static_cast<unsigned char>(sv[end - 1]))) {
    --end;
  }
  return std::string(sv.substr(start, end - start));
}

bool is_valid_identifier(std::string_view id) {
  if (id.empty()) {
    return false;
  }
  for (char c : id) {
    const bool ok = std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
    if (!ok) {
      return false;
    }
  }
  return true;
}

}  // namespace

Result<bool> WaypointRegistry::register_waypoint(std::string id, Pose2D pose) {
  if (!is_valid_identifier(id)) {
    return Result<bool>::Err(
        ErrorCode::kInvalidArgument,
        "Waypoint ID must be non-empty alphanumeric/underscore/hyphen: '" + id + "'");
  }
  if (id == "none") {
    return Result<bool>::Err(
        ErrorCode::kInvalidArgument,
        "'none' is a reserved target identifier");
  }
  waypoints_[std::move(id)] = pose;
  return Result<bool>::Ok(true);
}

Result<Pose2D> WaypointRegistry::resolve(std::string_view id) const {
  auto it = waypoints_.find(id);
  if (it == waypoints_.end()) {
    return Result<Pose2D>::Err(
        ErrorCode::kWaypointNotFound,
        "Waypoint not found in registry: '" + std::string(id) + "'");
  }
  return Result<Pose2D>::Ok(it->second);
}

std::vector<std::string> WaypointRegistry::waypoint_ids() const {
  std::vector<std::string> ids;
  ids.reserve(waypoints_.size());
  for (const auto& [id, _] : waypoints_) {
    ids.push_back(id);
  }
  return ids;
}

Result<WaypointRegistry> WaypointRegistry::load_from_yaml_file(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return Result<WaypointRegistry>::Err(
        ErrorCode::kIoError, "Failed to open waypoints YAML file: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return load_from_yaml_string(buffer.str());
}

Result<WaypointRegistry> WaypointRegistry::load_from_yaml_string(std::string_view yaml_content) {
  WaypointRegistry registry;
  std::istringstream stream{std::string(yaml_content)};
  std::string line;
  std::string current_wp;
  Pose2D current_pose{};
  bool in_waypoints_section = false;
  bool has_active_wp = false;

  auto flush_active = [&]() -> Result<bool> {
    if (!has_active_wp) {
      return Result<bool>::Ok(true);
    }
    auto res = registry.register_waypoint(current_wp, current_pose);
    current_wp.clear();
    current_pose = Pose2D{};
    has_active_wp = false;
    return res;
  };

  while (std::getline(stream, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    std::string trimmed = trim(line);
    if (trimmed.empty()) {
      continue;
    }

    if (trimmed == "waypoints:") {
      in_waypoints_section = true;
      continue;
    }
    if (!in_waypoints_section) {
      continue;
    }

    const auto colon_pos = trimmed.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string key = trim(trimmed.substr(0, colon_pos));
    std::string val = trim(trimmed.substr(colon_pos + 1));

    if (val.empty()) {
      auto flush_res = flush_active();
      if (flush_res.is_err()) {
        return Result<WaypointRegistry>::Err(
            flush_res.error().code, flush_res.error().message);
      }
      current_wp = key;
      has_active_wp = true;
    } else if (has_active_wp) {
      try {
        const double number = std::stod(val);
        if (key == "x") {
          current_pose.x = number;
        } else if (key == "y") {
          current_pose.y = number;
        } else if (key == "yaw") {
          current_pose.yaw = number;
        }
      } catch (...) {
        return Result<WaypointRegistry>::Err(
            ErrorCode::kInvalidArgument,
            "Invalid numeric value for '" + key + "' in waypoint '" + current_wp + "'");
      }
    }
  }

  auto flush_res = flush_active();
  if (flush_res.is_err()) {
    return Result<WaypointRegistry>::Err(
        flush_res.error().code, flush_res.error().message);
  }

  if (registry.empty()) {
    return Result<WaypointRegistry>::Err(
        ErrorCode::kEmptyRegistry, "No valid waypoints found in YAML content");
  }

  return Result<WaypointRegistry>::Ok(std::move(registry));
}

}  // namespace llm_nav2_core::domain
