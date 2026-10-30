#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "llm_nav2_core/domain/result.hpp"
#include "llm_nav2_core/domain/types.hpp"

namespace llm_nav2_core::domain {

class WaypointRegistry {
public:
  WaypointRegistry() = default;

  Result<bool> register_waypoint(std::string id, Pose2D pose);

  [[nodiscard]] Result<Pose2D> resolve(std::string_view id) const;

  [[nodiscard]] std::vector<std::string> waypoint_ids() const;

  [[nodiscard]] std::size_t size() const noexcept { return waypoints_.size(); }
  [[nodiscard]] bool empty() const noexcept { return waypoints_.empty(); }

  static Result<WaypointRegistry> load_from_yaml_file(const std::string& path);
  static Result<WaypointRegistry> load_from_yaml_string(std::string_view yaml_content);

private:
  std::map<std::string, Pose2D, std::less<>> waypoints_;
};

}  // namespace llm_nav2_core::domain
