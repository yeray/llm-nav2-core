#pragma once

#include <cmath>
#include <string>

namespace llm_nav2_core::domain {

struct Quaternion2D {
  double z{0.0};
  double w{1.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};  // radians

  [[nodiscard]] Quaternion2D to_quaternion() const noexcept {
    return Quaternion2D{std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
  }
};

enum class ActionType {
  kNavigate,
  kStop,
  kUnknown
};

struct NavIntent {
  ActionType action{ActionType::kUnknown};
  std::string target_waypoint;
};

struct NavGoal {
  ActionType action{ActionType::kUnknown};
  std::string waypoint_id;
  Pose2D pose{};
};

}  // namespace llm_nav2_core::domain
