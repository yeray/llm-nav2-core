#pragma once

#include "llm_nav2_core/domain/result.hpp"
#include "llm_nav2_core/domain/types.hpp"

namespace llm_nav2_core::ports {

class INavigator {
public:
  virtual ~INavigator() = default;

  virtual domain::Result<bool> navigate_to(const domain::NavGoal& goal) = 0;
  virtual domain::Result<bool> cancel_navigation() = 0;
};

}  // namespace llm_nav2_core::ports
