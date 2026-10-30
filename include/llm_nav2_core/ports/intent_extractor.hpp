#pragma once

#include <string>
#include <string_view>

#include "llm_nav2_core/domain/result.hpp"

namespace llm_nav2_core::ports {

class IIntentExtractor {
public:
  virtual ~IIntentExtractor() = default;

  // Extracts a strict JSON string constrained by `gbnf_grammar` from `user_prompt`.
  virtual domain::Result<std::string> extract_json(
      std::string_view user_prompt,
      std::string_view gbnf_grammar) = 0;
};

}  // namespace llm_nav2_core::ports
