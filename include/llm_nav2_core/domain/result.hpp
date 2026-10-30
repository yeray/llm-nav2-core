#pragma once

#include <string>
#include <utility>
#include <variant>

namespace llm_nav2_core::domain {

enum class ErrorCode {
  kInvalidArgument,
  kWaypointNotFound,
  kEmptyRegistry,
  kJsonParseError,
  kInferenceError,
  kNavigationError,
  kIoError
};

struct Error {
  ErrorCode code;
  std::string message;
};

template <typename T>
class Result {
public:
  static Result Ok(T value) { return Result(std::move(value)); }

  static Result Err(ErrorCode code, std::string message) {
    return Result(Error{code, std::move(message)});
  }

  [[nodiscard]] bool is_ok() const noexcept {
    return std::holds_alternative<T>(storage_);
  }

  [[nodiscard]] bool is_err() const noexcept { return !is_ok(); }

  [[nodiscard]] const T& value() const& { return std::get<T>(storage_); }
  [[nodiscard]] T& value() & { return std::get<T>(storage_); }
  [[nodiscard]] T&& value() && { return std::get<T>(std::move(storage_)); }

  [[nodiscard]] const Error& error() const& {
    return std::get<Error>(storage_);
  }

private:
  explicit Result(T value) : storage_(std::move(value)) {}
  explicit Result(Error error) : storage_(std::move(error)) {}

  std::variant<T, Error> storage_;
};

}  // namespace llm_nav2_core::domain
