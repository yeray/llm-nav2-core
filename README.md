# llm-nav2-core

Native **C++17** bridge connecting local Large Language Models powered by [`llama.cpp`](https://github.com/ggml-org/llama.cpp) to the **ROS 2 Navigation Stack (Nav2)**.

## Purpose

Translate natural language commands into precise navigation goals (`nav2_msgs/action/NavigateToPose`) in a **100% deterministic** manner. By constraining LLM logit sampling token-by-token through dynamically generated **GBNF grammars**, the output is mathematically guaranteed to conform to a strict JSON schema and valid map waypoints—eliminating syntactic hallucinations and invalid spatial coordinates while running entirely offline on **ARM64** hardware.

## Architecture (Pragmatic Hexagonal / Ports & Adapters)

1. **Domain Core (`llm_nav2_domain` — Pure C++17, zero external dependencies):**
   - [`WaypointRegistry`](include/llm_nav2_core/domain/waypoint_registry.hpp): Single source of truth mapping semantic location IDs (`kitchen`, `living_room`, etc.) from [`config/waypoints.yaml`](config/waypoints.yaml) to exact 2D planar poses `(x, y, yaw)`.
   - [`GbnfBuilder`](include/llm_nav2_core/domain/gbnf_builder.hpp): Dynamically compiles the GBNF grammar from registered waypoints at startup so the LLM cannot generate nonexistent locations.
   - [`NavCommandProcessor`](include/llm_nav2_core/domain/nav_command_processor.hpp): Orchestrates intent extraction, strict JSON parsing, spatial resolution, and navigation dispatch using explicit [`Result<T>`](include/llm_nav2_core/domain/result.hpp) error handling (no exceptions in control flow).
2. **Outbound / Driven Adapters:**
   - `LlamaCppExtractor` (`ports::IIntentExtractor`): RAII + Pimpl wrapper around `libllama` with `llama_sampler_init_grammar` + greedy decoding (`temperature = 0`).
   - `Nav2ActionClient` (`ports::INavigator`): `rclcpp_action` client dispatching `NavigateToPose` goals to Nav2.
3. **Inbound / Driving Adapters:**
   - `llm_nav2_cli`: Standalone CLI runner for fast local testing and benchmarking without booting ROS 2.
   - `llm_nav2_node`: ROS 2 node subscribing to natural language commands and driving Nav2.

## Build & Run Unit Tests (Core)

```bash
cmake -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Roadmap

- [x] Phase 0: Repository initialization and build configuration
- [x] Phase 1a: Pure C++17 Hexagonal Domain Core, dynamic GBNF builder, and unit tests
- [ ] Phase 1b: `llama.cpp` adapter (RAII + Pimpl) and standalone CLI runner
- [ ] Phase 2: ROS 2 node adapter (`rclcpp` + `NavigateToPose` action client)
- [ ] Phase 3: CI/CD pipeline (GitHub Actions)

## License

Licensed under the [Apache License, Version 2.0](LICENSE). See [NOTICE](NOTICE) for third-party attributions (`llama.cpp` under MIT, ROS 2 and Nav2 under Apache-2.0).
