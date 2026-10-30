# llm-nav2-core

Nodo puente nativo en **C++** que conecta modelos de lenguaje locales ejecutados con [`llama.cpp`](https://github.com/ggml-org/llama.cpp) al stack de navegación de **ROS 2 (Nav2)**.

## Objetivo

Traducir comandos en lenguaje natural a metas de navegación precisas (`nav2_msgs/action/NavigateToPose`) de forma **100% determinista**, operando íntegramente en local sobre hardware **ARM64** sin depender de APIs en la nube.

## Arquitectura Base (Enfoque Determinista)

1. **Inferencia Local (`llama.cpp` C/C++ API):** Carga directa de modelos cuantizados (`.gguf`) en memoria con aceleración ARM NEON.
2. **Restricción Sintáctica y Semántica (Gramáticas GBNF):** El muestreo de logits se restringe token a token mediante una gramática GBNF estricta y decodificación *greedy* (`temperature = 0`), garantizando matemáticamente una salida JSON válida y acotada a waypoints conocidos del mapa.
3. **Resolución Espacial y Acción Nav2 (`rclcpp_action`):** El nodo C++ traduce la intención semántica validada a coordenadas exactas `(x, y, yaw)` del mapa topológico y ejecuta la meta `NavigateToPose` en Nav2.

## Estado del Proyecto

- [x] Inicialización del repositorio y configuración base
- [ ] Fase 1: Core C++ determinista (`llama.cpp` + sampler GBNF standalone)
- [ ] Fase 2: Nodo puente ROS 2 (`rclcpp` + cliente `NavigateToPose`)
- [ ] Fase 3: Pruebas unitarias e integración continua
