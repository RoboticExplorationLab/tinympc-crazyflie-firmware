# TinyMPC IROS 2024 Paper Codebase

# Updates:

- SOCP
  - Embedded SOCP solver that also handles quadratic cost functions (unlike ECOS, which is one of the only other embedded SOCP solvers)
  - Rocket landing demo
  - Safety filter demo
  - Quadrotor field of view demo
  - Quadruped balancing demo
- Codegen support
  - [Python interface](https://github.com/TinyMPC/tinympc-python)
  - [Julia interface](https://github.com/TinyMPC/tinympc-julia)
  - [MATLAB interface](https://github.com/TinyMPC/tinympc-matlab)
  - Compiles C++ code that can be tested through the interface with C calls or copy-pasted into an embedded project and compiled and flashed to a board