# Contributing to Visual SLAM Front-End

We welcome contributions! The `visual-slam-cpp` project aims to build a robust, modular, and mathematically sound SLAM system.

## Getting Started

1. **Fork the Repository:** Create your own fork of the repository to make changes.
2. **Branch Strategy:** Create a feature branch off of the `main` branch.
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Draft your changes:** Make your code changes in your branch.
4. **Compile and Test:** Ensure your changes compile without warnings and that all `gtest` unit tests pass.

## Code Style & Conventions

To maintain a readable and consistent codebase, please adhere to the following guidelines:

* **C++ Standard:** The project uses **C++17**.
* **Naming Conventions:**
  * `CamelCase` for Classes and Structs (`VisualOdometry`).
  * `snake_case` for variables, functions, and methods (`process_frame()`, `prev_keypoints_`). *Note: Wait, our API uses `processFrame` right now, please conform to the existing API style (`camelCase` for methods).*
  * Member variables should end with an underscore (`is_initialized_`).
* **Documentation:**
  * All public APIs must be documented using Doxygen-style block comments in the header files (`.hpp`).
  * Add concise inline comments in `.cpp` files to explain *why* something is done, not *what*.
* **Memory Management:**
  * Prefer smart pointers (`std::unique_ptr`, `std::shared_ptr`, `cv::Ptr`) over raw pointers.
  * Minimize copies; utilize `std::move` when transferring ownership of large objects like feature descriptors or point clouds.
* **Separation of Concerns:**
  * DO NOT bleed OpenCV (`cv::Mat`) dependencies into the math or map layers. Use Eigen (`Eigen::Matrix`, `Eigen::Vector`) for geometric outputs.

## Development Status

Currently, the **Tracking** (Front-End) is implemented.
The following stubs exist and are high-priority for upcoming contributions:
- `include/vslam/tracker.hpp` / `src/tracker.cpp`: Will evolve to manage state tracking over time.
- `include/vslam/mapper.hpp` / `src/mapper.cpp`: Target files for the Back-End local bundle adjustment and mapping.

## Submitting a Pull Request

1. Commit your changes with descriptive messages.
2. Push your branch to your fork.
3. Open a Pull Request against the `main` branch of the original repository.
4. Ensure the CI pipeline passes (if configured). A maintainer will review your code.
