# Visual SLAM Front-End (C++)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](#)
[![License](https://img.shields.io/badge/license-MIT-blue)](#)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)](#)

A robust, mathematically rigorous Visual Odometry (VO) pipeline written in modern C++. This repository represents the perception front-end of a Simultaneous Localization and Mapping (vSLAM) system, engineered to track camera ego-motion across sequential video frames in real-time.

## 📖 Documentation

Detailed documentation is available in the [`docs/`](docs/) directory:

* **[Architecture Overview](docs/architecture.md)**: Deep dive into the pipeline (ORB, Epipolar Geometry, RANSAC).
* **[API Reference](docs/api_reference.md)**: Details on the `slam::VisualOdometry` class and how to integrate it.

## 🚀 Key Features

* **Strict Dependency Segregation:** The pipeline ingests frames using OpenCV (`cv::Mat`) but actively shields downstream modules by outputting relative poses using pure Eigen types (`Eigen::Matrix3d`, `Eigen::Vector3d`).
* **Real-Time Feature Tracking:** Utilizes ORB (Oriented FAST and Rotated BRIEF) coupled with a Hamming-distance Brute-Force Matcher.
* **Aggressive Outlier Rejection:** Applies Lowe's Ratio Test to discard ambiguous feature matches, followed by RANSAC during Epipolar geometry estimation to mathematically eliminate dynamic outliers.
* **Epipolar Geometry:** Robustly solves the Essential Matrix $E$ to recover the relative rotation $R$ and translation $t$ between frames.

## 🧩 Pipeline Overview

The visual odometry front-end follows a structured pipeline to estimate camera motion:

1.  **Feature Detection & Description**: ORB features are extracted from the current frame.
2.  **Feature Matching**: Hamming distance is used to match features with the previous frame.
3.  **Outlier Removal**: 
    *   **Lowe's Ratio Test** to filter matches.
    *   **RANSAC** during geometric estimation to handle dynamic objects and mismatches.
4.  **Motion Estimation**: 
    *   Compute the **Essential Matrix** from matches.
    *   Decompose the matrix into **Rotation (R)** and **Translation (t)**.
    *   Recover the pose relative to the previous frame.

## 🛠️ Build Instructions

This project uses **vcpkg** for dependency management to handle heavy-weight computer vision libraries deterministically across Windows, macOS, and Linux.

### Prerequisites

* CMake 3.15+
* C++17 compliant compiler (MSVC, GCC, or Clang)
* [vcpkg](https://vcpkg.io/) installed and bootstrapped

### Dependencies

Managed automatically via `vcpkg.json`:
* `opencv4` (Computer Vision)
* `eigen3` (Linear Algebra)
* `gtest` (Unit Testing)

### Compilation

```bash
# 1. Clone the repository
git clone https://github.com/your-username/visual-slam-cpp.git
cd visual-slam-cpp

# 2. Configure the project with the vcpkg toolchain
#    Replace <path-to-vcpkg> with your actual vcpkg installation path
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="<path-to-vcpkg>/scripts/buildsystems/vcpkg.cmake"

# 3. Compile the project
cmake --build build --config Release
```

## 🧪 Testing

The system is rigorously tested using Google Test (`gtest`). Ensure you run tests after building to verify the numerical stability of the VO pipeline.

```bash
cd build
ctest -C Release --output-on-failure
```

## 🛣️ Roadmap / Future Work

* **Local Mapper Integration:** Implementing windowed Bundle Adjustment (BA) to jointly optimize camera poses and 3D map points.
* **Loop Closure Detection:** Integrating Bag-of-Words (BoW) place recognition to eliminate accumulated trajectory drift.
* **Stereo/RGB-D Support:** Extending the pipeline beyond Monocular cameras to resolve absolute scale ambiguity inherently.
