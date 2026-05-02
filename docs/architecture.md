# Architecture Overview

The `visual-slam-cpp` project implements the perception front-end for a Visual Simultaneous Localization and Mapping (vSLAM) system. The core responsibility of this module is **Visual Odometry (VO)**: estimating the ego-motion of a camera across sequential image frames.

## Pipeline Breakdown

The visual odometry pipeline executes the following stages for each incoming frame:

1. **Feature Extraction:**
   - **Algorithm:** ORB (Oriented FAST and Rotated BRIEF).
   - **Purpose:** Identifies distinct, repeatable interest points (keypoints) in the image and computes a binary descriptor for each point. ORB is chosen for its real-time performance and invariance to rotation.

2. **Feature Matching:**
   - **Algorithm:** Brute-Force Matcher with `NORM_HAMMING` distance and K-Nearest Neighbors (KNN).
   - **Purpose:** Finds the best matching features between the previous frame $t-1$ and the current frame $t$. 

3. **Outlier Rejection:**
   - **Lowe's Ratio Test:** Filters out ambiguous matches by ensuring the best match is significantly better (distance-wise) than the second-best match.
   - **RANSAC (Random Sample Consensus):** Applied during the geometric estimation step to mathematically discard dynamic outliers (e.g., moving objects) and incorrectly matched features.

4. **Epipolar Geometry & Pose Recovery:**
   - **Essential Matrix Estimation:** Using the matched 2D points and the camera intrinsic matrix, the Essential Matrix $E$ is computed. This matrix encapsulates the geometric relationship between the two camera views.
   - **Pose Decomposition:** The Essential Matrix is decomposed into the relative Rotation matrix ($R \in SO(3)$) and Translation vector ($t \in \mathbb{R}^3$) using `cv::recoverPose`.

## Architectural Principles

### 1. Separation of Concerns
The architecture strictly enforces a boundary between Computer Vision routines and mathematical state estimation. 
- The pipeline ingests `cv::Mat` (OpenCV) objects.
- It outputs `Eigen::Matrix3d` and `Eigen::Vector3d` (Eigen) objects.
This prevents heavy OpenCV dependencies from polluting downstream modules like graph optimizers, bundle adjustment, or control algorithms.

### 2. State Management
The `VisualOdometry` class encapsulates the minimal state required to process the next frame:
- Previous frame keypoints
- Previous frame descriptors
- Initialization flag

This stateful but lightweight design allows the system to operate efficiently in real-time.

## Future Integration (Back-End)

The current repository represents the **Front-End Tracking Layer**. Future architecture expansions will include:
* **Local Mapping:** A windowed Bundle Adjustment (BA) module to jointly optimize camera poses and 3D map points.
* **Loop Closure:** A bag-of-words (BoW) approach to recognize previously visited locations and globally correct trajectory drift.
