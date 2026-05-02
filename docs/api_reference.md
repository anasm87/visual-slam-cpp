# API Reference

The core functionality of the Visual SLAM front-end is exposed through the `slam::VisualOdometry` class. This section outlines the primary data structures and methods available to developers.

## Data Structures

### `slam::RelativePose`
Encapsulates the 3D geometric transformation from the current camera frame to the previous camera frame.

```cpp
struct RelativePose {
    Eigen::Matrix3d R; // 3x3 Rotation matrix (Special Orthogonal Group SO(3))
    Eigen::Vector3d t; // 3x1 Translation vector
};
```
* **`R`**: Rotation matrix mapping points from frame $t$ to frame $t-1$.
* **`t`**: Translation vector representing the displacement between the optical centers.

## Core Classes

### `slam::VisualOdometry`
The stateful pipeline processor that ingests image frames and outputs relative ego-motion.

#### Constructor
```cpp
explicit VisualOdometry(const cv::Mat& cameraMatrix);
```
* **Parameters**: 
  * `cameraMatrix`: A 3x3 OpenCV Matrix (`cv::Mat` of type `CV_64F` or `CV_32F`) representing the camera intrinsics (focal length and principal point).
* **Throws**: `std::invalid_argument` if the matrix is improperly sized or empty.

#### Process Frame
```cpp
std::optional<RelativePose> processFrame(const cv::Mat& frame);
```
* **Parameters**:
  * `frame`: The incoming camera image (supports grayscale or color `cv::Mat`).
* **Returns**: 
  * `std::optional<RelativePose>`: Contains the valid `R` and `t` matrices if tracking succeeds.
  * Returns `std::nullopt` in the following scenarios:
    * The frame is empty.
    * It is the first frame (initialization phase).
    * Insufficient features are detected or matched (requires at least 8 inliers for the Essential Matrix).
    * RANSAC fails to find a geometric consensus.

## Usage Example

```cpp
#include "slam/visual_odometry.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 1. Define Camera Intrinsics
    cv::Mat K = (cv::Mat_<double>(3,3) << 
        718.856, 0.0, 607.192,
        0.0, 718.856, 185.215,
        0.0, 0.0, 1.0);

    // 2. Instantiate the Pipeline
    slam::VisualOdometry vo(K);

    // 3. Process Video Feed
    cv::VideoCapture cap("dataset/sequence.mp4");
    cv::Mat frame;

    while (cap.read(frame)) {
        auto pose_opt = vo.processFrame(frame);

        if (pose_opt.has_value()) {
            std::cout << "Rotation:\n" << pose_opt->R << "\n";
            std::cout << "Translation:\n" << pose_opt->t.transpose() << "\n";
        } else {
            std::cerr << "Tracking lost or initializing...\n";
        }
    }
    return 0;
}
```
