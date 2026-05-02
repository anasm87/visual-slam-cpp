#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <opencv2/features2d.hpp>

namespace slam {

/**
 * @brief Represents the relative pose (rotation and translation) between two frames.
 * 
 * This structure encapsulates the 3D geometry recovery from the Essential Matrix.
 * The transformation maps 3D points from the current camera coordinate frame
 * to the previous camera coordinate frame: \f$ X_{prev} = R \cdot X_{curr} + t \f$.
 */
struct RelativePose {
    Eigen::Matrix3d R; ///< 3x3 Rotation matrix (SO(3))
    Eigen::Vector3d t; ///< 3x1 Translation vector
};

/**
 * @class VisualOdometry
 * @brief Core pipeline for the Visual SLAM Front-End.
 * 
 * The VisualOdometry class processes sequential camera frames to estimate the 
 * relative ego-motion of the camera. It utilizes ORB feature extraction, 
 * K-Nearest Neighbors matching, Lowe's Ratio Test, and RANSAC-based 
 * Essential Matrix estimation to robustly recover relative rotation and translation.
 * 
 * @note This implementation actively shields downstream modules from OpenCV 
 * dependencies by outputting pure Eigen types for mathematical processing.
 */
class VisualOdometry {
public:
    /**
     * @brief Constructs a new VisualOdometry object.
     * 
     * Initializes the ORB feature detector and Brute-Force Matcher (with Hamming distance).
     * 
     * @param cameraMatrix 3x3 Camera Intrinsic Matrix (cv::Mat of type CV_64F or CV_32F).
     *                     Contains focal lengths (fx, fy) and principal point (cx, cy).
     * @throws std::invalid_argument if the camera matrix is empty or not 3x3.
     */
    explicit VisualOdometry(const cv::Mat& cameraMatrix);

    /**
     * @brief Processes a sequential camera frame and estimates the relative pose.
     * 
     * The processing pipeline includes:
     * 1. Detecting ORB keypoints and computing descriptors.
     * 2. Matching descriptors against the previous frame using KNN.
     * 3. Filtering outliers using Lowe's ratio test.
     * 4. Estimating the Essential Matrix with RANSAC.
     * 5. Recovering the relative pose from the Essential Matrix.
     * 
     * @param frame Current image frame (grayscale or color).
     * @return std::optional<RelativePose> The relative rotation (R) and translation (t) 
     *         if tracking is successful, or std::nullopt if tracking fails 
     *         (e.g., insufficient features or inliers).
     */
    std::optional<RelativePose> processFrame(const cv::Mat& frame);

private:
    cv::Mat K_; ///< Camera Intrinsic Matrix (CV_64F)
    
    // Previous frame state
    bool is_initialized_;                      ///< True if the pipeline has processed at least one frame
    std::vector<cv::KeyPoint> prev_keypoints_; ///< Keypoints detected in the previous frame
    cv::Mat prev_descriptors_;                 ///< Descriptors computed in the previous frame
    
    cv::Ptr<cv::Feature2D> feature_detector_;       ///< ORB feature detector and descriptor extractor
    cv::Ptr<cv::DescriptorMatcher> feature_matcher_; ///< Brute-Force matcher utilizing NORM_HAMMING
};

} // namespace slam
