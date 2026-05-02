#include "slam/visual_odometry.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

namespace slam {

VisualOdometry::VisualOdometry(const cv::Mat& cameraMatrix)
    : is_initialized_(false) {
    if (cameraMatrix.empty() || cameraMatrix.rows != 3 || cameraMatrix.cols != 3) {
        throw std::invalid_argument("Camera matrix must be a 3x3 cv::Mat");
    }
    cameraMatrix.convertTo(K_, CV_64F);

    // Initialize ORB detector
    feature_detector_ = cv::ORB::create();
    
    // Initialize BFMatcher with Hamming distance for ORB
    feature_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
}

std::optional<RelativePose> VisualOdometry::processFrame(const cv::Mat& frame) {
    if (frame.empty()) {
        return std::nullopt;
    }

    std::vector<cv::KeyPoint> curr_keypoints;
    cv::Mat curr_descriptors;
    feature_detector_->detectAndCompute(frame, cv::noArray(), curr_keypoints, curr_descriptors);

    if (curr_descriptors.empty() || curr_keypoints.size() < 8U) {
        // Not enough features to proceed, update state for next frame and return nullopt
        prev_keypoints_ = std::move(curr_keypoints);
        prev_descriptors_ = std::move(curr_descriptors);
        return std::nullopt;
    }

    if (!is_initialized_) {
        prev_keypoints_ = std::move(curr_keypoints);
        prev_descriptors_ = std::move(curr_descriptors);
        is_initialized_ = true;
        return std::nullopt; // Need at least two frames to compute relative pose
    }

    // Match descriptors using KNN
    std::vector<std::vector<cv::DMatch>> knn_matches;
    if (prev_descriptors_.empty()) {
        prev_keypoints_ = std::move(curr_keypoints);
        prev_descriptors_ = std::move(curr_descriptors);
        return std::nullopt;
    }
    
    feature_matcher_->knnMatch(prev_descriptors_, curr_descriptors, knn_matches, 2);

    // Apply Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.8f;
    for (const auto& match : knn_matches) {
        if (match.size() >= 2U) {
            if (match[0].distance < ratio_thresh * match[1].distance) {
                good_matches.push_back(match[0]);
            }
        } else if (match.size() == 1U) {
            good_matches.push_back(match[0]);
        }
    }

    // Minimum 8 matches required for Essential matrix
    if (good_matches.size() < 8U) {
        prev_keypoints_ = std::move(curr_keypoints);
        prev_descriptors_ = std::move(curr_descriptors);
        return std::nullopt;
    }

    // Extract aligned point sets
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(good_matches.size());
    pts2.reserve(good_matches.size());
    for (const auto& m : good_matches) {
        pts1.push_back(prev_keypoints_[m.queryIdx].pt);
        pts2.push_back(curr_keypoints[m.trainIdx].pt);
    }

    // Compute Essential Matrix
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K_, cv::RANSAC, 0.999, 1.0, mask);

    if (E.empty()) {
        prev_keypoints_ = std::move(curr_keypoints);
        prev_descriptors_ = std::move(curr_descriptors);
        return std::nullopt;
    }

    // Recover Pose (Rotation and Translation)
    cv::Mat R, t;
    int inliers = cv::recoverPose(E, pts1, pts2, K_, R, t, mask);

    // Update state for the next frame
    prev_keypoints_ = std::move(curr_keypoints);
    prev_descriptors_ = std::move(curr_descriptors);

    // Ensure we have enough inliers to trust the pose
    if (inliers < 5) {
        return std::nullopt;
    }

    // Convert to Eigen matrices
    RelativePose pose;
    cv::cv2eigen(R, pose.R);
    cv::cv2eigen(t, pose.t);

    return pose;
}

} // namespace slam