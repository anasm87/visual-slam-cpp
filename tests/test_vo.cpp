#include <gtest/gtest.h>
#include "slam/visual_odometry.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <random>

using namespace slam;

cv::Mat renderPoints(const std::vector<cv::Point3f>& pts, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t) {
    cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts, rvec, t, K, cv::noArray(), pts2d);
    
    for (const auto& p : pts2d) {
        if (p.x >= 15 && p.x < 625 && p.y >= 15 && p.y < 465) {
            // Draw a square with a distinct corner pattern
            int cx = static_cast<int>(p.x);
            int cy = static_cast<int>(p.y);
            cv::rectangle(img, cv::Rect(cx - 10, cy - 10, 20, 20), cv::Scalar(255), cv::FILLED);
            cv::rectangle(img, cv::Rect(cx - 5, cy - 5, 10, 10), cv::Scalar(0), cv::FILLED);
        }
    }
    
    // Add some blur to make it slightly more realistic for feature detection
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
    return img;
}

TEST(VisualOdometryTest, PureTranslationHorizontal) {
    cv::Mat K = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    VisualOdometry vo(K);
    
    // Generate 3D points
    std::vector<cv::Point3f> points3d;
    std::mt19937 gen(42); // fixed seed for determinism
    std::uniform_real_distribution<float> x_dist(-3.0f, 3.0f);
    std::uniform_real_distribution<float> y_dist(-3.0f, 3.0f);
    std::uniform_real_distribution<float> z_dist(5.0f, 15.0f);
    
    for (int i = 0; i < 150; i++) {
        points3d.push_back(cv::Point3f(x_dist(gen), y_dist(gen), z_dist(gen)));
    }
    
    // Frame 1: Identity pose
    cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t1 = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat img1 = renderPoints(points3d, K, R1, t1);
    
    auto pose1 = vo.processFrame(img1);
    EXPECT_FALSE(pose1.has_value());
    
    // Frame 2: Translated by tx = -0.5 (Camera moved right)
    cv::Mat R2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t2 = (cv::Mat_<double>(3, 1) << -0.5, 0.0, 0.0);
    cv::Mat img2 = renderPoints(points3d, K, R2, t2);
    
    auto pose2 = vo.processFrame(img2);
    ASSERT_TRUE(pose2.has_value());
    
    // Check direction of movement
    // t2 is normalized by recoverPose, so it should be approximately (-1, 0, 0)
    EXPECT_LT(pose2->t.x(), -0.8);
    EXPECT_LT(std::abs(pose2->t.y()), 0.2);
    EXPECT_LT(std::abs(pose2->t.z()), 0.2);
    
    // Rotation should be identity
    Eigen::Matrix3d R_expected = Eigen::Matrix3d::Identity();
    EXPECT_LT((pose2->R - R_expected).norm(), 0.1);
}

TEST(VisualOdometryTest, InsufficientFeatures) {
    cv::Mat K = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    VisualOdometry vo(K);
    
    // Blank images
    cv::Mat img1 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat img2 = cv::Mat::zeros(480, 640, CV_8UC1);
    
    auto pose1 = vo.processFrame(img1);
    EXPECT_FALSE(pose1.has_value());
    
    auto pose2 = vo.processFrame(img2);
    EXPECT_FALSE(pose2.has_value());
}
