#ifndef __FAST_CV_H__
#define __FAST_CV_H__

#include "fast.h"
#include <opencv2/opencv.hpp>

namespace fastcv
{

static inline void fast_xy_2KeyPoint(const fast::fast_xy& fast_xy, cv::KeyPoint& keypoint)
{
    keypoint.pt.x = fast_xy.x;
    keypoint.pt.y = fast_xy.y;
}

static inline void fast_xy_2KeyPoints(const std::vector<fast::fast_xy> corners, std::vector<cv::KeyPoint>& keypoints)
{
    for (uint32_t i = 0; i < corners.size(); i++)
    {
		cv::KeyPoint keypoint;
        keypoint.pt.x = corners[i].x;
        keypoint.pt.y = corners[i].y;
        keypoints.push_back(keypoint);
    }
}

static inline void keyPoint2fast_xy(fast::fast_xy& fast_xy, const cv::KeyPoint& keypoint)
{
    fast_xy.x = keypoint.pt.x;
    fast_xy.y = keypoint.pt.y;
}

static inline void keyPoints2fast_xy(std::vector<fast::fast_xy> corners, const std::vector<cv::KeyPoint>& keypoints)
{
    for (uint32_t i = 0; i < corners.size(); i++)
    {
        fast::fast_xy corner(0, 0);
        corner.x = keypoints[i].pt.x;
        corner.y = keypoints[i].pt.y;
        corners.push_back(corner);
    }
}

/// SSE2 optimized version of the corner 10
static inline void keypoint_detect_10_sse2(const cv::Mat& img, short barrier, std::vector<cv::KeyPoint>& keypoints)
{
    std::vector<fast::fast_xy> corners;
    fast::fast_corner_detect_10_sse2((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, barrier, corners);
    fast_xy_2KeyPoints(corners, keypoints);
}    

/// plain C++ version of the corner 10
static inline void keypoint_detect_10(const cv::Mat& img, short barrier, std::vector<cv::KeyPoint>& keypoints)
{
    std::vector<fast::fast_xy> corners;
    fast::fast_corner_detect_10((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, barrier, corners);
    fast_xy_2KeyPoints(corners, keypoints);
} 

/// corner score 10
static inline void keypoint_score_10(const cv::Mat& img, const int img_stride, std::vector<cv::KeyPoint>& keypoints, const int threshold, std::vector<int>& scores)
{
    std::vector<fast::fast_xy> corners;
    keyPoints2fast_xy(corners, keypoints);
    fast::fast_corner_score_10((fast::fast_byte *)(img.data), img_stride, corners, threshold, scores);
}

/// Nonmax Suppression on a 3x3 Window
static inline void nonmax_3x3(const std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& scores, std::vector<int>& nonmax_corners)
{
    std::vector<fast::fast_xy> corners;
    keyPoints2fast_xy(corners, keypoints);
    fast::fast_nonmax_3x3(corners, scores, nonmax_corners);
}

/// NEON optimized version of the corner 9
static inline void keypoint_detect_9_neon(const cv::Mat& img, short barrier, std::vector<cv::KeyPoint>& keypoints)
{
    std::vector<fast::fast_xy> corners;
    fast::fast_corner_detect_9_neon((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, barrier, corners);
    fast_xy_2KeyPoints(corners, keypoints);    
}

} // namespace fast

#endif

