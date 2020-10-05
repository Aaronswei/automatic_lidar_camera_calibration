/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include "../Types.hpp"
#include "PointCloudFilter.hpp"

namespace perception
{
inline cv::Scalar colorCodingDepthBGR(const int depth)
{
    std::uint8_t r, g, b;
    if (depth < 0) {
        // magnenta
        r = 255;
        g = 0;
        b = 255;
    } else if (depth < 2) {
        r = 0;
        g = static_cast<uint8_t>((depth - 0) / 50 * 255) & 0xff;
        b = 0xff;
    } else if (depth < 4) {
        r = 0;
        g = 0xff;
        b = 0xff - (static_cast<uint8_t>((depth - 50) / 70 * 255) & 0xff);
    } else if (depth < 6) {
        r = static_cast<uint8_t>((depth - 120) / 80 * 255) & 0xff;
        g = 0xff;
        b = 0;
    } else if (depth < 8) {
        r = 0xff;
        g = 0xff - (static_cast<uint8_t>((depth - 200) / 150 * 255) & 0xff);
        b = 0;
    } else if (depth < 10) {
        r = 0xff;
        g = 0x00;
        b = static_cast<uint8_t>((depth - 350) / 150 * 255) & 0xff;
    } else {
        r = 0xff;
        g = 0;
        b = 0xff;
    }

    return cv::Scalar(b, g, r);
}

TransformInfo getTransformInfo(const std::string transformationInfoPath);

template <typename PointCloudType>
cv::Point projectToImagePlane(const PointCloudType& point3d, const CameraInfo& cameraInfo)
{
    Eigen::Matrix<double, 3, 1> point(point3d.x, point3d.y, point3d.z);
    Eigen::Matrix<double, 3, 1> point2d = cameraInfo.K() * point;  // homogenous coordinate of 2d point

    return cv::Point(point2d(0) / point2d(2), point2d(1) / point2d(2));
}

template <typename PointCloudType>
cv::Mat drawPointCloudOnImagePlane(const cv::Mat& img, const typename pcl::PointCloud<PointCloudType>::Ptr& inCloud,
                                   const CameraInfo& cameraInfo,
                                   const Eigen::Affine3d& affine = Eigen::Affine3d::Identity())
{
    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());

    cv::Mat visualizedImg = img.clone();
    for (const auto& point : alignedCloud->points) {
        if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
            continue;
        }

        cv::Point imgPoint = projectToImagePlane<PointCloudType>(point, cameraInfo);
        if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
            continue;
        }

        cv::circle(visualizedImg, imgPoint, 1 /* radius */, colorCodingDepthBGR(point.x), -1);
    }

    return visualizedImg;
}
}  // namespace perception

#ifdef DEBUG
#define ENABLE_DEBUG 1
#include <iostream>
#else
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define DEBUG_LOG(...)                                                                                                 \
    {                                                                                                                  \
        char str[100];                                                                                                 \
        snprintf(str, sizeof(str), __VA_ARGS__);                                                                       \
        std::cout << "[" << __FILE__ << "][" << __FUNCTION__ << "][Line " << __LINE__ << "] >>> " << str << std::endl; \
    }
#else
#define DEBUG_LOG(...)
#endif
