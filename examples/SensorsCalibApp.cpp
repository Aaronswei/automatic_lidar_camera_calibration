/**
 * @file    SensorsCalibApp.cpp
 *
 * @author  btran
 *
 */

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <sensors_calib/sensors_calib.hpp>

namespace
{
using PointCloudType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 5) {
        std::cerr << "Usage: [app] [path/to/camera/info] [path/to/image] [path/to/pcd] [path/to/initial/guess]"
                  << std::endl;
        return EXIT_FAILURE;
    }

    const std::string CAMERA_INFO_PATH = argv[1];
    const std::string IMAGE_PATH = argv[2];
    const std::string PCD_PATH = argv[3];
    const std::string INITIAL_GUESS_PATH = argv[4];
    const int NUM_BINS = 256;

    perception::CameraInfo::Ptr cameraInfo(new perception::CameraInfo(CAMERA_INFO_PATH));
    cv::Mat img = cv::imread(IMAGE_PATH);
    if (img.empty()) {
        std::cerr << "failed to load image: " << IMAGE_PATH << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat grayImg;
    cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

    PointCloudPtr inCloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointCloudType>(PCD_PATH, *inCloud) == -1) {
        std::cerr << "failed to load pcd: " << PCD_PATH << std::endl;
        return EXIT_FAILURE;
    }

    perception::TransformInfo initialGuess = perception::getTransformInfo(INITIAL_GUESS_PATH);

    perception::HistogramHandler::Ptr histogramHandler(new perception::HistogramHandler(NUM_BINS));
    histogramHandler->update<PointCloudType>({grayImg}, {inCloud}, *cameraInfo, perception::toAffine(initialGuess));

    perception::ProbabilityHandler::Ptr probabilityHandler(new perception::ProbabilityHandler(NUM_BINS));
    probabilityHandler->estimateMLE(histogramHandler);

    cv::Mat visualized = perception::drawPointCloudOnImagePlane<PointCloudType>(img, inCloud, *cameraInfo,
                                                                                perception::toAffine(initialGuess));
    cv::imwrite("visualized.png", visualized);

    return EXIT_SUCCESS;
}
