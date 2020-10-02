/**
 * @file    TestGaussian.cpp
 *
 * @author  btran
 *
 */

#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

#include <sensors_calib/sensors_calib.hpp>

#include "RandDouble.hpp"

TEST(TestGaussian, TestInitialization)
{
    const std::size_t size = 1;
    const std::size_t numSamples = 200;

    using Gaussian = opti::Gaussian<size>;
    Gaussian::DataType mean(100);
    Gaussian::CovType cov(200);

    Gaussian::KernelType kernel = Gaussian::estimate(mean, cov);
    Gaussian::DataType testSample(2);
    EXPECT_NO_THROW(kernel(testSample));

    // draw gaussian
    {
        std::vector<double> ys;
        ys.reserve(numSamples);
        for (std::size_t i = 0; i < numSamples; ++i) {
            ys.emplace_back(kernel(Gaussian::DataType(i)));
        }
        cv::Mat data(ys);
        cv::Ptr<cv::plot::Plot2d> plot;
#if CV_MAJOR_VERSION < 4
        plot = cv::plot::createPlot2d(data);
#else
        plot = cv::plot::Plot2d::create(data);
#endif
        plot->setInvertOrientation(true);

        cv::Mat image;
        plot->render(image);
        cv::imwrite("gaussian.png", image);
    }
}
