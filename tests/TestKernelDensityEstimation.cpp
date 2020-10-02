/**
 * @file    TestKernelDensityEstimation.cpp
 *
 * @author  btran
 *
 */

#include <gtest/gtest.h>

#include <sensors_calib/sensors_calib.hpp>

#include "RandDouble.hpp"

TEST(TestKernelDensityEstimation, TestInitialization)
{
    const std::size_t size = 1;
    const std::size_t numSamples = 100;
    auto doubleGenerator = ::RandDouble(0.0, 100.0);

    using KDE = opti::KernelDensityEstimation<size, double>;
    KDE::KernelType kernel = [&](const KDE::DataType& x) { return x.dot(x); };
    KDE::DataTypes samples(numSamples);

    for (std::size_t i = 0; i < numSamples; ++i) {
        samples[i] << doubleGenerator();
    }
    double bandwidth = 20;

    KDE::KernelType estimatedKernel = KDE::estimate(kernel, samples, bandwidth);

    KDE::DataType testSample(2000.4);
    EXPECT_NO_THROW(estimatedKernel(testSample));
}
