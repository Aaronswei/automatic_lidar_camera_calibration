/**
 * @file    Gaussian.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include "Types.hpp"

namespace opti
{
template <std::size_t size, typename Scalar = double> class Gaussian
{
 public:
    using DataType = opti::DataType<size, Scalar>;
    using CovType = opti::CovType<size, Scalar>;
    using KernelType = opti::KernelType<size, Scalar>;

    Gaussian() = delete;

    static KernelType estimate(const DataType& mean, const CovType& covariance);
};

template <std::size_t size, typename Scalar>
typename Gaussian<size, Scalar>::KernelType Gaussian<size, Scalar>::estimate(const DataType& mean,
                                                                             const CovType& covariance)
{
    KernelType outputKernel = [&](const DataType& x) {
        Scalar result;
        result = std::exp(-1 / 2.0 * ((x - mean).transpose() * covariance.inverse() * (x - mean))(0)) /
                 std::sqrt(std::pow(2 * M_PI, size) * covariance.determinant());

        return result;
    };

    return outputKernel;
}
}  // namespace opti
