/**
 * @file    KernelDensityEstimation.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include "Types.hpp"

namespace opti
{
template <std::size_t size, typename Scalar = double> class KernelDensityEstimation
{
 public:
    using DataType = opti::DataType<size, Scalar>;
    using DataTypes = opti::DataTypes<size, Scalar>;
    using KernelType = opti::KernelType<size, Scalar>;

    KernelDensityEstimation() = delete;

    static KernelType estimate(const KernelType& kernel, const DataTypes& samples, const Scalar bandwidth);
};

template <std::size_t size, typename Scalar>
typename KernelDensityEstimation<size, Scalar>::KernelType
KernelDensityEstimation<size, Scalar>::estimate(const KernelType& kernel, const DataTypes& samples,
                                                const Scalar bandwidth)
{
    const int N = samples.size();

    if (N == 0) {
        throw std::runtime_error("number of samples must be more than 0");
    }

    KernelType outputKernel = [&](const DataType& x) {
        Scalar result = std::accumulate(samples.begin(), samples.end(), 0,
                                        [&kernel, &bandwidth, &x](Scalar result, const DataType& elem) {
                                            return std::move(result) + kernel((x - elem) / bandwidth);
                                        }) /
                        (N * bandwidth);
        return result;
    };

    return outputKernel;
}
}  // namespace opti
