/**
 * @file    Types.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <functional>
#include <numeric>
#include <vector>

#include <Eigen/Dense>

namespace opti
{
template <std::size_t size, typename Scalar = double> using DataType = Eigen::Matrix<Scalar, size, 1>;

template <std::size_t size, typename Scalar = double>
using DataTypes = std::vector<DataType<size, Scalar>, Eigen::aligned_allocator<DataType<size, Scalar>>>;

template <std::size_t size, typename Scalar = double> using CovType = Eigen::Matrix<Scalar, size, size>;

template <std::size_t size, typename Scalar = double>
using KernelType = std::function<Scalar(const DataType<size, Scalar>&)>;
}  // namespace opti
