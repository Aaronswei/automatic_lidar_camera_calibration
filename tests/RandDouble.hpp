/**
 * @file    RandDouble.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <functional>
#include <random>

class RandDouble
{
 public:
    RandDouble(const double low, const double high)
        : m_genFunc(std::bind(std::uniform_real_distribution<>(low, high), std::default_random_engine()))
    {
    }

    double operator()() const
    {
        return m_genFunc();
    }

 private:
    std::function<double()> m_genFunc;
};
