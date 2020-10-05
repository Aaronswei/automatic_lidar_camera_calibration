/**
 * @file    ProbabilityHandler.cpp
 *
 * @author  btran
 *
 */

#include <opencv2/opencv.hpp>

#include <sensors_calib/ProbabilityHandler.hpp>
#include <sensors_calib/utils/utils.hpp>

namespace perception
{
ProbabilityHandler::ProbabilityHandler(int numBins)
    : m_numBins(numBins)
{
    if (m_numBins <= 0 || m_numBins > perception::MAX_BINS) {
        throw std::runtime_error("invalid number of bins");
    }

    m_grayProb = Probability::zeros(1, m_numBins, CV_64FC1);
    m_intensityProb = Probability::zeros(1, m_numBins, CV_64FC1);
    m_jointProb = JointProbability::zeros(m_numBins, m_numBins, CV_64FC1);
}

ProbabilityHandler::~ProbabilityHandler()
{
}

bool ProbabilityHandler::estimateMLE(const HistogramHandler::Ptr& histogram)
{
    if (histogram->totalPoints() == 0) {
        DEBUG_LOG("empty sample datas");
        return false;
    }
    m_totalPoints = histogram->totalPoints();

    auto stds = histogram->calculateStds();
    double sigmaGray = stds[0];
    double sigmaIntensity = stds[1];
    double sigmaCorr = stds[2];

    m_corrCoeff = sigmaCorr / (sigmaGray * sigmaIntensity);

    for (int i = 0; i < m_numBins; ++i) {
        for (int j = 0; j < m_numBins; ++j) {
            m_jointProb.at<double>(i, j) = histogram->jointHist().at<double>(i, j) / m_totalPoints;
        }

        m_grayProb.at<double>(i) = histogram->grayHist().at<double>(i) / m_totalPoints;
        m_intensityProb.at<double>(i) = histogram->intensityHist().at<double>(i) / m_totalPoints;
    }

    // bandwidths for kernel density estimation based on Silverman's rule of thumb
    m_sigmaGrayBandwidth = 1.06 * std::sqrt(sigmaGray) / std::pow(m_totalPoints, 0.2);
    m_sigmaIntensityBandwidth = 1.06 * std::sqrt(sigmaIntensity) / std::pow(m_totalPoints, 0.2);

    this->smoothKDE();

    return true;
}

void ProbabilityHandler::smoothKDE()
{
    cv::GaussianBlur(m_grayProb, m_grayProb, cv::Size(0, 0), m_sigmaGrayBandwidth);
    cv::GaussianBlur(m_intensityProb, m_intensityProb, cv::Size(0, 0), m_sigmaIntensityBandwidth);
    cv::GaussianBlur(m_jointProb, m_jointProb, cv::Size(0, 0), m_sigmaGrayBandwidth, m_sigmaIntensityBandwidth);
}
}  // namespace perception
