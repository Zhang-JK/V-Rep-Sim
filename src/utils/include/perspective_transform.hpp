#pragma once

#include <opencv2/imgproc/imgproc.hpp>

cv::Mat perspective_transform(cv::Mat image, cv::Point2f *points);

cv::Mat extractHuman(cv::Mat image, cv::Point2f& centerP, bool show);