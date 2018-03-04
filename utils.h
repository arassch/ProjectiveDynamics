#ifndef UTILS_H
#define UTILS_H

#include <QImage>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)

#include <Eigen/Core>

#include "Vector3.h"

LA::Vector3 toLAVector3(const Eigen::Vector3f &v);

Eigen::Vector3f toEigenVector3(const LA::Vector3 &v);

Eigen::Vector3f toEigenVector3(Eigen::VectorXf &v, int index);

Eigen::Vector3f toEigenVector3(Eigen::VectorXf &vx, Eigen::VectorXf &vy, Eigen::VectorXf &vz, int index);

float clamp(float value, float min, float max);

cv::Mat qImageToCVMat(QImage const &img);

#endif // UTILS_H
