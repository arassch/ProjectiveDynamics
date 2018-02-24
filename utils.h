#ifndef UTILS_H
#define UTILS_H


#include <Eigen/Core>
#include "Vector3.h"

Eigen::Vector3f toEigenVector3(const LA::Vector3 &v);

float clamp(float value, float min, float max);

#endif // UTILS_H
