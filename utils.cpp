#include "utils.h"

Eigen::Vector3f toEigenVector3(const LA::Vector3 &v)
{
    Eigen::Vector3f r(v[0], v[1], v[2]);
    return r;
}

float clamp(float value, float min, float max)
{
    float res = std::min(value, max);
    res = std::max(res, min);
    return res;
}
