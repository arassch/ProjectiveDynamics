#include "utils.h"

Eigen::Vector3f toEigenVector3(const LA::Vector3 &v)
{
    Eigen::Vector3f r(v[0], v[1], v[2]);
    return r;
}

Eigen::Vector3f toEigenVector3(Eigen::VectorXf &v, int index)
{
    Eigen::Vector3f r(v[index*3+0], v[index*3+1], v[index*3+2]);
    return r;
}

Eigen::Vector3f toEigenVector3(Eigen::VectorXf &vx, Eigen::VectorXf &vy, Eigen::VectorXf &vz, int index)
{
    Eigen::Vector3f r(vx[index], vy[index], vz[index]);
    return r;
}


float clamp(float value, float min, float max)
{
    float res = std::min(value, max);
    res = std::max(res, min);
    return res;
}

