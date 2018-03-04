#include "utils.h"

LA::Vector3 toLAVector3(const Eigen::Vector3f &v)
{
    LA::Vector3 r(v[0], v[1], v[2]);
    return r;
}


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

cv::Mat qImageToCVMat(QImage const &src)
{
    cv::Mat mat = cv::Mat(src.height(), src.width(), CV_8UC4, (uchar*)src.bits(), src.bytesPerLine());
    cv::Mat result = cv::Mat(mat.rows, mat.cols, CV_8UC3 );
    int from_to[] = { 0,0,  1,1,  2,2 };
    cv::mixChannels( &mat, 1, &result, 1, from_to, 3 );
    return result;
}

