#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/Sparse>


class SpringConstraint
{
    float m_restLength;
    int m_vIndex1;
    int m_vIndex2;



public:
    float m_stiffness;

    SpringConstraint(float stiffness, Eigen::Vector3f p1, Eigen::Vector3f p2, int vIndex1, int vIndex2);

    void project(Eigen::Vector3f q1, Eigen::Vector3f q2, Eigen::Vector3f &p1, Eigen::Vector3f &p2);

    Eigen::Matrix2f getAMatrix();
    Eigen::Matrix2f getBMatrix();
    Eigen::SparseMatrix<float> getSMatrix(int numParticles, int dim);

    int getVIndex1() { return m_vIndex1; }
    int getVIndex2() { return m_vIndex2; }


};

#endif // CONSTRAINT_H
