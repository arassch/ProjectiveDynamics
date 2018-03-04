#ifndef TETRACONSTRAINT_H
#define TETRACONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SVD>

#include "projectiveConstraint.h"

class TetraConstraint : public ProjectiveConstraint
{
    int m_vIndex1;
    int m_vIndex2;
    int m_vIndex3;
    int m_vIndex4;

    Eigen::Matrix3f m_Dm;
    Eigen::Matrix3f m_invDm;

public:
    float m_stiffness;
    float m_minSigma;
    float m_maxSigma;

    TetraConstraint(ProjectiveBody *body,
                    float stiffness,
                    Eigen::Vector3f p1,
                    Eigen::Vector3f p2,
                    Eigen::Vector3f p3,
                    Eigen::Vector3f p4,
                    int vIndex1,
                    int vIndex2,
                    int vIndex3,
                    int vIndex4);

    void project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f>& p);

    Eigen::MatrixXf getAMatrix();
    Eigen::MatrixXf getBMatrix();
    Eigen::SparseMatrix<float> getSMatrix(int numParticles, int bodyIndex, int dim);

    int getVIndex(int index);


};

#endif // TETRACONSTRAINT_H
