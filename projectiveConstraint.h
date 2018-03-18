#ifndef ProjectiveConstraint_H
#define ProjectiveConstraint_H

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "projectiveBody.h"

class ProjectiveConstraint
{
public:
    float m_stiffness;
    int m_numParticles;

    ProjectiveBody *m_body;

    int m_rhsComputedForNumParticles;
    int m_rhsComputedForBodyIndex;
    Eigen::SparseMatrix<float> m_rhsMatrix[3];

    ProjectiveConstraint(ProjectiveBody* body, float stiffness, int numParticles);

    virtual Eigen::MatrixXf getAMatrix() = 0;
    virtual Eigen::MatrixXf getBMatrix() = 0;
    virtual Eigen::SparseMatrix<float> getSMatrix(int numParticles, int bodyIndex, int dim) = 0;

    Eigen::SparseMatrix<float> &getRhsMatrix(int numParticles, int bodyIndex, int dim);

    virtual int getVIndex(int index) = 0;

    virtual void project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f>& p) = 0;
};

#endif // ProjectiveConstraint_H
