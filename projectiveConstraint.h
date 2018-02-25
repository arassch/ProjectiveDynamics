#ifndef ProjectiveConstraint_H
#define ProjectiveConstraint_H

#include <Eigen/Core>
#include <Eigen/Sparse>

class ProjectiveConstraint
{
public:
    float m_stiffness;
    int m_numParticles;

    ProjectiveConstraint(float stiffness, int numParticles)
        : m_stiffness(stiffness), m_numParticles(numParticles)
    {}

    virtual Eigen::MatrixXf getAMatrix() = 0;
    virtual Eigen::MatrixXf getBMatrix() = 0;
    virtual Eigen::SparseMatrix<float> getSMatrix(int numParticles, int dim) = 0;

    virtual int getVIndex(int index) = 0;

    virtual void project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f>& p) = 0;
};

#endif // ProjectiveConstraint_H