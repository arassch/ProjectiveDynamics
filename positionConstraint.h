#ifndef POSITIONCONSTRAINT_H
#define POSITIONCONSTRAINT_H

#include <Eigen/Core>
#include <Eigen/Sparse>


class PositionConstraint
{
    int m_vIndex;

    Eigen::Vector3f m_position;



public:
    float m_stiffness;

    PositionConstraint(float stiffness, Eigen::Vector3f p, int vIndex);

    void project(Eigen::Vector3f q, Eigen::Vector3f &p);

    Eigen::SparseMatrix<float> getSMatrix(int numParticles, int dim);

    int getVIndex() { return m_vIndex; }

    Eigen::Vector3f getPosition() { return m_position; }
};

#endif // POSITIONCONSTRAINT_H
