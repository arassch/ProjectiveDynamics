#ifndef POSITIONCONSTRAINT_H
#define POSITIONCONSTRAINT_H


#include "projectiveConstraint.h"


class PositionConstraint : public ProjectiveConstraint
{
    int m_vIndex;

    Eigen::Vector3f m_position;

public:


    PositionConstraint(float stiffness, Eigen::Vector3f p, int vIndex);

    void project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f>& p);
    void project(Eigen::Vector3f q, Eigen::Vector3f& p);

    Eigen::MatrixXf getAMatrix();
    Eigen::MatrixXf getBMatrix();
    Eigen::SparseMatrix<float> getSMatrix(int numParticles, int dim);

    int getVIndex(int index) { return m_vIndex; }

    Eigen::Vector3f getPosition() { return m_position; }
};

#endif // POSITIONCONSTRAINT_H
