#ifndef COLLISIONCONSTRAINT_H
#define COLLISIONCONSTRAINT_H


#include <Eigen/Core>
#include <Eigen/Sparse>

#include "projectiveConstraint.h"

class CollisionConstraint : public ProjectiveConstraint
{
    float m_restLength;
    int m_vIndex1;
    int m_vIndex2;

public:

    CollisionConstraint(float stiffness, Eigen::Vector3f p1, Eigen::Vector3f p2, int vIndex1, int vIndex2);

    void project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f>& p);

    Eigen::MatrixXf getAMatrix();
    Eigen::MatrixXf getBMatrix();
    Eigen::SparseMatrix<float> getSMatrix(int numParticles, int dim);

    int getVIndex(int index);


};

#endif // COLLISIONCONSTRAINT_H
