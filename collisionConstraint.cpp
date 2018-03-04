#include "collisionConstraint.h"

CollisionConstraint::CollisionConstraint(float stiffness, Eigen::Vector3f p1, Eigen::Vector3f p2, int vIndex1, int vIndex2) :
    ProjectiveConstraint(stiffness, 2), m_vIndex1(vIndex1), m_vIndex2(vIndex2)
{

}

void CollisionConstraint::project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f> &p)
{

}

Eigen::MatrixXf CollisionConstraint::getAMatrix()
{

}

Eigen::MatrixXf CollisionConstraint::getBMatrix()
{

}

Eigen::SparseMatrix<float> CollisionConstraint::getSMatrix(int numParticles, int dim)
{

}

int CollisionConstraint::getVIndex(int index)
{

}
