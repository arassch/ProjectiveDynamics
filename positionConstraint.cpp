#include "positionConstraint.h"

using namespace std;

PositionConstraint::PositionConstraint(float stiffness, Eigen::Vector3f p, int vIndex)
    : Constraint(stiffness, 1)
{
    m_position = p;
    m_vIndex = vIndex;
}

void PositionConstraint::project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f> &p)
{
    p = q;
}

void PositionConstraint::project(Eigen::Vector3f q, Eigen::Vector3f &p)
{
    p = q;
}

Eigen::MatrixXf PositionConstraint::getAMatrix()
{
    Eigen::MatrixXf A(1,1);
    A.setOnes();
    return A;
}

Eigen::MatrixXf PositionConstraint::getBMatrix()
{
    return getAMatrix();
}

Eigen::SparseMatrix<float> PositionConstraint::getSMatrix(int numParticles, int dim)
{
    Eigen::SparseMatrix<float> Si(1, 3*numParticles);
    vector<Eigen::Triplet<float> > SiData;
    SiData.push_back(Eigen::Triplet<float> (0, 3*m_vIndex+dim, 1));

    Si.setFromTriplets(SiData.begin(), SiData.end());
    return Si;
}
