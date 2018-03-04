#include "positionConstraint.h"

using namespace std;


PositionConstraint::PositionConstraint(ProjectiveBody *body, float stiffness, Eigen::Vector3f p, int vIndex)
    : ProjectiveConstraint(body, stiffness, 1)
{
    m_position = p;
    m_vIndex = vIndex;
}

PositionConstraint::PositionConstraint(const PositionConstraint &c)
    : ProjectiveConstraint(c.m_body, c.m_stiffness, 1)
{
    m_position = c.m_position;
    m_vIndex = c.m_vIndex;
}

void PositionConstraint::project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f> &p)
{
    p.resize(1);
    p[0] = m_position;
}

void PositionConstraint::project(Eigen::Vector3f q, Eigen::Vector3f &p)
{
    p = m_position;
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

Eigen::SparseMatrix<float> PositionConstraint::getSMatrix(int numParticles, int bodyIndex, int dim)
{
    Eigen::SparseMatrix<float> Si(1, numParticles);
    vector<Eigen::Triplet<float> > SiData;
    SiData.push_back(Eigen::Triplet<float> (0, bodyIndex+m_vIndex, 1));

    Si.setFromTriplets(SiData.begin(), SiData.end());
    return Si;
}
