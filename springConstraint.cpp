#include "springConstraint.h"

using namespace std;

SpringConstraint::SpringConstraint(float stiffness, Eigen::Vector3f p1, Eigen::Vector3f p2, int vIndex1, int vIndex2) :
    m_vIndex1(vIndex1), m_vIndex2(vIndex2)
{
    m_restLength = (p1-p2).norm();

    m_stiffness = stiffness;
}

void SpringConstraint::project(Eigen::Vector3f q1, Eigen::Vector3f q2, Eigen::Vector3f &p1, Eigen::Vector3f &p2)
{
    float l = (q1-q2).norm();
    float offset = (l - m_restLength)/2.0;

    Eigen::Vector3f v = (q2 - q1);
    v.normalize();
    p1 = q1 + offset * v;
    p2 = q2 - offset * v;
}

Eigen::Matrix2f SpringConstraint::getAMatrix()
{
    Eigen::Matrix2f A;
    A.setOnes();
    A *= 0.5;
    A(0, 1) *= -1.0;
    A(1, 0) *= -1.0;
    return A;
}

Eigen::Matrix2f SpringConstraint::getBMatrix()
{
    Eigen::Matrix2f B;
    B.setOnes();
    B *= 0.5;
    B(0, 1) *= -1.0;
    B(1, 0) *= -1.0;
    return B;
}

Eigen::SparseMatrix<float> SpringConstraint::getSMatrix(int numParticles, int dim)
{
    Eigen::SparseMatrix<float> Si(2, 3*numParticles);
    vector<Eigen::Triplet<float> > SiData;
    SiData.push_back(Eigen::Triplet<float> (0, 3*m_vIndex1+dim, 1));
    SiData.push_back(Eigen::Triplet<float> (1, 3*m_vIndex2+dim, 1));

    Si.setFromTriplets(SiData.begin(), SiData.end());
    return Si;
}
