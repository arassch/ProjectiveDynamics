
#include "tetraConstraint.h"
#include "utils.h"

TetraConstraint::TetraConstraint(float stiffness, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4, int vIndex1, int vIndex2, int vIndex3, int vIndex4)
    : Constraint(stiffness, 4), m_vIndex1(vIndex1), m_vIndex2(vIndex2), m_vIndex3(vIndex3), m_vIndex4(vIndex4)
{
    m_Dm.col(0) = p4 - p1;
    m_Dm.col(1) = p4 - p2;
    m_Dm.col(2) = p4 - p3;

    m_invDm = m_Dm.inverse();

    m_minSigma = 0.95;
    m_maxSigma = 1.05;
}

void TetraConstraint::project(std::vector<Eigen::Vector3f> q, std::vector<Eigen::Vector3f> &p)
{
    assert(q.size() == 4);
    p.resize(4);

    Eigen::Vector3f q1 = q[0];
    Eigen::Vector3f q2 = q[1];
    Eigen::Vector3f q3 = q[2];
    Eigen::Vector3f q4 = q[3];

    Eigen::Matrix3f F, Ds;

    Ds.col(0) = q4 - q1;
    Ds.col(1) = q4 - q2;
    Ds.col(2) = q4 - q3;

    F = Ds*m_invDm;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd( F, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector3f s = svd.singularValues();
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    float detU = U.determinant();
    float detV = V.determinant();

    // make sure that both U and V are rotation matrices without reflection
    if (detU < 0)
    {
        U.block<3,1>(0,2) *= -1;
        s[2] *= -1;
    }
    if (detV < 0)
    {
        V.block<3,1>(0,2) *= -1;
        s[2] *= -1;
    }

    s[0] = clamp(s[0], m_minSigma, m_maxSigma);
    s[1] = clamp(s[1], m_minSigma, m_maxSigma);
    s[2] = clamp(s[2], m_minSigma, m_maxSigma);
    Eigen::Matrix3f newF = U * s.asDiagonal() * V.transpose();

    Eigen::Matrix3f newDef = newF * m_Dm;

    Eigen::Vector3f centroid = (q1+q2+q3+q4)/4.0;

    p[3] = (centroid + (newDef.col(0) + newDef.col(1) + newDef.col(2)) / 4.0);
    p[0] = p[3] - newDef.col(0);
    p[1] = p[3] - newDef.col(1);
    p[2] = p[3] - newDef.col(2);
}

Eigen::MatrixXf TetraConstraint::getAMatrix()
{
    Eigen::MatrixXf A(4,4);
    A.setConstant(-1.0/4.0);
    A(0,0) = A(1,1) = A(2,2) = A(3,3) = 3.0/4.0;
    return A;
}

Eigen::MatrixXf TetraConstraint::getBMatrix()
{
    return getAMatrix();
}

Eigen::SparseMatrix<float> TetraConstraint::getSMatrix(int numParticles, int dim)
{
    Eigen::SparseMatrix<float> Si(4, 3*numParticles);
    vector<Eigen::Triplet<float> > SiData;
    SiData.push_back(Eigen::Triplet<float> (0, 3*m_vIndex1+dim, 1));
    SiData.push_back(Eigen::Triplet<float> (1, 3*m_vIndex2+dim, 1));
    SiData.push_back(Eigen::Triplet<float> (2, 3*m_vIndex3+dim, 1));
    SiData.push_back(Eigen::Triplet<float> (3, 3*m_vIndex4+dim, 1));

    Si.setFromTriplets(SiData.begin(), SiData.end());
    return Si;
}

int TetraConstraint::getVIndex(int index)
{
    switch(index)
    {
    case 0: return m_vIndex1; break;
    case 1: return m_vIndex2; break;
    case 2: return m_vIndex3; break;
    case 3: return m_vIndex4; break;
    default: return -1; break;
    }
    return -1;
}
