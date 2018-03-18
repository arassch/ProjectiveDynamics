#include "projectiveConstraint.h"

ProjectiveConstraint::ProjectiveConstraint(ProjectiveBody *body, float stiffness, int numParticles)
    : m_body(body), m_stiffness(stiffness), m_numParticles(numParticles)
{
    m_rhsComputedForBodyIndex = -1;
    m_rhsComputedForNumParticles = -1;
}

Eigen::SparseMatrix<float>& ProjectiveConstraint::getRhsMatrix(int numParticles, int bodyIndex, int dim)
{
    if(numParticles == m_rhsComputedForNumParticles && bodyIndex == m_rhsComputedForBodyIndex)
        return m_rhsMatrix[dim];

    m_rhsComputedForNumParticles = numParticles;
    m_rhsComputedForBodyIndex = bodyIndex;

    Eigen::MatrixXf Ai = getAMatrix();
    Eigen::MatrixXf Bi = getBMatrix();
    Eigen::SparseMatrix<float> SiX = getSMatrix(numParticles, bodyIndex, 0);
    Eigen::SparseMatrix<float> SiY = getSMatrix(numParticles, bodyIndex, 1);
    Eigen::SparseMatrix<float> SiZ = getSMatrix(numParticles, bodyIndex, 2);

    m_rhsMatrix[0].resize(SiX.cols(), SiX.rows());
    m_rhsMatrix[1].resize(SiY.cols(), SiY.rows());
    m_rhsMatrix[2].resize(SiZ.cols(), SiZ.rows());
    m_rhsMatrix[0].setZero();
    m_rhsMatrix[1].setZero();
    m_rhsMatrix[2].setZero();

    m_rhsMatrix[0] = m_rhsMatrix[0] + m_stiffness * SiX.transpose() * Ai.transpose() * Bi;
    m_rhsMatrix[1] = m_rhsMatrix[1] + m_stiffness * SiY.transpose() * Ai.transpose() * Bi;
    m_rhsMatrix[2] = m_rhsMatrix[2] + m_stiffness * SiZ.transpose() * Ai.transpose() * Bi;

    return m_rhsMatrix[dim];
}
