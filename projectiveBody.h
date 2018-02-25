#ifndef PROJECTIVEBODY_H
#define PROJECTIVEBODY_H

#include <Eigen/Core>

class ProjectiveBody
{
protected:

    int m_numParticles;

    float m_totalMass;

public:

    ProjectiveBody(float totalMass) : m_totalMass(totalMass) {}

    int getNumParticles() { return m_numParticles; }

    Eigen::VectorXf getParticleMassVector()
    {
        Eigen::VectorXf mass(m_numParticles);
        mass.setOnes();
        mass *= m_totalMass / m_numParticles;
        return mass;
    }

    virtual Eigen::VectorXf getPositions() = 0;

    virtual void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz) = 0;

    virtual std::vector<ProjectiveConstraint *> getConstraints() = 0;

    virtual void addPositionConstraint(float stiffness, int vIndex) = 0;

    virtual void draw() = 0;
};

#endif // PROJECTIVEBODY_H
