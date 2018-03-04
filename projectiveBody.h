#ifndef PROJECTIVEBODY_H
#define PROJECTIVEBODY_H

#include <Eigen/Core>

#include <HashCCDHandler.h>

class ProjectiveConstraint;
class PositionConstraint;

class ProjectiveBody
{
protected:

    std::string m_name;

    int m_numParticles;

    float m_totalMass;

    float m_damping;

    float m_restitution;

    HashCCDHandler* m_CCDHandler;

public:

    enum Type {STATIC, SPRING, TETRA};
    Type m_type;

    ProjectiveBody(std::string name, Type type, float totalMass, float damping=0.95, float restitution=1)
        : m_name(name), m_type(type), m_totalMass(totalMass), m_damping(damping), m_restitution(restitution), m_CCDHandler(0) {}

    std::string getName() { return m_name; }

    Type getType() { return m_type; }

    int getNumParticles() { return m_numParticles; }

    float getDamping() { return m_damping; }

    Eigen::VectorXf getParticleMassVector()
    {
        Eigen::VectorXf mass(m_numParticles);
        mass.setOnes();
        mass *= m_totalMass / m_numParticles;
        return mass;
    }

    HashCCDHandler* getCCDHandler() { return m_CCDHandler; }

    virtual Eigen::VectorXf getPositions() = 0;

    virtual Eigen::VectorXf getVelocities(float dt) = 0;

    virtual void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz) = 0;

    virtual std::vector<ProjectiveConstraint *> getConstraints() = 0;

    virtual void addPositionConstraint(float stiffness, int vIndex) = 0;

    virtual std::vector<PositionConstraint> getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position) = 0;

    virtual void draw() = 0;
};

#endif // PROJECTIVEBODY_H
