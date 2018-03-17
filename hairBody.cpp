#include "hairBody.h"

#include "springBody.h"
#include "utils.h"

HairBody::HairBody(std::vector<Eigen::Vector3f> controlPoints, string name, float totalMass, float springStiffness,
                   float damping, float restitution)
    : ProjectiveBody(name, SPRING, totalMass, damping, restitution)
{
    m_numParticles = controlPoints.size();
    m_positions = controlPoints;
    m_velocities.resize(m_numParticles);
    for(int i=0; i<m_numParticles; ++i)
        m_velocities[i].setZero();

    m_springConstraints.clear();
    for(int i=1;i<controlPoints.size();++i)
    {
        int v1 = i-1;
        int v2 = i;

        Eigen::Vector3f p1 = controlPoints[v1];
        Eigen::Vector3f p2 = controlPoints[v2];

        SpringConstraint *c = new SpringConstraint(this, springStiffness, p1, p2, v1, v2);
        m_springConstraints.push_back(c);

        {
            auto it = m_indexToSpring.find(v1);
            if (it != m_indexToSpring.end())
                it->second.push_back(c);
            else
            {
                std::vector<SpringConstraint*> vec;
                vec.push_back(c);
                m_indexToSpring[v1] = vec;
            }

            it = m_indexToSpring.find(v2);
            if (it != m_indexToSpring.end())
                it->second.push_back(c);
            else
            {
                std::vector<SpringConstraint*> vec;
                vec.push_back(c);
                m_indexToSpring[v2] = vec;
            }
        }
    }

    m_CCDHandler = 0;
}

Eigen::VectorXf HairBody::getPositions()
{
    Eigen::VectorXf q;
    q.resize(m_positions.size()*3);
    for(int i=0; i<m_positions.size(); ++i)
    {
        q[i*3+0] = m_positions[i][0];
        q[i*3+1] = m_positions[i][1];
        q[i*3+2] = m_positions[i][2];
    }
    return q;
}

Eigen::VectorXf HairBody::getVelocities(float dt)
{
    Eigen::VectorXf v;
    v.resize(m_velocities.size()*3);
    for(int i=0; i<m_velocities.size(); ++i)
    {
        v[i*3+0] = m_velocities[i][0];
        v[i*3+1] = m_velocities[i][1];
        v[i*3+2] = m_velocities[i][2];
    }
    return v;
}

void HairBody::setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz)
{
    for(int i=0; i<m_positions.size(); ++i)
        m_positions[i] = Eigen::Vector3f(qx[i], qy[i], qz[i]);
}

std::vector<ProjectiveConstraint *> HairBody::getConstraints()
{
    std::vector<ProjectiveConstraint *> result;
    result.insert(result.end(), m_springConstraints.begin(), m_springConstraints.end());
    result.insert(result.end(), m_positionConstraints.begin(), m_positionConstraints.end());
    return result;
}

void HairBody::addPositionConstraint(float stiffness, int vIndex)
{
    Eigen::Vector3f p = m_positions[vIndex];
    PositionConstraint *c = new PositionConstraint(this, stiffness, p, vIndex);
    m_positionConstraints.push_back(c);

    auto it = m_indexToSpring.find(vIndex);
    if(it != m_indexToSpring.end())
    {
        std::vector<SpringConstraint*> v = it->second;
        for(int i=0;i<v.size();++i)
            v[i]->setFixed(vIndex);
    }
}

std::vector<PositionConstraint> HairBody::getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position)
{
    std::vector<PositionConstraint> constraints;
    constraints.push_back(PositionConstraint(this, stiffness, position, vIndex));
    return constraints;
}

std::vector<int> HairBody::getIndices(int vIndex)
{
    std::vector<int> indices;
    indices.push_back(vIndex);
    return indices;
}

void HairBody::draw()
{

}
