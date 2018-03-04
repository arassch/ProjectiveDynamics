#ifndef STATICBODY_H
#define STATICBODY_H

#include <TriMesh.h>
#include <HashCCDHandler.h>

#include "projectiveBody.h"
#include "projectiveConstraint.h"

class StaticBody : public ProjectiveBody
{
    TriMesh* m_mesh;

public:
    StaticBody(TriMesh* mesh, std::string name)
        : ProjectiveBody(name, 0), m_mesh(mesh)
    {
        m_CCDHandler = new HashCCDHandler(m_mesh);
        m_CCDHandler->Init();
        m_numParticles = 0;
    }


    Eigen::VectorXf getParticleMassVector()
    {
        Eigen::VectorXf mass(0);
        return mass;
    }

    Eigen::VectorXf getPositions()
    {
        Eigen::VectorXf pos(0);
        return pos;
    }

    Eigen::VectorXf getVelocities(float dt)
    {
        Eigen::VectorXf vel(0);
        return vel;
    }

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz) {}

    std::vector<ProjectiveConstraint *> getConstraints()
    {
        std::vector<ProjectiveConstraint *> constraints;
        return constraints;
    }

    void addPositionConstraint(float stiffness, int vIndex)
    {

    }

    std::vector<PositionConstraint> getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position)
    {
        std::vector<PositionConstraint> constraints;
        return constraints;
    }

    void draw()
    {
        m_mesh->Draw();
    }
};

#endif // STATICBODY_H
