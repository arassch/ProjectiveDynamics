#ifndef STATICBODY_H
#define STATICBODY_H

#include <TriMesh.h>
#include <HashCCDHandler.h>

#include "projectiveBody.h"
#include "projectiveConstraint.h"

class StaticBody : public ProjectiveBody
{
    TriMesh* m_mesh;

    HashCCDHandler* m_meshCCDHandler;

public:
    StaticBody(TriMesh* mesh)
        : ProjectiveBody(0), m_mesh(mesh)
    {
        m_meshCCDHandler = new HashCCDHandler(m_mesh);
        m_meshCCDHandler->Init();
    }

    int getNumParticles() { return 0; }

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

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz) {}

    std::vector<ProjectiveConstraint *> getConstraints()
    {
        std::vector<ProjectiveConstraint *> constraints;
        return constraints;
    }

    void addPositionConstraint(float stiffness, int vIndex)
    {

    }

    void draw()
    {
        m_mesh->Draw();
    }
};

#endif // STATICBODY_H
