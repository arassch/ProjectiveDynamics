#ifndef SPRINGBODY_H
#define SPRINGBODY_H

#include <TriMesh.h>
#include <TriVertex.h>
#include <HashCCDHandler.h>

#include "projectiveConstraint.h"
#include "springConstraint.h"
#include "positionConstraint.h"
#include "projectiveBody.h"

class SpringBody : public ProjectiveBody
{
    TriMesh *m_mesh;

    std::vector<SpringConstraint*> m_springConstraints;

    std::vector<PositionConstraint*> m_positionConstraints;

public:
    SpringBody(TriMesh* mesh, float particleMass, float springStiffness, bool addExtraSprings=false, float maxDistance=0);

    Eigen::VectorXf getPositions();

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz);

    void setVelocities(const Eigen::VectorXf &vx, const Eigen::VectorXf &vy, const Eigen::VectorXf &vz);

    std::vector<ProjectiveConstraint *> getConstraints();

    void addPositionConstraint(float stiffness, int vIndex);

    void draw();

};

#endif // SPRINGBODY_H
