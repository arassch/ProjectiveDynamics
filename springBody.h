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

    std::map<int, std::vector<SpringConstraint*> > m_indexToSpring;

public:
    SpringBody(TriMesh* mesh, std::string name, float totalMass, float springStiffness, bool addExtraSprings=false, float maxDistance=0,
               float damping=0.95, float restitution=1);

    Eigen::VectorXf getPositions();

    Eigen::VectorXf getVelocities(float dt);

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz);

    std::vector<ProjectiveConstraint *> getConstraints();

    void addPositionConstraint(float stiffness, int vIndex);

    std::vector<PositionConstraint> getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position);

    std::vector<int> getIndices(int vIndex);

    void draw();

};

#endif // SPRINGBODY_H
