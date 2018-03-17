#ifndef HAIRBODY_H
#define HAIRBODY_H

#include "projectiveConstraint.h"
#include "springConstraint.h"
#include "positionConstraint.h"
#include "projectiveBody.h"

class HairBody : public ProjectiveBody
{

    std::vector<SpringConstraint*> m_springConstraints;

    std::vector<PositionConstraint*> m_positionConstraints;

    std::map<int, std::vector<SpringConstraint*> > m_indexToSpring;

    std::vector<Eigen::Vector3f> m_positions;
    std::vector<Eigen::Vector3f> m_velocities;


public:
    HairBody(std::vector<Eigen::Vector3f> controlPoints, std::string name, float totalMass, float springStiffness,
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




#endif // HAIRBODY_H
