#ifndef TETRABODY_H
#define TETRABODY_H


#include <TriMesh.h>
#include <TriVertex.h>
#include <HashCCDHandler.h>
#include <TetraMesh.h>
#include <Tetra.h>
#include <Vertex.h>

#include "projectiveConstraint.h"
#include "tetraConstraint.h"
#include "positionConstraint.h"
#include "projectiveBody.h"

class TetraBody : public ProjectiveBody
{
    TriMesh *m_mesh;

    TetraMesh *m_tetraMesh;

    std::vector<TetraConstraint*> m_tetraConstraints;

    std::vector<PositionConstraint*> m_positionConstraints;

public:
    TetraBody(TriMesh* mesh, std::string name, float totalMass, float tetraStiffness, int numTetras, bool deleteExtraTetras, float damping=0.95, float restitution=1);

    Eigen::VectorXf getPositions();

    Eigen::VectorXf getVelocities(float dt);

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz);

    std::vector<ProjectiveConstraint *> getConstraints();

    void addPositionConstraint(float stiffness, int vIndex);

    std::vector<PositionConstraint> getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position);

    void draw();

    TetraMesh *getTetraMesh() { return m_tetraMesh; }

    void addVelocity(Eigen::Vector3f v);

};

#endif // TETRABODY_H
