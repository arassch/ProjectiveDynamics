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
    TetraBody(TriMesh* mesh, float particleMass, float tetraStiffness);

    Eigen::VectorXf getPositions();

    void setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz);

    std::vector<ProjectiveConstraint *> getConstraints();

    void addPositionConstraint(float stiffness, int vIndex);

    void draw();

    TetraMesh *getTetraMesh() { return m_tetraMesh; }

};

#endif // TETRABODY_H
