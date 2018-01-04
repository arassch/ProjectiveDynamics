#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>

#include <TriMesh.h>


#include <CDQueries.h>
#include <HashCCDHandler.h>
#include <TriMesh.h>

#include "springConstraint.h"
#include "positionConstraint.h"

class Simulator
{
    TriMesh *m_mesh;
    int m_numParticles;

    vector<TriMesh*> m_rigidBodies;

    HashCCDHandler *m_meshCCDHandler;
    vector<HashCCDHandler*> m_rigidBodiesCCDHandler;
    vector<CollisionInfo*> m_collisions;

    float m_dt;
    int m_iterations;
    float m_springStiffness;
    float m_collisionStiffness;

    std::vector<PositionConstraint*> m_positionConstraints;
    std::vector<SpringConstraint*> m_springConstraints;

    double m_particleMass;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrix;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrixInv;

    Eigen::SparseMatrix<float> m_Lhs;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float> > m_cholesky;


public:

    Eigen::VectorXf m_fext;

    vector<Eigen::Vector3f> m_projected;


    Simulator(TriMesh* mesh, vector<TriMesh*> &rigidBodies, float dt, int iterations, double particleMass,
              vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
              vector<pair<int,int> > &springConstraints, float springStiffness,
              float collisionStiffness);
    virtual ~Simulator();

    void initialize(float dt, vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
                    vector<pair<int,int> > &springConstraints);

    void advanceTime(vector<Eigen::Vector3f> &hardConstraintsVector);


    vector<CollisionInfo *> getCollisions() const { return m_collisions; }
};

#endif // SIMULATOR_H
