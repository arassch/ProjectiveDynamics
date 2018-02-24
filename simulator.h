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

#include "constraint.h"
#include "springConstraint.h"
#include "positionConstraint.h"
#include "tetraConstraint.h"

class Simulator
{
    TriMesh *m_mesh;
    int m_numParticles;

    vector<TriMesh*> m_staticBodies;

    HashCCDHandler *m_meshCCDHandler;
    vector<HashCCDHandler*> m_staticBodiesCCDHandler;
    vector<CollisionInfo*> m_collisions;

    float m_dt;
    int m_iterations;
    float m_springStiffness;
    float m_collisionStiffness;
    float m_positionStiffness;
    float m_tetraStiffness;



    double m_particleMass;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrix;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrixInv;

    Eigen::SparseMatrix<float> m_Lhs;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float> > m_cholesky;

    int m_timeCollisionDetection;
    int m_timeLocalSolve;
    int m_timeGlobalSolve;


public:

    Eigen::VectorXf m_fext;

    std::vector<Eigen::Vector3f> m_projected;
    std::vector<PositionConstraint*> m_positionConstraints;
    std::vector<SpringConstraint*> m_springConstraints;
    std::vector<TetraConstraint*> m_tetraConstraints;
    std::vector<Constraint*> m_constraints;


    Simulator(TriMesh* mesh, vector<TriMesh*> &staticBodies, float dt, int iterations, double particleMass,
              vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
              vector<pair<int,int> > &springConstraints, float springStiffness,
              vector<vector<int> > &tetraConstraints, float tetraStiffness,
              float collisionStiffness, float positionStiffness);
    virtual ~Simulator();

    void initialize(float dt, vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
                    vector<pair<int,int> > &springConstraints,
                    vector<vector<int> > &tetraConstraints);

    void advanceTime(vector<Eigen::Vector3f> &hardConstraintsVector);


    vector<CollisionInfo *> getCollisions() const { return m_collisions; }
};

#endif // SIMULATOR_H
