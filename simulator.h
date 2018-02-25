#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>


#include <CDQueries.h>
#include <HashCCDHandler.h>
#include <TriMesh.h>

#include "springBody.h"
#include "projectiveConstraint.h"
#include "springConstraint.h"
#include "positionConstraint.h"
#include "tetraConstraint.h"

class Simulator
{

    int m_numParticles;

    vector<ProjectiveBody*> m_bodies;

    vector<CollisionInfo*> m_collisions;


    float m_dt;
    int m_iterations;



    Eigen::VectorXf m_particleMass;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrix;
    Eigen::DiagonalMatrix<float, Eigen::Dynamic> m_massMatrixInv;

    Eigen::SparseMatrix<float> m_Lhs[3];
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float> > m_cholesky[3];

    int m_timeCollisionDetection;
    int m_timeLocalSolve;
    int m_timeGlobalSolve;


public:

    Eigen::VectorXf m_q[3];
    Eigen::VectorXf m_v[3];
    Eigen::VectorXf m_fext[3];
    Eigen::VectorXf m_sn[3];

    std::vector<Eigen::Vector3f> m_projected;
    std::vector<ProjectiveConstraint*> m_constraints;
    float m_collisionStiffness;


    Simulator(float dt, int iterations, std::vector<ProjectiveBody *> &bodies, float collisionStiffness);
    virtual ~Simulator();

    void initialize(float dt, int iterations, vector<ProjectiveBody*> &bodies, float collisionStiffness);

    void advanceTime();


    vector<CollisionInfo *> getCollisions() const { return m_collisions; }
};

#endif // SIMULATOR_H
