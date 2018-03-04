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
public:
    int m_numParticles;

    vector<ProjectiveBody*> m_bodies;
    map<ProjectiveBody*, int> m_bodyToIndex;


    struct Collision
    {
        ProjectiveBody* body1;
        ProjectiveBody* body2;
        CollisionInfo* info;

        Collision(ProjectiveBody* b1, ProjectiveBody* b2, CollisionInfo* i)
        {
            body1 = b1;
            body2 = b2;
            info = i;
        }
    };

    vector<Collision> m_collisions;
    bool m_collisionsInPreviousFrame;


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



    Eigen::VectorXf m_q[3];
    Eigen::VectorXf m_v[3];
    Eigen::VectorXf m_fext[3];
    Eigen::VectorXf m_sn[3];

    std::vector<Eigen::Vector3f> m_projected;
    std::vector<Eigen::Vector3f> m_projectedCollisions;
    std::vector<ProjectiveConstraint*> m_constraints;
    float m_collisionStiffness;


    Simulator(float dt, int iterations, std::vector<ProjectiveBody *> &bodies, float collisionStiffness);
    virtual ~Simulator();

    void initialize(float dt, int iterations, vector<ProjectiveBody*> &bodies, float collisionStiffness);

    void advanceTime();


    vector<Collision> getCollisions() const { return m_collisions; }
};

#endif // SIMULATOR_H
