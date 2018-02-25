#include "simulator.h"
#include "utils.h"

#include <QTime>

Simulator::Simulator(float dt, int iterations, std::vector<ProjectiveBody*> &bodies)
{
    initialize(dt, iterations, bodies);
}

Simulator::~Simulator()
{
//    if(m_meshCCDHandler)
//        delete(m_meshCCDHandler);
//    m_meshCCDHandler = 0;

//    for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
//        delete(m_staticBodiesCCDHandler[i]);
//    m_staticBodiesCCDHandler.clear();
}

void Simulator::initialize(float dt, int iterations, vector<ProjectiveBody*> &bodies)
{
    m_dt = dt;
    m_iterations = iterations;
    m_q[0].resize(0);
    m_q[1].resize(0);
    m_q[2].resize(0);
    m_particleMass.resize(0);
    m_constraints.clear();
    for(int i=0; i<bodies.size(); ++i)
    {
        m_bodies.push_back(bodies[i]);
        m_q[0].resize(m_q[0].rows()+bodies[i]->getNumParticles());
        m_q[1].resize(m_q[1].rows()+bodies[i]->getNumParticles());
        m_q[2].resize(m_q[2].rows()+bodies[i]->getNumParticles());
        m_particleMass.resize(m_particleMass.rows()+bodies[i]->getNumParticles());
        Eigen::VectorXf pos = bodies[i]->getPositions();
        for(int j=0; j<pos.rows()/3; ++j)
        {
            m_q[0][j] = pos[j*3+0];
            m_q[1][j] = pos[j*3+1];
            m_q[2][j] = pos[j*3+2];
        }
        m_particleMass << bodies[i]->getParticleMassVector();
        std::vector<ProjectiveConstraint*> constraints = bodies[i]->getConstraints();
        m_constraints.insert(m_constraints.end(), constraints.begin(), constraints.end());
    }

    m_v[0].resize(m_q[0].rows());
    m_v[0].setZero();
    m_v[1].resize(m_q[1].rows());
    m_v[1].setZero();
    m_v[2].resize(m_q[2].rows());
    m_v[2].setZero();

    m_numParticles = m_q[0].rows();

    m_massMatrix = m_particleMass.asDiagonal();
    m_massMatrixInv = m_massMatrix.inverse();

    m_Lhs[0] = m_massMatrix * (1.0/(dt*dt));
    m_Lhs[1] = m_massMatrix * (1.0/(dt*dt));
    m_Lhs[2] = m_massMatrix * (1.0/(dt*dt));

    for (int i=0; i < m_constraints.size(); ++i)
    {
        ProjectiveConstraint* c = m_constraints[i];
        Eigen::MatrixXf Ai = c->getAMatrix();

        Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
        Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
        Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);

        m_Lhs[0] = m_Lhs[0] + c->m_stiffness * SiX.transpose() * Ai.transpose() * Ai * SiX;
        m_Lhs[1] = m_Lhs[1] + c->m_stiffness * SiY.transpose() * Ai.transpose() * Ai * SiY;
        m_Lhs[2] = m_Lhs[2] + c->m_stiffness * SiZ.transpose() * Ai.transpose() * Ai * SiZ;
    }

    m_cholesky[0].compute(m_Lhs[0]);
    m_cholesky[1].compute(m_Lhs[1]);
    m_cholesky[2].compute(m_Lhs[2]);

    m_fext[0].resize(m_numParticles);
    m_fext[0].setZero();
    m_fext[1].resize(m_numParticles);
    m_fext[1].setConstant(-9.8);
    m_fext[2].resize(m_numParticles);
    m_fext[2].setZero();
}

void Simulator::advanceTime()
{   
    QTime myTotalTimer;
    myTotalTimer.start();

    Eigen::VectorXf qold[3], sn[3], rhs[3];
    qold[0] = m_q[0];
    qold[1] = m_q[1];
    qold[2] = m_q[2];

    sn[0] = m_q[0] + m_dt*m_v[0] + m_dt*m_dt*m_massMatrixInv*m_fext[0];
    sn[1] = m_q[1] + m_dt*m_v[1] + m_dt*m_dt*m_massMatrixInv*m_fext[1];
    sn[2] = m_q[2] + m_dt*m_v[2] + m_dt*m_dt*m_massMatrixInv*m_fext[2];

    rhs[0].resize(m_numParticles);
    rhs[1].resize(m_numParticles);
    rhs[2].resize(m_numParticles);
//    m_collisions.clear();
    for(int iter=0;iter<m_iterations;++iter)
    {
        QTime myTimer, myIterTimer;
        myIterTimer.start();
        myTimer.start();

//        m_meshCCDHandler->PreQuery(false);
//        for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
//            m_staticBodiesCCDHandler[i]->PreQuery(false);


//        for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
//        {
//            if(m_meshCCDHandler->GetBox().intersect(m_staticBodiesCCDHandler[i]->GetBox()))
//            {
//                vector<CollisionInfo*> newcollisions = CDQueries::HashQuery(m_meshCCDHandler, m_staticBodiesCCDHandler[i], false);
//                m_collisions.insert(m_collisions.end(), newcollisions.begin(), newcollisions.end());
//            }
//            m_staticBodiesCCDHandler[i]->PostQuery();
//        }
//        m_meshCCDHandler->PostQuery();

        m_timeCollisionDetection = myTimer.elapsed();

        rhs[0].setZero();
        rhs[1].setZero();
        rhs[2].setZero();
        m_projected.clear();

        myTimer.restart();

#pragma omp parallel for
        for(int i=0;i<m_constraints.size(); ++ i)
        {
            ProjectiveConstraint *c = m_constraints[i];

            std::vector<Eigen::Vector3f> q, p;
            for(int j=0; j<c->m_numParticles; ++j)
            {
                Eigen::Vector3f qpos = toEigenVector3(m_q[0], m_q[1], m_q[2], c->getVIndex(j));
                q.push_back(qpos);
            }

            c->project(q, p);

            m_projected.insert(m_projected.end(), p.begin(), p.end());

            Eigen::MatrixXf Ai = c->getAMatrix();
            Eigen::MatrixXf Bi = c->getBMatrix();

            Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
            Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
            Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);


            std::vector<Eigen::VectorXf> pdim(3);
            pdim[0].resize(c->m_numParticles);
            pdim[1].resize(c->m_numParticles);
            pdim[2].resize(c->m_numParticles);

            for(int j=0; j<c->m_numParticles; ++j)
            {
                for(int k=0; k<3; ++k)
                    pdim[k][j] = p[j][k];
            }

#pragma omp critical
            {
                rhs[0] += c->m_stiffness * SiX.transpose() * Ai.transpose() * Bi * pdim[0];
                rhs[1] += c->m_stiffness * SiY.transpose() * Ai.transpose() * Bi * pdim[1];
                rhs[2] += c->m_stiffness * SiZ.transpose() * Ai.transpose() * Bi * pdim[2];
            }
        }

//        Eigen::SparseMatrix<float> collisionsLHS = m_Lhs;
//        collisionsLHS.setZero();

//#pragma omp parallel for
//        for(int i=0;i<m_collisions.size(); ++ i)
//        {
//            CollisionInfoVF* col = (CollisionInfoVF*) m_collisions[i];
//            if(col->objsSwitched)
//                continue;
//            col->n.normalize();
//            LA::Vector3 p_projLA = col->v->Position() - Vector3::dotProd(col->v->Position() - col->p, col->n) * col->n;
//            Eigen::Vector3f p_proj = toEigenVector3(p_projLA);

//            PositionConstraint c(m_collisionStiffness, p_proj, col->v->Id());

//            int v = c.getVIndex(0);
//            Eigen::Vector3f q = toEigenVector3(m_mesh->Vertices()[v]->Position());

//            Eigen::Vector3f p;
//            c.project(p_proj, p);
//            m_projected.push_back(p);

//            Eigen::SparseMatrix<float> SiX = c.getSMatrix(m_numParticles, 0);
//            Eigen::SparseMatrix<float> SiY = c.getSMatrix(m_numParticles, 1);
//            Eigen::SparseMatrix<float> SiZ = c.getSMatrix(m_numParticles, 2);

//#pragma omp critical
//            {
//                rhs += c.m_stiffness * SiX.transpose() * p[0];
//                rhs += c.m_stiffness * SiY.transpose() * p[1];
//                rhs += c.m_stiffness * SiZ.transpose() * p[2];
//            }


//            collisionsLHS = collisionsLHS + c.m_stiffness * SiX.transpose() * SiX;
//            collisionsLHS = collisionsLHS + c.m_stiffness * SiY.transpose() * SiY;
//            collisionsLHS = collisionsLHS + c.m_stiffness * SiZ.transpose() * SiZ;
//        }
//        m_Lhs = m_Lhs + collisionsLHS;

        m_timeLocalSolve = myTimer.elapsed();
        myTimer.restart();

        rhs[0] += (1.0/(m_dt*m_dt)) * m_massMatrix * sn[0];
        rhs[1] += (1.0/(m_dt*m_dt)) * m_massMatrix * sn[1];
        rhs[2] += (1.0/(m_dt*m_dt)) * m_massMatrix * sn[2];

//        m_cholesky.compute(m_Lhs);
#pragma omp parallel for
        for(int k=0; k<3; ++k)
            m_q[k] = m_cholesky[k].solve(rhs[k]);

//        m_Lhs = m_Lhs - collisionsLHS;

        m_timeGlobalSolve = myTimer.elapsed();



        int index=0;
        for(int i=0; i<m_bodies.size(); ++i)
        {
            int numParticles = m_bodies[i]->getNumParticles();
            m_bodies[i]->setPositions(m_q[0].segment(index,numParticles), m_q[1].segment(index,numParticles), m_q[2].segment(index,numParticles));
            index += numParticles;
        }


        //        cout << "##################################################" << endl;
        //        cout << "Iter " << iter << endl;
        //        cout << "Collision Time: " << m_timeCollisionDetection << endl;
        //        cout << "Local Solve Time: " << m_timeLocalSolve << endl;
        //        cout << "Global Solve Time: " << m_timeGlobalSolve << endl;
        //        cout << "Total Iter Time: " << myIterTimer.elapsed() << endl;
        //        cout << "##################################################" << endl;
    }

    for(int i=0; i<m_q[0].rows(); ++i)
    {
        m_v[0] = (1/m_dt) * (m_q[0] - qold[0]);
        m_v[1] = (1/m_dt) * (m_q[1] - qold[1]);
        m_v[2] = (1/m_dt) * (m_q[2] - qold[2]);
    }

    cout << "Total Time: " << myTotalTimer.elapsed() << endl;
}
