#include "simulator.h"
#include "utils.h"

#include <QTime>

Simulator::Simulator(TriMesh *mesh, vector<TriMesh *> &staticBodies, float dt, int iterations, double particleMass,
                     vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
                     vector<pair<int,int> > &springConstraints, float springStiffness,
                     vector<vector<int> > &tetraConstraints, float tetraStiffness,
                     float collisionStiffness, float positionStiffness)
{
    m_mesh = mesh;
    m_staticBodies = staticBodies;
    m_numParticles = mesh->NumVertices();
    m_particleMass = particleMass;
    m_iterations = iterations;
    m_springStiffness = springStiffness;
    m_tetraStiffness = tetraStiffness;
    m_collisionStiffness = collisionStiffness;
    m_positionStiffness = positionStiffness;

    initialize(dt, hardConstraintsIndices, hardConstraintsVector, springConstraints, tetraConstraints);
}

Simulator::~Simulator()
{
    if(m_meshCCDHandler)
        delete(m_meshCCDHandler);
    m_meshCCDHandler = 0;

    for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
        delete(m_staticBodiesCCDHandler[i]);
    m_staticBodiesCCDHandler.clear();
}

void Simulator::initialize(float dt, vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector, vector<pair<int, int> > &springConstraints, vector<vector<int> > &tetraConstraints)
{
    m_dt = dt;

    for(int i=0;i<m_springConstraints.size();++i)
        delete(m_springConstraints[i]);
    for(int i=0;i<m_positionConstraints.size();++i)
        delete(m_positionConstraints[i]);
    for(int i=0;i<m_tetraConstraints.size();++i)
        delete(m_tetraConstraints[i]);
    m_springConstraints.clear();
    m_positionConstraints.clear();
    m_tetraConstraints.clear();

    Eigen::VectorXf massVector(3*m_numParticles);
    massVector.setOnes();
    massVector *= m_particleMass;
    m_massMatrix = massVector.asDiagonal();
    m_massMatrixInv = m_massMatrix.inverse();

    m_Lhs = m_massMatrix * (1.0/(dt*dt));
    for(int i = 0; i < springConstraints.size(); ++i)
    {
        int v1 = springConstraints[i].first;
        int v2 = springConstraints[i].second;

        Eigen::Vector3f p1 = toEigenVector3(m_mesh->Vertices()[v1]->PositionInit());
        Eigen::Vector3f p2 = toEigenVector3(m_mesh->Vertices()[v2]->PositionInit());

        SpringConstraint *c = new SpringConstraint(m_springStiffness, p1, p2, v1, v2);
        m_springConstraints.push_back(c);
        m_constraints.push_back(c);
    }

    for(int i = 0; i < tetraConstraints.size(); ++i)
    {
        int v1 = tetraConstraints[i][0];
        int v2 = tetraConstraints[i][1];
        int v3 = tetraConstraints[i][2];
        int v4 = tetraConstraints[i][3];

        Eigen::Vector3f p1 = toEigenVector3(m_mesh->Vertices()[v1]->PositionInit());
        Eigen::Vector3f p2 = toEigenVector3(m_mesh->Vertices()[v2]->PositionInit());
        Eigen::Vector3f p3 = toEigenVector3(m_mesh->Vertices()[v3]->PositionInit());
        Eigen::Vector3f p4 = toEigenVector3(m_mesh->Vertices()[v4]->PositionInit());

        TetraConstraint *c = new TetraConstraint(m_tetraStiffness, p1, p2, p3, p4, v1, v2, v3, v4);
        m_tetraConstraints.push_back(c);
        m_constraints.push_back(c);
    }

    for (int i = 0; i < hardConstraintsIndices.size(); ++i)
    {
        PositionConstraint *c = new PositionConstraint(m_positionStiffness, hardConstraintsVector[i], hardConstraintsIndices[i]);
        m_positionConstraints.push_back(c);
        m_constraints.push_back(c);
    }

    for (int i=0; i < m_constraints.size(); ++i)
    {
        Constraint* c = m_constraints[i];
        Eigen::MatrixXf Ai = c->getAMatrix();

        Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
        Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
        Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);

        m_Lhs = m_Lhs + c->m_stiffness * SiX.transpose() * Ai.transpose() * Ai * SiX;
        m_Lhs = m_Lhs + c->m_stiffness * SiY.transpose() * Ai.transpose() * Ai * SiY;
        m_Lhs = m_Lhs + c->m_stiffness * SiZ.transpose() * Ai.transpose() * Ai * SiZ;
    }


    m_cholesky.compute(m_Lhs);

    m_fext.resize(3*m_numParticles);
    m_fext.setZero();
    for(int i=0;i<m_numParticles;++i)
        m_fext[i*3+1] = -9.8;

    m_meshCCDHandler = new HashCCDHandler(m_mesh);
    m_meshCCDHandler->Init();
    m_mesh->DeformWithoutInterpolation();
    m_staticBodiesCCDHandler.clear();
    for(int i=0;i<m_staticBodies.size();++i)
    {
        m_staticBodiesCCDHandler.push_back(new HashCCDHandler(m_staticBodies[i]));
        m_staticBodiesCCDHandler[i]->Init();
    }
}

void Simulator::advanceTime(vector<Eigen::Vector3f> &hardConstraintsVector)
{   
    QTime myTotalTimer;
    myTotalTimer.start();

    Eigen::VectorXf qn(3*m_numParticles), vn(3*m_numParticles);
    for(int i=0;i<m_numParticles;++i)
    {
        Vector3 q = m_mesh->Vertices()[i]->Position();
        qn[i*3+0] = q[0];
        qn[i*3+1] = q[1];
        qn[i*3+2] = q[2];

        Vector3 qold = m_mesh->Vertices()[i]->PositionOld();
        Vector3 v = (q - qold) * (1/m_dt);
        vn[i*3+0] = v[0];
        vn[i*3+1] = v[1];
        vn[i*3+2] = v[2];
    }

    Eigen::VectorXf sn = qn + m_dt*vn + m_dt*m_dt*m_massMatrixInv*m_fext;

    Eigen::VectorXf rhs(3*m_numParticles);
    m_collisions.clear();
    for(int iter=0;iter<m_iterations;++iter)
    {
        QTime myTimer, myIterTimer;
        myIterTimer.start();
        myTimer.start();


        m_meshCCDHandler->PreQuery(false);
        for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
            m_staticBodiesCCDHandler[i]->PreQuery(false);


        for(int i=0;i<m_staticBodiesCCDHandler.size();++i)
        {
            if(m_meshCCDHandler->GetBox().intersect(m_staticBodiesCCDHandler[i]->GetBox()))
            {
                vector<CollisionInfo*> newcollisions = CDQueries::HashQuery(m_meshCCDHandler, m_staticBodiesCCDHandler[i], false);
                m_collisions.insert(m_collisions.end(), newcollisions.begin(), newcollisions.end());
            }
            m_staticBodiesCCDHandler[i]->PostQuery();
        }
        m_meshCCDHandler->PostQuery();

        m_timeCollisionDetection = myTimer.elapsed();

        rhs.setZero();
        m_projected.clear();

        myTimer.restart();

#pragma omp parallel for
        for(int i=0;i<m_constraints.size(); ++ i)
        {
            Constraint *c = m_constraints[i];

            int v1 = c->getVIndex(0);
            int v2 = c->getVIndex(1);
            Eigen::Vector3f q1 = toEigenVector3(m_mesh->Vertices()[v1]->Position());
            Eigen::Vector3f q2 = toEigenVector3(m_mesh->Vertices()[v2]->Position());

            std::vector<Eigen::Vector3f> q, p;
            q.push_back(q1);
            q.push_back(q2);
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
                rhs += c->m_stiffness * SiX.transpose() * Ai.transpose() * Bi * pdim[0];
                rhs += c->m_stiffness * SiY.transpose() * Ai.transpose() * Bi * pdim[1];
                rhs += c->m_stiffness * SiZ.transpose() * Ai.transpose() * Bi * pdim[2];
            }
        }



        Eigen::SparseMatrix<float> collisionsLHS = m_Lhs;
        collisionsLHS.setZero();

#pragma omp parallel for
        for(int i=0;i<m_collisions.size(); ++ i)
        {
            CollisionInfoVF* col = (CollisionInfoVF*) m_collisions[i];
            if(col->objsSwitched)
                continue;
            col->n.normalize();
            LA::Vector3 p_projLA = col->v->Position() - Vector3::dotProd(col->v->Position() - col->p, col->n) * col->n;
            Eigen::Vector3f p_proj = toEigenVector3(p_projLA);

            PositionConstraint c(m_collisionStiffness, p_proj, col->v->Id());

            int v = c.getVIndex(0);
            Eigen::Vector3f q = toEigenVector3(m_mesh->Vertices()[v]->Position());

            Eigen::Vector3f p;
            c.project(p_proj, p);
            m_projected.push_back(p);

            Eigen::SparseMatrix<float> SiX = c.getSMatrix(m_numParticles, 0);
            Eigen::SparseMatrix<float> SiY = c.getSMatrix(m_numParticles, 1);
            Eigen::SparseMatrix<float> SiZ = c.getSMatrix(m_numParticles, 2);

#pragma omp critical
            {
                rhs += c.m_stiffness * SiX.transpose() * p[0];
                rhs += c.m_stiffness * SiY.transpose() * p[1];
                rhs += c.m_stiffness * SiZ.transpose() * p[2];
            }


            collisionsLHS = collisionsLHS + c.m_stiffness * SiX.transpose() * SiX;
            collisionsLHS = collisionsLHS + c.m_stiffness * SiY.transpose() * SiY;
            collisionsLHS = collisionsLHS + c.m_stiffness * SiZ.transpose() * SiZ;
        }
        m_Lhs = m_Lhs + collisionsLHS;

        m_timeLocalSolve = myTimer.elapsed();
        myTimer.restart();

        //        Eigen::DiagonalMatrix<float, Eigen::Dynamic> M = m_massMatrix;
        rhs += (1.0/(m_dt*m_dt)) * m_massMatrix * sn;

        m_cholesky.compute(m_Lhs);
        Eigen::VectorXf qn_1 = m_cholesky.solve(rhs);

        m_Lhs = m_Lhs - collisionsLHS;

        m_timeGlobalSolve = myTimer.elapsed();

        for(int i=0;i<m_positionConstraints.size();++i)
        {
            qn_1[m_positionConstraints[i]->getVIndex(0)*3+0] = m_positionConstraints[i]->getPosition()[0];
            qn_1[m_positionConstraints[i]->getVIndex(0)*3+1] = m_positionConstraints[i]->getPosition()[1];
            qn_1[m_positionConstraints[i]->getVIndex(0)*3+2] = m_positionConstraints[i]->getPosition()[2];
        }

        for(int i=0;i<m_numParticles;++i)
        {
            m_mesh->Vertices()[i]->UpdatePositionOld();
            m_mesh->Vertices()[i]->Position(qn_1[i*3+0], qn_1[i*3+1], qn_1[i*3+2]);
        }

        //        cout << "##################################################" << endl;
        //        cout << "Iter " << iter << endl;
        //        cout << "Collision Time: " << m_timeCollisionDetection << endl;
        //        cout << "Local Solve Time: " << m_timeLocalSolve << endl;
        //        cout << "Global Solve Time: " << m_timeGlobalSolve << endl;
        //        cout << "Total Iter Time: " << myIterTimer.elapsed() << endl;
        //        cout << "##################################################" << endl;
    }

    cout << "Total Time: " << myTotalTimer.elapsed() << endl;
}
