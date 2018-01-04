#include "simulator.h"
#include "utils.h"

Simulator::Simulator(TriMesh *mesh, vector<TriMesh *> &rigidBodies, float dt, int iterations, double particleMass,
                     vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector,
                     vector<pair<int,int> > &springConstraints, float springStiffness, float collisionStiffness)
{
    m_mesh = mesh;
    m_rigidBodies = rigidBodies;
    m_numParticles = mesh->NumVertices();
    m_particleMass = particleMass;
    m_iterations = iterations;
    m_springStiffness = springStiffness;
    m_collisionStiffness = collisionStiffness;

    initialize(dt, hardConstraintsIndices, hardConstraintsVector, springConstraints);
}

Simulator::~Simulator()
{
    if(m_meshCCDHandler)
        delete(m_meshCCDHandler);
    m_meshCCDHandler = 0;

    for(int i=0;i<m_rigidBodiesCCDHandler.size();++i)
        delete(m_rigidBodiesCCDHandler[i]);
    m_rigidBodiesCCDHandler.clear();
}

void Simulator::initialize(float dt, vector<int> &hardConstraintsIndices, vector<Eigen::Vector3f> &hardConstraintsVector, vector<pair<int, int> > &springConstraints)
{
    m_dt = dt;

    for(int i=0;i<m_springConstraints.size();++i)
        delete(m_springConstraints[i]);
    for(int i=0;i<m_positionConstraints.size();++i)
        delete(m_positionConstraints[i]);
    m_springConstraints.clear();
    m_positionConstraints.clear();

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

        Eigen::Matrix2f Ai = c->getAMatrix();

        Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
        Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
        Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);

        m_Lhs = m_Lhs + c->m_stiffness * SiX.transpose() * Ai.transpose() * Ai * SiX;
        m_Lhs = m_Lhs + c->m_stiffness * SiY.transpose() * Ai.transpose() * Ai * SiY;
        m_Lhs = m_Lhs + c->m_stiffness * SiZ.transpose() * Ai.transpose() * Ai * SiZ;
    }

    for (int i = 0; i < hardConstraintsIndices.size(); ++i)
    {
        PositionConstraint *c = new PositionConstraint(m_collisionStiffness, hardConstraintsVector[i], hardConstraintsIndices[i]);
        m_positionConstraints.push_back(c);

        Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
        Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
        Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);

        m_Lhs = m_Lhs + c->m_stiffness * SiX.transpose() * SiX;
        m_Lhs = m_Lhs + c->m_stiffness * SiY.transpose() * SiY;
        m_Lhs = m_Lhs + c->m_stiffness * SiZ.transpose() * SiZ;
    }

    m_cholesky.compute(m_Lhs);

    m_fext.resize(3*m_numParticles);
    m_fext.setZero();
    for(int i=0;i<m_numParticles;++i)
        m_fext[i*3+1] = -9.8;

    m_meshCCDHandler = new HashCCDHandler(m_mesh);
    m_meshCCDHandler->Init();
    m_mesh->DeformWithoutInterpolation();
    m_rigidBodiesCCDHandler.clear();
    for(int i=0;i<m_rigidBodies.size();++i)
    {
        m_rigidBodiesCCDHandler.push_back(new HashCCDHandler(m_rigidBodies[i]));
        m_rigidBodiesCCDHandler[i]->Init();
    }
}

void Simulator::advanceTime(vector<Eigen::Vector3f> &hardConstraintsVector)
{   
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
        m_meshCCDHandler->PreQuery(false);
        for(int i=0;i<m_rigidBodiesCCDHandler.size();++i)
            m_rigidBodiesCCDHandler[i]->PreQuery(false);


        for(int i=0;i<m_rigidBodiesCCDHandler.size();++i)
        {
            if(m_meshCCDHandler->GetBox().intersect(m_rigidBodiesCCDHandler[i]->GetBox()))
            {
                vector<CollisionInfo*> newcollisions = CDQueries::HashQuery(m_meshCCDHandler, m_rigidBodiesCCDHandler[i], false);
                m_collisions.insert(m_collisions.end(), newcollisions.begin(), newcollisions.end());
            }
            m_rigidBodiesCCDHandler[i]->PostQuery();
        }
        m_meshCCDHandler->PostQuery();

        rhs.setZero();
        m_projected.clear();

//        #pragma omp parallel for
        for(int i=0;i<m_springConstraints.size(); ++ i)
        {
            SpringConstraint *c = m_springConstraints[i];

            int v1 = c->getVIndex1();
            int v2 = c->getVIndex2();
            Eigen::Vector3f q1 = toEigenVector3(m_mesh->Vertices()[v1]->Position());
            Eigen::Vector3f q2 = toEigenVector3(m_mesh->Vertices()[v2]->Position());

            Eigen::Vector3f p1, p2;
            c->project(q1, q2, p1, p2);

            m_projected.push_back(p1);
            m_projected.push_back(p2);

            Eigen::Matrix2f Ai = c->getAMatrix();
            Eigen::Matrix2f Bi = c->getBMatrix();

            Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
            Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
            Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);


            Eigen::Vector2f px, py, pz;
            px[0] = p1[0];
            px[1] = p2[0];
            py[0] = p1[1];
            py[1] = p2[1];
            pz[0] = p1[2];
            pz[1] = p2[2];

//            #pragma omp critical
            {
                rhs += c->m_stiffness * SiX.transpose() * Ai.transpose() * Bi * px;
                rhs += c->m_stiffness * SiY.transpose() * Ai.transpose() * Bi * py;
                rhs += c->m_stiffness * SiZ.transpose() * Ai.transpose() * Bi * pz;
            }

        }

//        #pragma omp parallel for
        for(int i=0;i<m_positionConstraints.size(); ++ i)
        {
            PositionConstraint *c = m_positionConstraints[i];

            Eigen::Vector3f p;
            c->project(hardConstraintsVector[i], p);
            m_projected.push_back(p);

            Eigen::SparseMatrix<float> SiX = c->getSMatrix(m_numParticles, 0);
            Eigen::SparseMatrix<float> SiY = c->getSMatrix(m_numParticles, 1);
            Eigen::SparseMatrix<float> SiZ = c->getSMatrix(m_numParticles, 2);

//            #pragma omp critical
            {
                rhs += c->m_stiffness * SiX.transpose() * p[0];
                rhs += c->m_stiffness * SiY.transpose() * p[1];
                rhs += c->m_stiffness * SiZ.transpose() * p[2];
            }
        }

        Eigen::SparseMatrix<float> collisionsLHS = m_Lhs;
        collisionsLHS.setZero();
//        #pragma omp parallel for
        for(int i=0;i<m_collisions.size(); ++ i)
        {
            CollisionInfoVF* col = (CollisionInfoVF*) m_collisions[i];
            if(col->objsSwitched)
                continue;
            col->n.normalize();
            LA::Vector3 p_projLA = col->v->Position() - Vector3::dotProd(col->v->Position() - col->p, col->n) * col->n;
            Eigen::Vector3f p_proj = toEigenVector3(p_projLA);

            PositionConstraint c(m_collisionStiffness, p_proj, col->v->Id());

            int v = c.getVIndex();
            Eigen::Vector3f q = toEigenVector3(m_mesh->Vertices()[v]->Position());

            Eigen::Vector3f p;
            c.project(p_proj, p);
            m_projected.push_back(p);

            Eigen::SparseMatrix<float> SiX = c.getSMatrix(m_numParticles, 0);
            Eigen::SparseMatrix<float> SiY = c.getSMatrix(m_numParticles, 1);
            Eigen::SparseMatrix<float> SiZ = c.getSMatrix(m_numParticles, 2);

//            #pragma omp critical
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


        Eigen::DiagonalMatrix<float, Eigen::Dynamic> M = m_massMatrix;
        rhs += (1.0/(m_dt*m_dt)) * m_massMatrix * sn;

        m_cholesky.compute(m_Lhs);
        Eigen::VectorXf qn_1 = m_cholesky.solve(rhs);

        m_Lhs = m_Lhs - collisionsLHS;


        for(int i=0;i<m_positionConstraints.size();++i)
        {
            qn_1[m_positionConstraints[i]->getVIndex()*3+0] = m_positionConstraints[i]->getPosition()[0];
            qn_1[m_positionConstraints[i]->getVIndex()*3+1] = m_positionConstraints[i]->getPosition()[1];
            qn_1[m_positionConstraints[i]->getVIndex()*3+2] = m_positionConstraints[i]->getPosition()[2];
        }

        for(int i=0;i<m_numParticles;++i)
        {
            m_mesh->Vertices()[i]->UpdatePositionOld();
            m_mesh->Vertices()[i]->Position(qn_1[i*3+0], qn_1[i*3+1], qn_1[i*3+2]);
        }
    }
}
