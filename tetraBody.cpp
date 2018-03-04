#include <LinearInterpolation.h>

#include "tetraBody.h"
#include "utils.h"



TetraBody::TetraBody(TriMesh *mesh, std::string name, float totalMass, float tetraStiffness, int numTetras, bool deleteExtraTetras, float damping, float restitution)
    : ProjectiveBody(name, TETRA, totalMass, damping, restitution), m_mesh(mesh)
{
    m_tetraMesh = TetraMesh::CreateEmbeddingMesh(numTetras, mesh, deleteExtraTetras);
    m_mesh = m_tetraMesh->LinkTriMesh(mesh);

    m_numParticles = m_tetraMesh->numVertices();
    for(int i=0; i<m_numParticles; ++i)
    {
        m_tetraMesh->getVertex()[i]._velocity.set(0,0,0);
    }

    m_tetraConstraints.clear();
    for(int i=0;i<m_tetraMesh->numTetras();++i)
    {
        Tetra *tetra = &(m_tetraMesh->getTetra()[i]);
        Vertex* v1 = tetra->_vertex[0];
        Vertex* v2 = tetra->_vertex[1];
        Vertex* v3 = tetra->_vertex[2];
        Vertex* v4 = tetra->_vertex[3];

        Eigen::Vector3f p1 = toEigenVector3(v1->getPosition());
        Eigen::Vector3f p2 = toEigenVector3(v2->getPosition());
        Eigen::Vector3f p3 = toEigenVector3(v3->getPosition());
        Eigen::Vector3f p4 = toEigenVector3(v4->getPosition());

        TetraConstraint *c = new TetraConstraint(this, tetraStiffness, p1, p2, p3, p4, v1->_id, v2->_id, v3->_id, v4->_id);
        m_tetraConstraints.push_back(c);
    }

    m_CCDHandler = new HashCCDHandler(m_mesh);
    m_CCDHandler->Init();
}

Eigen::VectorXf TetraBody::getPositions()
{
    Eigen::VectorXf q;
    q.resize(m_numParticles*3);
    for(int i=0; i<m_numParticles; ++i)
    {
        q[i*3+0] = m_tetraMesh->getVertex()[i].getPosition()[0];
        q[i*3+1] = m_tetraMesh->getVertex()[i].getPosition()[1];
        q[i*3+2] = m_tetraMesh->getVertex()[i].getPosition()[2];
    }
    return q;
}

Eigen::VectorXf TetraBody::getVelocities(float dt)
{
    Eigen::VectorXf v;
    v.resize(m_numParticles*3);
    for(int i=0; i<m_numParticles; ++i)
    {
        LA::Vector3 cv = m_tetraMesh->getVertex()[i].getVelocity();
        v[i*3+0] = cv[0];
        v[i*3+1] = cv[1];
        v[i*3+2] = cv[2];
    }
    return v;
}

void TetraBody::setPositions(const Eigen::VectorXf &qx, const Eigen::VectorXf &qy, const Eigen::VectorXf &qz)
{
    for(int i=0; i<m_numParticles; ++i)
    {
        Vertex* v = &(m_tetraMesh->getVertex()[i]);
        v->setPosition(LA::Vector3(qx[i], qy[i], qz[i]));

//        std::cout << "New positions " << i << ": " << qx[i] << " " << qy[i] << " " << qz[i] << std::endl;
    }
    m_mesh->ComputePositions();
}

std::vector<ProjectiveConstraint *> TetraBody::getConstraints()
{
    std::vector<ProjectiveConstraint *> result;
    result.insert(result.end(), m_tetraConstraints.begin(), m_tetraConstraints.end());
    result.insert(result.end(), m_positionConstraints.begin(), m_positionConstraints.end());
    return result;
}

void TetraBody::addPositionConstraint(float stiffness, int vIndex)
{
    TriVertex *v = m_mesh->Vertices()[vIndex];
    linearinterpolationlib::LinearInterpolation* inter = dynamic_cast<linearinterpolationlib::LinearInterpolation*>(v->GetInterpolation());

    Vertex* nodes = m_tetraMesh->getVertex();

    for(int i=0; i<4; ++i)
    {
        int index = inter->tet->_vertex[i]->_id;
        Eigen::Vector3f p = toEigenVector3(nodes[index].getPosition());
        PositionConstraint *c = new PositionConstraint(this, stiffness/4, p, index);
        m_positionConstraints.push_back(c);
    }
}

std::vector<PositionConstraint> TetraBody::getPositionConstraints(float stiffness, int vIndex, Eigen::Vector3f &position)
{
    std::vector<PositionConstraint> constraints;

    TriVertex *v = m_mesh->Vertices()[vIndex];
    linearinterpolationlib::LinearInterpolation* inter = dynamic_cast<linearinterpolationlib::LinearInterpolation*>(v->GetInterpolation());
    LA::Vector4 weights = inter->weights;
    Eigen::Vector3f offset = position - toEigenVector3(v->Position());

//    std::cout << "Collision position: " << position.transpose() << std::endl;

    Vertex* nodes = m_tetraMesh->getVertex();
    for(int i=0; i<4; ++i)
    {
        int index = inter->tet->_vertex[i]->_id;
        Eigen::Vector3f pold = toEigenVector3(nodes[index].getPosition());
        Eigen::Vector3f p = pold + offset*weights[i];
        constraints.push_back(PositionConstraint(this, stiffness/4, p, index));

//        std::cout << "\t" << index << ":" << pold.transpose() << " --> " << p.transpose() << std::endl;
    }

    return constraints;
}

void TetraBody::draw()
{
    m_mesh->Draw();
    //    m_tetraMesh->draw();
}

void TetraBody::addVelocity(Eigen::Vector3f v)
{
    LA::Vector3 vLA = toLAVector3(v);
    for(int i=0; i<m_numParticles; ++i)
    {
        m_tetraMesh->getVertex()[i]._velocity = vLA;
    }
}
