#include "viewer.h"
#include <math.h>
#include <set>
#include <stdlib.h> // RAND_MAX

#include <QFileDialog>

#include <TriCirculator.h>
#include <Tetra.h>
#include <Vertex.h>

#include "utils.h"
#include "staticBody.h"
#include "tetraBody.h"


using namespace qglviewer;
using namespace std;

///////////////////////   V i e w e r  ///////////////////////
void Viewer::init()
{
    restoreStateFromFile();
    glDisable(GL_LIGHTING);

    m_bodies.clear();

    testSceneClothOnStaticBody();
//    testSceneClothConstrainedTopCorners();
//    testSceneClothConstrainedCorners();
//    testSceneClothDropping();
//    testSceneDeformableSphere();
//    testSceneDeformableBlock();
//    testSceneDeformableBlockDropping();

    m_simulator = new Simulator(m_dt, m_iterations, m_bodies, m_collisionStiffness);

    m_play = false;
    m_step = false;
    m_currentTime = 0;

//    help();
    startAnimation();
}

void Viewer::testSceneClothOnStaticBody()
{
    if(!m_init)
    {
        m_meshRows = 15;
        m_meshColumns = 15;
        m_dt = 0.01;
        m_iterations = 15;
        m_totalMass = 5;

        m_init = true;
    }

    int rows = m_meshRows;
    int cols = m_meshColumns;
    float meshCellSize = m_meshSize/m_meshRows;
    TriMesh* mesh = TriMesh::CreatePatchMesh(rows, cols, meshCellSize);
    mesh->GetFaceProperties()->invertNormal = true;
    mesh->Transform(Vector3(0,0.5,0), LA::Quaternion::FromAngleAxis(M_PI_2, Vector3(1,0,0)));
//    m_mesh->Transform(Vector3(0,0,0), LA::Quaternion::FromAngleAxis(M_PI_4, Vector3(0,0,1)));
    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    SpringBody* body = new SpringBody(mesh, "cloth", m_totalMass, m_springStiffness, true, 1.01*2*meshCellSize*meshCellSize, 0.9, 1);

    m_bodies.push_back(body);

    // static bodies

    // bunny
//    TriMesh *rb = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/bunny.obj", TriMesh::OBJ, 2.0f);
//    rb->Transform(Vector3(0,0.0,0), LA::Quaternion::FromAngleAxis(M_PI_2, Vector3(-1,0,0)));

    //sphere
    TriMesh *rb = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/sphere.obj", TriMesh::OBJ, 0.2f);

    StaticBody* staticBody = new StaticBody(rb, "floor");
    m_bodies.push_back(staticBody);
}

void Viewer::testSceneClothConstrainedTopCorners()
{
    int rows = m_meshRows;
    int cols = m_meshColumns;
    float meshCellSize = m_meshSize/m_meshRows;
    TriMesh* mesh = TriMesh::CreatePatchMesh(rows, cols, meshCellSize);

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    vector<TriVertex *> topVertices = *(mesh->getTriVertexOnTop(0.001));

    mesh->Transform(Vector3(0,0.5,0), LA::Quaternion::FromAngleAxis(M_PI_2, Vector3(1,0,0)));
    SpringBody* body = new SpringBody(mesh, "cloth", m_totalMass, m_springStiffness, true, 1.01*2*meshCellSize*meshCellSize);

    body->addPositionConstraint(m_positionStiffness, topVertices.front()->Id());
    body->addPositionConstraint(m_positionStiffness, topVertices.back()->Id());

    m_bodies.push_back(body);
}

void Viewer::testSceneClothConstrainedCorners()
{
    int rows = m_meshRows;
    int cols = m_meshColumns;
    float meshCellSize = m_meshSize/m_meshRows;
    TriMesh* mesh = TriMesh::CreatePatchMesh(rows, cols, meshCellSize);

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    vector<TriVertex *> topVertices = *(mesh->getTriVertexOnTop(0.001));
    vector<TriVertex *> bottomVertices = *(mesh->getTriVertexOnBottom(0.001));

    mesh->Transform(Vector3(0,0.5,0), LA::Quaternion::FromAngleAxis(M_PI_2, Vector3(1,0,0)));
    SpringBody* body = new SpringBody(mesh, "cloth", m_totalMass, m_springStiffness, true, 1.01*2*meshCellSize*meshCellSize);

    vector<TriVertex *> cornerVertices;
    cornerVertices.push_back(topVertices.front());
    cornerVertices.push_back(topVertices.back());
    cornerVertices.push_back(bottomVertices.front());
    cornerVertices.push_back(bottomVertices.back());
    for(int i=0;i<cornerVertices.size();++i)
    {
        body->addPositionConstraint(m_positionStiffness, cornerVertices[i]->Id());
    }

    m_bodies.push_back(body);
}

void Viewer::testSceneClothDropping()
{
    int rows = m_meshRows;
    int cols = m_meshColumns;
    float meshCellSize = m_meshSize/m_meshRows;
    TriMesh* mesh = TriMesh::CreatePatchMesh(rows, cols, meshCellSize);
    mesh->Transform(Vector3(0,0,0), LA::Quaternion::FromAngleAxis(0.1, Vector3(1,0,0)));

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    SpringBody* body = new SpringBody(mesh, "cloth", m_totalMass, m_springStiffness, true, 1.01*2*meshCellSize*meshCellSize);

    m_bodies.push_back(body);

    // static bodies
    TriMesh *rb = TriMesh::CreateBlockMesh(Vector3(-10,-10,-10), Vector3(10, -1, 10));

    StaticBody* floor = new StaticBody(rb, "floor");
    m_bodies.push_back(floor);


}

void Viewer::testSceneDeformableSphere()
{
    if(!m_init)
    {
        m_dt = 0.001;
        m_iterations = 10;
        m_tetraStiffness = 10000000;
        m_totalMass = 1;

        m_init = true;
    }

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    {
        TriMesh* mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/sphere.obj", TriMesh::OBJ, 0.1);
        TetraBody* body = new TetraBody(mesh, "sphere", m_totalMass, 10000000, 5, true, 0.98, 2);

//            body->addVelocity(Eigen::Vector3f(30,0,0));

        m_bodies.push_back(body);
    }

    {
        TriMesh* mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/sphere.obj", TriMesh::OBJ, 0.1);
        mesh->Transform(LA::Vector3(-0.3,0,0), LA::Quaternion(0,1,0,0));
        TetraBody* body = new TetraBody(mesh, "sphere", m_totalMass, 3000, 5, true, 0.98, 2);

        //    body->addVelocity(Eigen::Vector3f(10,0,0));

        m_bodies.push_back(body);
    }

    {
        TriMesh* mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/sphere.obj", TriMesh::OBJ, 0.1);
        mesh->Transform(LA::Vector3(-0.6,0,0), LA::Quaternion(0,1,0,0));
        TetraBody* body = new TetraBody(mesh, "sphere", m_totalMass, 500, 5, true, 0.98, 2);

        //    body->addVelocity(Eigen::Vector3f(10,0,0));

        m_bodies.push_back(body);
    }


    // static bodies
    TriMesh *rb = TriMesh::CreateBlockMesh(Vector3(-10,-10,-10), Vector3(10, -0.3, 10));

    StaticBody* floor = new StaticBody(rb, "floor");
    m_bodies.push_back(floor);
}

void Viewer::testSceneDeformableBlock()
{
    if(!m_init)
    {
        m_dt = 0.01;
        m_iterations = 1;
        m_totalMass = 10;

        m_init = true;
    }

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    TriMesh* mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/blockSubdiv.obj", TriMesh::OBJ, 0.1);
    LA::Bounds3d bbox(LA::Vector3(-1,-1,-1), LA::Vector3(0.01, 1, 1));
    std::vector<TriVertex*> leftVertices = *(mesh->getTriVertexInBB(bbox));
    mesh->Transform(LA::Vector3(0,0,-0.0), LA::Quaternion(0,1,0,0));

    TetraBody* body = new TetraBody(mesh, "block", m_totalMass, 10000, 15, true, 1, 1);


    for(int i=0;i<leftVertices.size();++i)
    {
        body->addPositionConstraint(m_positionStiffness, leftVertices[i]->Id());
    }

    m_bodies.push_back(body);



    TriMesh* mesh2 = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/blockSubdiv.obj", TriMesh::OBJ, 0.1);
    LA::Bounds3d bbox2(LA::Vector3(-1,-1,-1), LA::Vector3(0.01, 1, 1));
    std::vector<TriVertex*> leftVertices2 = *(mesh2->getTriVertexInBB(bbox2));
    mesh2->Transform(LA::Vector3(0,0,-0.3), LA::Quaternion(0,1,0,0));
    TetraBody* body2 = new TetraBody(mesh2, "block", m_totalMass, 100000, 15, true, 1, 1);

    for(int i=0;i<leftVertices2.size();++i)
    {
        body2->addPositionConstraint(m_positionStiffness, leftVertices2[i]->Id());
    }

    m_bodies.push_back(body2);



    TriMesh* mesh3 = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/blockSubdiv.obj", TriMesh::OBJ, 0.1);
    LA::Bounds3d bbox3(LA::Vector3(-1,-1,-1), LA::Vector3(0.01, 1, 1));
    std::vector<TriVertex*> leftVertices3 = *(mesh3->getTriVertexInBB(bbox2));
    mesh3->Transform(LA::Vector3(0,0,-0.6), LA::Quaternion(0,1,0,0));
    TetraBody* body3 = new TetraBody(mesh3, "block", m_totalMass, 1000000, 15, true, 1, 1);

    for(int i=0;i<leftVertices3.size();++i)
    {
        body3->addPositionConstraint(m_positionStiffness, leftVertices3[i]->Id());
    }

    m_bodies.push_back(body3);


}

void Viewer::testSceneDeformableBlockDropping()
{
    if(!m_init)
    {
        m_dt = 0.001;
        m_iterations = 3;

        m_init = true;
    }

    glPointSize(10.0);
    glBlendFunc(GL_ONE, GL_ONE);

    TriMesh* mesh = TriMesh::ReadFromFile("/Users/sarac.schvartzman/Dropbox/Code/blockSubdiv.obj", TriMesh::OBJ);
    mesh->Transform(Vector3(0,0,0), LA::Quaternion::FromAngleAxis(M_PI/4, Vector3(0,1,1)));
    TetraBody* body = new TetraBody(mesh, "block", m_totalMass, m_tetraStiffness, 10, true);
//    TriMesh *mesh = TriMesh::CreateBlockMesh(Vector3(-0.1,-0.1,-0.1), Vector3(0.1, 0.1, 0.1));
//    mesh->Transform(Vector3(0,0,0), LA::Quaternion::FromAngleAxis(M_PI/4, Vector3(0,1,1)));
//    TetraBody* body = new TetraBody(mesh, m_totalMass, m_tetraStiffness, 1, true);

    m_bodies.push_back(body);

    // static bodies
    TriMesh *rb = TriMesh::CreateBlockMesh(Vector3(-10,-10,-10), Vector3(10, -0.2, 10));

    StaticBody* floor = new StaticBody(rb, "floor");
    m_bodies.push_back(floor);
}

void Viewer::draw()
{
    if(m_drawMeshes)
    {
        for(int i=0;i<m_bodies.size();++i)
            m_bodies[i]->draw();
    }

    glDisable(GL_LIGHTING);
    if(m_drawSimulationPoints)
    {
        glPointSize(10);

        glColor3f(1.0f,0.0,0.0f);
        glBegin(GL_POINTS);

        glColor3f(0,0,1);
        for (unsigned i = 0; i < m_simulator->m_projected.size(); i++)
        {
            Eigen::Vector3f p = m_simulator->m_projected[i];
            glVertex3f(p[0], p[1], p[2]);
        }

        glColor3f(1,0,1);
        for (unsigned i = 0; i < m_simulator->m_projectedCollisions.size(); i++)
        {
            Eigen::Vector3f p = m_simulator->m_projectedCollisions[i];
            glVertex3f(p[0], p[1], p[2]);
        }

        glColor3f(0,1,0);
        for (unsigned i = 0; i < m_simulator->m_sn[0].rows(); i++)
        {
            glVertex3f(m_simulator->m_sn[0][i], m_simulator->m_sn[1][i], m_simulator->m_sn[2][i]);
        }

        glColor3f(1,0,0);
        vector<Simulator::Collision> collisions = m_simulator->getCollisions();
        for (unsigned i = 0; i < collisions.size(); i++)
        {
            LA::Vector3 p = collisions[i].info->p;
            glVertex3f(p[0], p[1], p[2]);
        }


        glEnd();



        glBegin(GL_LINES);
        glColor3f(1,0,1);
        for (unsigned i = 0; i < m_simulator->m_constraints.size(); i++)
        {
            ProjectiveConstraint* constraint = m_simulator->m_constraints[i];
            int bodyIndex = m_simulator->m_bodyToIndex[constraint->m_body];
            for(int j=0; j<constraint->m_numParticles; ++j)
            {
                for(int k=j+1; k<constraint->m_numParticles; ++k)
                {

                    glVertex3f(m_simulator->m_q[0][bodyIndex+constraint->getVIndex(j)],
                            m_simulator->m_q[1][bodyIndex+constraint->getVIndex(j)],
                            m_simulator->m_q[2][bodyIndex+constraint->getVIndex(j)]);
                    glVertex3f(m_simulator->m_q[0][bodyIndex+constraint->getVIndex(k)],
                            m_simulator->m_q[1][bodyIndex+constraint->getVIndex(k)],
                            m_simulator->m_q[2][bodyIndex+constraint->getVIndex(k)]);
                }
            }
        }
        glEnd();

    }
    glEnable(GL_LIGHTING);

    // Draws rectangular selection area. Could be done in postDraw() instead.
//    if (m_selectionMode != NONE)
//        drawSelectionRectangle();

    if(m_saveVideo && (m_currentTime - m_timeOfLastImage >= 1.0/m_videoFps || !m_videoWriter.isOpened()))
    {
        QImage img = this->grab().toImage();
        cv::Mat mat = qImageToCVMat(img);

        if(!m_videoWriter.isOpened())
        {
            std::cout << "Writting video" << std::endl;
            m_videoWriter.open(m_videoName.c_str(),CV_FOURCC('M','J','P','G'), m_videoFps, cv::Size(mat.cols,mat.rows),true);
        }
        m_videoWriter.write(mat);
        m_timeOfLastImage = m_currentTime;
    }
}

void Viewer::animate()
{
    if(m_play || m_step)
    {
        m_simulator->advanceTime();
        m_currentTime += m_simulator->m_dt;
    }

    m_step &= false;
}

QString Viewer::helpString() const {
    QString text("<h2>A n i m a t i o n</h2>");
    text += "Use the <i>animate()</i> function to implement the animation part "
            "of your ";
    text += "application. Once the animation is started, <i>animate()</i> and "
            "<i>draw()</i> ";
    text += "are called in an infinite loop, at a frequency that can be "
            "fixed.<br><br>";
    text += "Press <b>Return</b> to start/stop the animation.";
    return text;
}

void Viewer::keyPressEvent(QKeyEvent *key)
{
    if(key->text() == "o")
    {
        m_step = true;
    }
    else if(key->text() == "p")
    {
        m_play = !m_play;
    }
    else if(key->text() == "d")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(0.03, 0, 0);
//        }
    }
    else if(key->text() == "a")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(-0.03, 0, 0);
//        }
    }
    else if(key->text() == "s")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(0, -0.03, 0);
//        }
    }
    else if(key->text() == "w")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(0, 0.03, 0);
//        }
    }
    else if(key->text() == "r")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(0, 0, 0.03);
//        }
    }
    else if(key->text() == "f")
    {
//        for(int i=0;i<m_hardConstraintsPositions.size();++i)
//        {
//            m_hardConstraintsPositions[i] += Eigen::Vector3f(0, 0, -0.03);
//        }
    }
//    else if(key->text() == "m")
//    {
//        m_move = !m_move;
//        if(m_move)
//        {
//            QList<int>::iterator it = m_selection.begin();
//            std::vector<TriVertex*> verts = m_mesh->Vertices();

//            m_hardConstraintsIndices.clear();
//            m_hardConstraintsPositions.clear();
//            for(;it!=m_selection.end();++it)
//            {
//                TriVertex* v = verts[*it];
//                Vector3 pos = v->Position();

//                m_hardConstraintsIndices.push_back(v->Id());
//                m_hardConstraintsPositions.push_back(toEigenVector3(pos));
//            }
//            m_simulator->initialize(m_dt, m_hardConstraintsIndices, m_hardConstraintsPositions, m_springConstraints, m_tetraConstraints);
//        }
//    }
    else if(key->text() == "v")
    {
        if(m_saveVideo)
        {
            m_saveVideo = false;
            m_videoWriter.release();
        }
        else
            m_saveVideo = true;
    }
    else
        QGLViewer::keyPressEvent(key);
}

//   C u s t o m i z e d   m o u s e   e v e n t s

void Viewer::mousePressEvent(QMouseEvent *e) {
    // Start selection. Mode is ADD with Shift key and TOGGLE with Alt key.
    m_rectangle = QRect(e->pos(), e->pos());

    if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ShiftModifier))
        m_selectionMode = ADD;
    else if ((e->button() == Qt::LeftButton) &&
             (e->modifiers() == Qt::ControlModifier))
        m_selectionMode = REMOVE;
    else if(m_move)
    {
        m_initialPoint = e->pos();
    }
    else {
        //    if (e->modifiers() == Qt::ControlModifier)
        //      startManipulation();

        QGLViewer::mousePressEvent(e);
    }
}

void Viewer::mouseMoveEvent(QMouseEvent *e) {
    if (m_selectionMode != NONE) {
        // Updates m_rectangle coordinates and redraws rectangle
        m_rectangle.setBottomRight(e->pos());
        update();
    }
//    else if(m_move)
//    {
//        QPoint currentPos = e->pos();
//        QPoint displacement = (currentPos - m_initialPoint);
//        qglviewer::Vec up = camera()->upVector();
//        qglviewer::Vec right = camera()->rightVector();
//        int width = camera()->screenWidth();
//        int height = camera()->screenHeight();
//        float realX = (float) displacement.x() / width;
//        float realY = (float) displacement.y() / height;
//        qglviewer::Vec v = up*realY*-1.0 + right*realX;

//        Vector3 u(v[0], v[1], v[2]);
//        QList<int>::iterator it = m_selection.begin();
//        std::vector<TriVertex*> verts = m_mesh->Vertices();

//        for(int i=0;it!=m_selection.end();++it, ++i)
//        {
//            TriVertex* v = verts[*it];
//            Vector3 pos = v->PositionInit();

//            qglviewer::Vec posSP = camera()->projectedCoordinatesOf(qglviewer::Vec(pos));
//            posSP.x += displacement.x();
//            posSP.y += displacement.y();

//            qglviewer::Vec disp = camera()->unprojectedCoordinatesOf(posSP);
//            Vector3 p(disp.x, disp.y, disp.z) ;
//            m_hardConstraintsPositions[i] = toEigenVector3(p);
//        }

//        update();

//    }
    else
    {
        QGLViewer::mouseMoveEvent(e);
    }
}

void Viewer::mouseReleaseEvent(QMouseEvent *e)
{
    if (m_selectionMode != NONE) {
//        bool somethingSelected = false;

//        // Actual selection on the rectangular area.
//        std::vector<TriVertex*> verts = m_mesh->Vertices();
//        for(unsigned i=0;i<verts.size();++i)
//        {
//            TriVertex* v = verts[i];
//            Vector3 p = v->Position();
//            qglviewer::Vec src;
//            src[0] = p[0];
//            src[1] = p[1];
//            src[2] = p[2];

//            qglviewer::Vec w = camera()->projectedCoordinatesOf(src);
//            if (m_rectangle.contains(w.x,w.y))
//            {
//                if(m_selectionMode == ADD)
//                    addIdToSelection(i);
//                else if (m_selectionMode == REMOVE)
//                    removeIdFromSelection(i);
//                somethingSelected = true;
//            }
//        }
//        glEnd();

//        if(!somethingSelected)
//            m_selection.clear();

//        m_selectionMode = NONE;
//        // Update display to show new selected objects
//        update();
    }
    else
        QGLViewer::mouseReleaseEvent(e);
}

//   S e l e c t i o n   t o o l s
void Viewer::addIdToSelection(int id)
{
    if (!m_selection.contains(id))
    {
        m_selection.push_back(id);
    }
}

void Viewer::removeIdFromSelection(int id)
{
    m_selection.removeAll(id);
}


void Viewer::drawSelectionRectangle() const
{
    startScreenCoordinatesSystem();
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);

    glColor4f(0.0, 0.0, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(m_rectangle.left(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.bottom());
    glVertex2i(m_rectangle.left(), m_rectangle.bottom());
    glEnd();

    glLineWidth(2.0);
    glColor4f(0.4f, 0.4f, 0.5f, 0.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(m_rectangle.left(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.top());
    glVertex2i(m_rectangle.right(), m_rectangle.bottom());
    glVertex2i(m_rectangle.left(), m_rectangle.bottom());
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    stopScreenCoordinatesSystem();
}


Viewer::Viewer(QWidget *parent) : QGLViewer(parent)
{
    m_selectionMode = NONE;
    m_move = false;

    m_drawMeshes = true;
    m_drawSimulationPoints = false;
    m_meshRows = 10;
    m_meshColumns = 10;
    m_meshSize = 1;

    m_dt = 0.01;
    m_totalMass = 1.0;
    m_iterations = 1;

    m_springStiffness = 10000;
    m_tetraStiffness = 10000;
    m_collisionStiffness = 100000000;
    m_positionStiffness = 100000000;

    m_simulator = 0;

    m_saveVideo = false;
}

void Viewer::reset()
{
    m_selectionMode = NONE;
    m_move = false;

    delete(m_simulator);

    init();
}

void Viewer::changeDrawMeshes(int val)
{
    m_drawMeshes = val;
}

void Viewer::changeDrawSimulationPoints(int val)
{
    m_drawSimulationPoints = val;
}

void Viewer::changeMeshRows(int val)
{
    m_meshRows = val;
}

void Viewer::changeMeshCols(int val)
{
    m_meshColumns = val;
}

void Viewer::changeSize(double size)
{
    m_meshSize = size;
}

void Viewer::changeParticleMass(double val)
{
    m_totalMass = val;
}

void Viewer::changeTimestep(double val)
{
    m_dt = val;
}

void Viewer::changeIterations(int val)
{
    m_iterations = val;
}

void Viewer::changeSpringStiffness(double val)
{
    m_springStiffness = val;
}

void Viewer::changeTetraStiffness(double val)
{
    m_tetraStiffness = val;
}

void Viewer::changePositionStiffness(double val)
{
    m_positionStiffness = val;
}

void Viewer::changeCollisionStiffness(double val)
{
    m_collisionStiffness = val;
    if(m_simulator)
        m_simulator->m_collisionStiffness = val;
}

void Viewer::makeVideo()
{
    if(m_saveVideo)
    {
        m_saveVideo = false;
        m_videoWriter.release();
        emit(makingVideo(false));
    }
    else
    {
        m_videoName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                   QString(PROJECT_FOLDER),
                                                   tr("Videos (*.avi)")).toStdString();
        if(m_videoName.length() > 0)
        {
            m_saveVideo = true;
            m_videoFps = 25;
            m_timeOfLastImage = 0;
            emit(makingVideo(true));
        }
    }

}


