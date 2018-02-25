#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

#include <Eigen/Core>

#include <TriMesh.h>
#include <TetraMesh.h>

#include "simulator.h"

class Viewer : public QGLViewer {
    Q_OBJECT

    void testSceneClothOnBunny();
    void testSceneClothConstrainedTopCorners();
    void testSceneClothConstrainedCorners();
    void testSceneClothDropping();
//    void testSceneSingleTetra();
//    void testSceneDeformableSphere();

protected:
    virtual void draw();
    virtual void init();
    virtual void animate();
    virtual QString helpString() const;

    void keyPressEvent(QKeyEvent *key);

    // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);

    void drawSelectionRectangle() const;
    void addIdToSelection(int id);
    void removeIdFromSelection(int id);

public:
    Viewer(QWidget *parent);

    int m_meshRows, m_meshColumns;
    double m_meshSize;
    float m_dt;
    double m_totalMass;
    int m_iterations;
    double m_springStiffness;
    double m_tetraStiffness;
    double m_collisionStiffness;
    double m_positionStiffness;

private:

    vector<ProjectiveBody*> m_bodies;


    Simulator *m_simulator;

    bool m_play;
    bool m_step;


    enum SelectionMode { NONE, ADD, REMOVE };
    SelectionMode m_selectionMode;

    // Current rectangular selection
    QRect m_rectangle;

    QList<int> m_selection;
//    QList<int> m_fixed;

    bool m_move;
    QPoint m_initialPoint;

public Q_SLOTS:
    void reset();

    void changeMeshRows(int val);
    void changeMeshCols(int val);
    void changeSize(double size);
    void changeParticleMass(double val);
    void changeTimestep(double val);
    void changeIterations(int val);
    void changeSpringStiffness(double val);
    void changeTetraStiffness(double val);
    void changePositionStiffness(double val);
    void changeCollisionStiffness(double val);



};

#endif // VIEWER_H
