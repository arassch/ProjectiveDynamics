#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

#include <Eigen/Core>

#include <TriMesh.h>
#include "simulator.h"

class Viewer : public QGLViewer {
    Q_OBJECT
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



private:
    int m_meshRows, m_meshColumns;
    double m_meshCellSize;
    TriMesh *m_mesh;
    vector<TriMesh*> m_rigidBodies;
    float m_dt;
    double m_particleMass;
    int m_iterations;
    double m_springStiffness;
    double m_collisionStiffness;

    Simulator *m_simulator;

    vector<pair<int, int> > m_springConstraints;
    vector<int> m_hardConstraintsIndices;
    vector<Eigen::Vector3f> m_hardConstraintsPositions;

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
    void changeCellSize(double size);
    void changeParticleMass(double val);
    void changeTimestep(double val);
    void changeIterations(int val);
    void changeSpringStiffness(double val);
    void changeCollisionStiffness(double val);


};

#endif // VIEWER_H
