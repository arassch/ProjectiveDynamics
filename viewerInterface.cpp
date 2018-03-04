#include "viewerInterface.h"


void ViewerInterface::changeVideoButtionText(bool active)
{
    if(!active)
        videoButton->setText("Make video");
    else
        videoButton->setText("Stop video");
}

ViewerInterface::ViewerInterface()
{
    setupUi(this);

    connect(viewer, SIGNAL(makingVideo(bool)), this, SLOT(changeVideoButtionText(bool)));
}

void ViewerInterface::updateVariables()
{
    drawMeshesCheck->setChecked(viewer->m_drawMeshes);
    drawSimulationPointsCheck->setChecked(viewer->m_drawSimulationPoints);
    meshRows->setValue(viewer->m_meshRows);
    meshColumns->setValue(viewer->m_meshColumns);
    meshSize->setValue(viewer->m_meshSize);
    timestep->setValue(viewer->m_dt);
    iterations->setValue(viewer->m_iterations);
    collisionStiffness->setValue(viewer->m_collisionStiffness);
    springStiffness->setValue(viewer->m_springStiffness);
    tetraStiffness->setValue(viewer->m_tetraStiffness);
    positionStiffness->setValue(viewer->m_positionStiffness);
}
