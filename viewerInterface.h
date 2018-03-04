#ifndef VIEWERINTERFACE_H
#define VIEWERINTERFACE_H

#include "ui_viewerInterface.h"

class ViewerInterface : public QDialog, public Ui::Dialog
{
    Q_OBJECT

public Q_SLOTS:
    void changeVideoButtionText(bool active);

public:
    ViewerInterface();

    void updateVariables();

};

#endif // VIEWERINTERFACE_H
