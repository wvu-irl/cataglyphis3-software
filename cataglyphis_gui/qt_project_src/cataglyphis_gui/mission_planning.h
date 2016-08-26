#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H

#include <QWidget>
#include <messages/MissionPlanningInfo.h>
#include <messages/MissionPlanningControl.h>

namespace Ui {
class mission_planning_form;
}

class mission_planning : public QWidget
{
    Q_OBJECT

public:
    explicit mission_planning(QWidget *parent = 0);
    ~mission_planning();

private:
    Ui::mission_planning_form *ui;

    void _implMsgToUi();
    void _implUiToMsg();
    void _implCurrentDataSet();
    void _implSetReadOnly(bool readOnly);
};

#endif // MISSION_PLANNING_H
