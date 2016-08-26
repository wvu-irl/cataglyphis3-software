#include "mission_planning.h"
#include "ui_mission_planning_form.h"

mission_planning::mission_planning(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::mission_planning_form)
{
    ui->setupUi(this);
}

mission_planning::~mission_planning()
{
    delete ui;
}
