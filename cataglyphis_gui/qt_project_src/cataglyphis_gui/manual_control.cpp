#include "manual_control.h"
#include "ui_manual_control_form.h"

manual_control::manual_control(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::manual_control_form)
{
    ui->setupUi(this);
    keysEnabled = false;
}

manual_control::~manual_control()
{
    delete ui;
}

void manual_control::on_manual_override(bool overrideEnabled)
{
    keysEnabled = overrideEnabled;
}
