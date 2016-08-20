#include "shift_map.h"
#include "ui_shift_map_form.h"

shift_map::shift_map(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::shift_map_form)
{
    ui->setupUi(this);
}

shift_map::~shift_map()
{
    delete ui;
}
