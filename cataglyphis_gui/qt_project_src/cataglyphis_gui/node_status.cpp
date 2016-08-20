#include "node_status.h"
#include "ui_node_status_form.h"

node_status::node_status(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::node_status_form)
{
    ui->setupUi(this);
}

node_status::~node_status()
{
    delete ui;
}
