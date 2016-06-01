#include "window_selector.h"
#include "ui_window_selector.h"

window_selector::window_selector(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::window_selector)
{
    ui->setupUi(this);
}

window_selector::~window_selector()
{
    delete ui;
}
