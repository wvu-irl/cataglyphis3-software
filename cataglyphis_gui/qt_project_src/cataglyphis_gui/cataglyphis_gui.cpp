#include "cataglyphis_gui.h"
#include "ui_cataglyphis_gui.h"

Cataglyphis_Gui::Cataglyphis_Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Cataglyphis_Gui)
{
    ui->setupUi(this);
}

Cataglyphis_Gui::~Cataglyphis_Gui()
{
    delete ui;
}
