#include "cataglyphis_gui.h"
#include "ui_cataglyphis_gui.h"

cataglyphis_gui::cataglyphis_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::cataglyphis_gui)
{
    ui->setupUi(this);
    cataglyphis_gui::cataglyphis_startup_form = new cataglyphis_startup_form_main;
    cataglyphis_gui::map_view_form = new map_viewer(0);
    ui->guiTabber->addTab(cataglyphis_gui::map_view_form, "Map");
    ui->guiTabber->addTab(cataglyphis_gui::cataglyphis_startup_form, "Startup");

}

cataglyphis_gui::~cataglyphis_gui()
{
    delete ui;
}


