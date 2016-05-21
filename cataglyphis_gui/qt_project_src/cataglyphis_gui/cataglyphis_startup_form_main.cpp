#include "cataglyphis_startup_form_main.h"
#include "ui_cataglyphis_startup_form_main.h"

cataglyphis_startup_form_main::cataglyphis_startup_form_main(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::cataglyphis_startup_form_main)
{
    ui->setupUi(this);
}

cataglyphis_startup_form_main::~cataglyphis_startup_form_main()
{
    delete ui;
}
