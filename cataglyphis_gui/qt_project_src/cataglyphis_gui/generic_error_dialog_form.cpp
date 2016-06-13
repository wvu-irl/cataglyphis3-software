#include "generic_error_dialog_form.h"
#include "ui_generic_error_dialog_form.h"

generic_error_dialog_form::generic_error_dialog_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    ui->setupUi(this);
}

generic_error_dialog_form::~generic_error_dialog_form()
{
    delete ui;
}
