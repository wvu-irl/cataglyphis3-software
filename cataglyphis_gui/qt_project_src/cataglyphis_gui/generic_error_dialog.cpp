#include "generic_error_dialog.h"
#include "ui_generic_error_dialog_form.h"

generic_error_dialog::generic_error_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    ui->setupUi(this);
}

generic_error_dialog::~generic_error_dialog()
{
    delete ui;
}
