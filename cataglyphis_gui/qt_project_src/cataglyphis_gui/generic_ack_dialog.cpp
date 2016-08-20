#include "generic_ack_dialog.h"
#include "ui_generic_ack_dialog_form.h"

generic_ack_dialog::generic_ack_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_ack_dialog)
{
    ui->setupUi(this);
}

generic_ack_dialog::generic_ack_dialog(const char * message, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_ack_dialog)
{
    ui->setupUi(this);
    ui->messageBox->setPlainText(message);
}

generic_ack_dialog::~generic_ack_dialog()
{
    delete ui;
}
