#include "generic_error_dialog.h"
#include "ui_generic_error_dialog_form.h"

#define OBJ_FUNCTION_CALL
#ifdef OBJ_FUNCTION_CALL
#define FUNCTION_CALL(_obj, args...) _obj->setupUi(args)
#else
#define FUNCTION_CALL(_obj)
#endif

generic_error_dialog::generic_error_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    FUNCTION_CALL(ui, this);
}

generic_error_dialog::generic_error_dialog(const QString &header, const QString &footer, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::generic_error_dialog_form)
{
    ui->setupUi(this);
    ui->header->setPlainText(header);
    ui->footer->setPlainText(footer);
    this->exec();
}

void generic_error_dialog::setHeaderText(const QString &headerText)
{
    ui->header->setPlainText(headerText);
}

void generic_error_dialog::setBodyText(const QString &bodyText)
{
    ui->footer->setPlainText(bodyText);
}

generic_error_dialog::~generic_error_dialog()
{
    delete ui;
}
