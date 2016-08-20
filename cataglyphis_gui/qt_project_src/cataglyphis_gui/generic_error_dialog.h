#ifndef GENERIC_ERROR_DIALOG_FORM_H
#define GENERIC_ERROR_DIALOG_FORM_H

#include <QString>
#include <QDialog>

namespace Ui {
class generic_error_dialog_form;
}

class generic_error_dialog : public QDialog
{
    Q_OBJECT

public:
    explicit generic_error_dialog(QWidget *parent = 0);
    generic_error_dialog(const QString &header, const QString &footer, QWidget *parent = 0);

    void setHeaderText(const QString &headerText);
    void setBodyText(const QString &bodyText);
    ~generic_error_dialog();

private:
    Ui::generic_error_dialog_form *ui;
};

#endif // GENERIC_ERROR_DIALOG_FORM_H
