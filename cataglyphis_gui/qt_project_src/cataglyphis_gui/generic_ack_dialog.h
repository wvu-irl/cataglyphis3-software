#ifndef GENERIC_ACK_DIALOG_H
#define GENERIC_ACK_DIALOG_H

#include <QDialog>

namespace Ui {
class generic_ack_dialog;
}

class generic_ack_dialog : public QDialog
{
    Q_OBJECT

public:
    explicit generic_ack_dialog(QWidget *parent = 0);
    explicit generic_ack_dialog(const char * message, QWidget *parent = 0);
    ~generic_ack_dialog();

    int bringUpDialogModal()
    {
        this->raise();
        this->activateWindow();
        return this->exec();
    }

private:
    Ui::generic_ack_dialog *ui;
};

#endif // GENERIC_ACK_DIALOG_H
