#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <QWidget>

namespace Ui {
class manual_control_form;
}

class manual_control : public QWidget
{
    Q_OBJECT

public slots:
    void on_manual_override(bool overrideEnabled);

public:
    explicit manual_control(QWidget *parent = 0);
    ~manual_control();

private:
    Ui::manual_control_form *ui;

    bool keysEnabled;
};

#endif // MANUAL_CONTROL_H
