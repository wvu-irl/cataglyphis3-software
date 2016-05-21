#ifndef CATAGLYPHIS_STARTUP_FORM_MAIN_H
#define CATAGLYPHIS_STARTUP_FORM_MAIN_H

#include <QWidget>

namespace Ui {
class cataglyphis_startup_form_main;
}

class cataglyphis_startup_form_main : public QWidget
{
    Q_OBJECT

public:
    explicit cataglyphis_startup_form_main(QWidget *parent = 0);
    ~cataglyphis_startup_form_main();

private:
    Ui::cataglyphis_startup_form_main *ui;
};

#endif // CATAGLYPHIS_STARTUP_FORM_MAIN_H
