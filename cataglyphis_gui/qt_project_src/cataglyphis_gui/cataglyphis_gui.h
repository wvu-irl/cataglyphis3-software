#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>

namespace Ui {
class Cataglyphis_Gui;
}

class Cataglyphis_Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Cataglyphis_Gui(QWidget *parent = 0);
    ~Cataglyphis_Gui();

private:
    Ui::Cataglyphis_Gui *ui;
};

#endif // CATAGLYPHIS_GUI_H
