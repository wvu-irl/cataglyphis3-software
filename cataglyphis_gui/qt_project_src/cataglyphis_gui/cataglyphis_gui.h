#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include "cataglyphis_startup_form_main.h"
#include "map_viewer.h"

namespace Ui {
class cataglyphis_gui;
}

class cataglyphis_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit cataglyphis_gui(QWidget *parent = 0);
    ~cataglyphis_gui();

private slots:

private:
    Ui::cataglyphis_gui *ui;
    cataglyphis_startup_form_main *cataglyphis_startup_form;
    map_viewer *map_view_form;

};

#endif // CATAGLYPHIS_GUI_H
