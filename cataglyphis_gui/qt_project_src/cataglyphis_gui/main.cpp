#include "cataglyphis_gui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Cataglyphis_Gui w;
    w.show();

    return a.exec();
}
