#include <QApplication>
#include "cataglyphis_gui.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //ros::init()

    cataglyphis_gui cata_gui;
    cata_gui.show();

    return a.exec();
}
