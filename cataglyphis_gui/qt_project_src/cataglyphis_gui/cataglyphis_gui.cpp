#include "cataglyphis_gui.h"
#include "ui_cataglyphis_gui.h"
#include <stdio.h>


Cataglyphis_Gui::Cataglyphis_Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Cataglyphis_Gui)
{
    scene = new QGraphicsScene();
    //field_pic = new QImage[4];
    field_pic[0] = new QImage(":/field_pictures/resources/pattern_test.jpg");
    field_pic[1] = new QImage(":/field_pictures/resources/institute_park.jpg");
    ui->setupUi(this);
}

Cataglyphis_Gui::~Cataglyphis_Gui()
{
    //printf("%p, %p, %p, %p\n", scene, view, item, field_pic);
    delete ui;
    //printf("%p, %p, %p, %p\n", scene, view, item, field_pic);
}

void Cataglyphis_Gui::on_fieldSelector_currentIndexChanged(int index)
{
    printf("Index Changed %d\n", index);

    scene->clear();
    item = new QGraphicsPixmapItem(QPixmap::fromImage(*field_pic[index]));

    //need to load new info about the map to display the roer on it.
    //where coordinate 0,0 is
    //the distance each pixel represents on the map
    ui->fieldDisplay->setScene(scene);
    scene->addItem(item);
    cataglyphisRect = scene->addRect(QRectF(0,0,50,50));
    ui->fieldDisplay->show();
}
