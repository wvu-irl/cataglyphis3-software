#include "cataglyphis_gui.h"
#include "ui_cataglyphis_gui.h"
#include <QGraphicsPixmapItem>
#include <stdio.h>

QGraphicsScene* scene;
QGraphicsView* view;
QGraphicsPixmapItem* item;
QImage* field_pic;

Cataglyphis_Gui::Cataglyphis_Gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Cataglyphis_Gui)
{
    ui->setupUi(this);
}

Cataglyphis_Gui::~Cataglyphis_Gui()
{
    delete ui;
}

void Cataglyphis_Gui::on_fieldSelector_currentIndexChanged(int index)
{
    printf("Index Changed %d\n", index);
    field_pic = new QImage(":/field_pictures/resources/institute_park.jpg");
    scene = new QGraphicsScene();
    view = new QGraphicsView(scene);
    item = new QGraphicsPixmapItem(QPixmap::fromImage(*field_pic));
    ui->fieldDisplay->setScene(scene);
    scene->addItem(item);
    ui->fieldDisplay->show();
}
