#include "map_viewer.h"
#include "ui_map_viewer.h"

map_viewer::map_viewer(QWidget *parent, int startIndex) :
    QWidget(parent),
    ui(new Ui::map_viewer)
{
    scene = boost::shared_ptr<QGraphicsScene>(new QGraphicsScene());
    field_pic = boost::shared_array<boost::shared_ptr<QImage> >(new boost::shared_ptr<QImage>[4]);
    field_pic[0] = boost::shared_ptr<QImage>(new QImage(":/field_pictures/resources/pattern_test.jpg"));
    field_pic[1] = boost::shared_ptr<QImage>(new QImage(":/field_pictures/resources/institute_park.jpg"));
    ui->setupUi(this);
    item = boost::shared_ptr<QGraphicsPixmapItem>(new QGraphicsPixmapItem(QPixmap::fromImage(*field_pic[startIndex])));
    scene->addItem(item.get());
    ui->fieldDisplay->setScene(scene.get());
    ui->fieldDisplay->show();
}

map_viewer::~map_viewer()
{
    //ui.reset();
}

void map_viewer::on_fieldSelector_currentIndexChanged(int index)
{
    printf("Index Changed %d\n", index);

    //scene->clear();
    item = boost::shared_ptr<QGraphicsPixmapItem>(new QGraphicsPixmapItem(QPixmap::fromImage(*field_pic[index])));

    //need to load new info about the map to display the roer on it.
    //where coordinate 0,0 is
    //the distance each pixel represents on the map

    scene->addItem(item.get());
    cataglyphisRect = boost::shared_ptr<QGraphicsRectItem>(scene->addRect(QRectF(0,0,50,50)));
    ui->fieldDisplay->setScene(scene.get());
    ui->fieldDisplay->show();
}
