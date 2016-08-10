#include "map_viewer.h"
#include "ui_map_viewer_form.h"

map_viewer::map_viewer(QWidget *parent, int startIndex) :
    QWidget(parent),
    ui(new Ui::map_viewer_form),
    defaultCircleFill(255,0,255,100),
    fullTransparentColor(0,0,0,0)
{
    ui->setupUi(this);

    scene = boost::shared_array<boost::shared_ptr<QGraphicsScene> >(new boost::shared_ptr<QGraphicsScene>[4]);
    scene[0] = boost::shared_ptr<QGraphicsScene>(new QGraphicsScene(this));
    scene[1] = boost::shared_ptr<QGraphicsScene>(new QGraphicsScene(this));
    scene[2] = boost::shared_ptr<QGraphicsScene>(new QGraphicsScene(this));
    scene[3] = boost::shared_ptr<QGraphicsScene>(new QGraphicsScene(this));
    fieldPictures = boost::shared_array<boost::shared_ptr<QImage> >(new boost::shared_ptr<QImage>[4]);
    fieldPictures[0] = boost::shared_ptr<QImage>(new QImage(":/field_pictures/resources/pattern_test.jpg"));
    fieldPictures[1] = boost::shared_ptr<QImage>(new QImage(":/field_pictures/resources/institute_park.jpg"));

    QGraphicsPixmapItem *item = new QGraphicsPixmapItem(QPixmap::fromImage(*fieldPictures[0]));
    scene[0]->addItem(item);
    item = new QGraphicsPixmapItem(QPixmap::fromImage(*fieldPictures[1]));
    scene[1]->addItem(item);

    ui->fieldDisplay->setScene(scene[startIndex].get());
    ui->fieldDisplay->show();
}

map_viewer::~map_viewer()
{
    delete ui;
    //ui.reset();
}

void map_viewer::on_fieldSelector_currentIndexChanged(int index)
{
    printf("Index Changed %d\n", index);

//    //scene->clear();
//    QGraphicsPixmapItem *item = new QGraphicsPixmapItem(QPixmap::fromImage(*fieldPictures[index]));

//    //need to load new info about the map to display the roer on it.
//    //where coordinate 0,0 is
//    //the distance each pixel represents on the map

//    scene->addItem(item.get());
//    cataglyphisRect = boost::shared_ptr<QGraphicsRectItem>(scene->addRect(QRectF(0,0,50,50)));
    ui->fieldDisplay->setScene(scene[index].get());
    ui->fieldDisplay->show();
}

void map_viewer::on_pushButton_clicked()
{
    QPen transparentPen(fullTransparentColor);
    map_view_roi_ellipse *roiEllipse = new map_view_roi_ellipse(0,0,100,100,transparentPen,defaultCircleFill);
    map_view_roi_ellipse *roiEllipse2 = new map_view_roi_ellipse(50,50,100,100,roiEllipse,transparentPen,defaultCircleFill);
    QGraphicsTextItem *roi2text = new QGraphicsTextItem("2", roiEllipse);
    roi2text->setPos(roi2text->parentItem()->boundingRect().width()/2-roi2text->boundingRect().width()/2,
                        roi2text->parentItem()->boundingRect().height()/2-roi2text->boundingRect().height()/2);//this centers the text in the center of the parent
    roiEllipse->setFlag(QGraphicsItem::ItemIsMovable);
    roiEllipse2->setFlag(QGraphicsItem::ItemIsMovable);

    scene[ui->fieldSelector->currentIndex()]->addItem(roiEllipse);
}

void map_viewer::draw_new_roi(/*put roi object info here*/)
{

}
