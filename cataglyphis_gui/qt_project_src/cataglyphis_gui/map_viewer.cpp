#include "map_viewer.h"
#include "ui_map_viewer_form.h"

map_viewer::map_viewer(QWidget *parent, int startIndex) :
    QWidget(parent),
    ui(new Ui::map_viewer_form),
    defaultCircleFill(255,0,255,100),
    fullTransparentColor(0,0,0,0)
{
    ui->setupUi(this);

    scene = boost::shared_array<boost::shared_ptr<QGraphicsSceneMapViewer> >(new boost::shared_ptr<QGraphicsSceneMapViewer>[4]);
    scene[0] = boost::shared_ptr<QGraphicsSceneMapViewer>(new QGraphicsSceneMapViewer(":/field_pictures/resources/pattern_test.jpg", this));
    scene[1] = boost::shared_ptr<QGraphicsSceneMapViewer>(new QGraphicsSceneMapViewer(":/field_pictures/resources/institute_park.jpg", this));
    scene[2] = boost::shared_ptr<QGraphicsSceneMapViewer>(new QGraphicsSceneMapViewer(":/field_pictures/resources/esb.jpg", this));
    scene[3] = boost::shared_ptr<QGraphicsSceneMapViewer>(new QGraphicsSceneMapViewer(this));

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
    ui->slamMapLayerButton->setChecked(false);
    ui->roiMapLayerButton->setChecked(false);
    ui->pathLayerButton->setChecked(false);
    ui->gridMapLayerButton->setChecked(false);
    ui->fieldDisplay->setScene(scene[index].get());
    ui->fieldDisplay->show();
}

void map_viewer::on_drawTestShapesButton_clicked()
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

void map_viewer::on_slamMapLayerButton_clicked(bool checked)
{

}

void map_viewer::on_roiMapLayerButton_clicked(bool checked)
{

}

void map_viewer::on_pathLayerButton_clicked(bool checked)
{

}

void map_viewer::on_gridMapLayerButton_clicked(bool checked)
{

}
