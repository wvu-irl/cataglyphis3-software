#include "map_viewer.h"
#include "ui_map_viewer_form.h"

map_viewer::map_viewer(QWidget *parent, int startIndex, boost::shared_ptr<ros::NodeHandle> nhArg) :
    QWidget(parent),
    ui(new Ui::map_viewer_form),
    defaultCircleFill(255,0,255,100),
    fullTransparentColor(0,0,0,0),
    scene(boost::shared_array<boost::scoped_ptr<QGraphicsSceneMapViewer> >(new boost::scoped_ptr<QGraphicsSceneMapViewer>[4]))
{
    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers(nhArg));
    rosWorker->moveToThread(&rosWorkerThread);
    rosWorkerThread.start();

    ui->setupUi(this);

    QPixmapCache::setCacheLimit(102400);

    ui->fieldDisplay->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    scene[0].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/pattern_test.jpg",   0,          true, this, rosWorker));
    scene[1].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/institute_park.jpg", 3.584058,   true, this, rosWorker));
    scene[2].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/esb.jpg",            4.52580645, true, this, rosWorker));
    scene[3].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/chestnut_ridge.jpg", 5.096154,   true, this, rosWorker));

    on_fieldSelector_currentIndexChanged(startIndex);
}

map_viewer::~map_viewer()
{
    std::printf("Map_Viewer Destruction\r\n");
    rosWorkerThread.quit();
    rosWorkerThread.wait();
    delete ui;
    //ui.reset();
}

void map_viewer::on_fieldSelector_currentIndexChanged(int index)
{
    printf("Index Changed %d\n", index);

    if(ui->fieldDisplay->scene())
    {
        ((QGraphicsSceneMapViewer*)ui->fieldDisplay->scene())->disconnectSignals();
    }

    disconnect(this, &map_viewer::set_map_layer_visibility,0,0);
    disconnect(this, &map_viewer::set_scene_setup_alert,0,0);

    connect(this, &map_viewer::set_map_layer_visibility,
                scene[index].get(), &QGraphicsSceneMapViewer::on_set_layer_visibility);
    connect(this, &map_viewer::set_scene_setup_alert,
                scene[index].get(), &QGraphicsSceneMapViewer::on_set_ignore_setup_flag);

    scene[index]->connectSignals();

    ui->keyframeMapLayerButton->setChecked(scene[index]->keyframeLayer.properties.isLayerVisible);
    ui->roiMapLayerButton->setChecked(scene[index]->roiLayer.properties.isLayerVisible);
    ui->pathLayerButton->setChecked(scene[index]->pathLayer.properties.isLayerVisible);
    ui->gridMapLayerButton->setChecked(scene[index]->gridMapLayer.properties.isLayerVisible);

    ui->fieldDisplay->setScene(scene[index].get());
    ui->fieldDisplay->show();
    ROS_DEBUG("MAP:: is the view active windows? %d", (int)ui->fieldDisplay->isActiveWindow());
    ROS_DEBUG("MAP:: Is current map active %d", scene[index]->isActive());
}

//void map_viewer::on_drawTestShapesButton_clicked()
//{
//    ROS_WARN("MAP:: Test shape drawing removed");
////    QPen transparentPen(fullTransparentColor);
////    map_view_roi_ellipse *roiEllipse = new map_view_roi_ellipse(0,0,100,100,transparentPen,defaultCircleFill);
////    map_view_roi_ellipse *roiEllipse2 = new map_view_roi_ellipse(50,50,100,100,roiEllipse,transparentPen,defaultCircleFill);
////    QGraphicsTextItem *roi2text = new QGraphicsTextItem("2", roiEllipse);
////    roi2text->setPos(roi2text->parentItem()->boundingRect().width()/2-roi2text->boundingRect().width()/2,
////                        roi2text->parentItem()->boundingRect().height()/2-roi2text->boundingRect().height()/2);//this centers the text in the center of the parent
////    roiEllipse->setFlag(QGraphicsItem::ItemIsMovable);
////    roiEllipse2->setFlag(QGraphicsItem::ItemIsMovable);

////    scene[ui->fieldSelector->currentIndex()]->addItem(roiEllipse);
//}

void map_viewer::on_gridMapLayerButton_clicked(bool checked)
{
    ROS_DEBUG("Grid Map Layer Button Clicked");
    emit set_map_layer_visibility(map_viewer_enums::GRIDMAP, checked);
}

void map_viewer::on_roiMapLayerButton_clicked(bool checked)
{
    ROS_DEBUG("ROI Layer Button Clicked");
    emit set_map_layer_visibility(map_viewer_enums::ROI, checked);
}

void map_viewer::on_pathLayerButton_clicked(bool checked)
{
    ROS_DEBUG("Path Layer Button Clicked");
    emit set_map_layer_visibility(map_viewer_enums::PATH, checked);
}

void map_viewer::on_keyframeMapLayerButton_clicked(bool checked)
{
    ROS_DEBUG("Keyframe Layer Button Clicked");
    emit set_map_layer_visibility(map_viewer_enums::keyframeDrive, checked);
}

void map_viewer::on_initMapButton_clicked()
{
    generic_ack_dialog dialog("Click the starting point of the robot.");
    int returnVal = dialog.bringUpDialogModal();
    ROS_DEBUG("MAP:: init dialog return val %d: ", returnVal);
    ui->fieldDisplay->activateWindow();
    ROS_DEBUG("MAP:: is the view active windows? %d", (int)ui->fieldDisplay->isActiveWindow());
    if(returnVal == QDialog::Accepted)
    {
        ROS_DEBUG("MAP:: Dialog accepted, sending scene setup alert");
        ui->fieldDisplay->setCursor(Qt::CrossCursor);
        emit set_scene_setup_alert(false);
    }
}

void map_viewer::on_satDriveMapLayerButton_clicked(bool checked)
{
    ROS_DEBUG("MAP::satDrive Layer Button Clicked %d", (int)checked);
    emit set_map_layer_visibility(map_viewer_enums::SatDriveability, checked);
}

void map_viewer::on_refresh_layers_button_clicked()
{
    ROS_WARN("MAP:: Refreshing Not Implemented Yet");
}
