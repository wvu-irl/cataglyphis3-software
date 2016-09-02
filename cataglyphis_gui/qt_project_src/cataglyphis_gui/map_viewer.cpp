#include "map_viewer.h"
#include "ui_map_viewer_form.h"

map_viewer::map_viewer(QWidget *parent, int startIndex) :
    QWidget(parent),
    ui(new Ui::map_viewer_form),
    defaultCircleFill(255,0,255,100),
    fullTransparentColor(0,0,0,0),
    scene(boost::shared_array<boost::scoped_ptr<QGraphicsSceneMapViewer> >(new boost::scoped_ptr<QGraphicsSceneMapViewer>[4]))
{
    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers());
    rosWorker->moveToThread(&rosWorkerThread);
    rosWorkerThread.start();

    connect(this, &map_viewer::run_global_map_request,
                rosWorker.get(), &ros_workers::on_run_map_manager_global_map_request);
    connect(rosWorker.get(), &ros_workers::map_manager_global_map_service_returned,
                this, &map_viewer::on_global_map_service_returned);

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j<3; j++)
        {
            startPlatformPositionsPixels[i][j] = QPointF(0,0);
        }
    }

    ui->setupUi(this);

    QPixmapCache::setCacheLimit(102400);

    ui->fieldDisplay->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    scene[0].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/pattern_test.jpg",   0,          true, this, rosWorker));
    scene[1].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/institute_park.jpg", 3.584058,   true, this, rosWorker, true));
    startPlatformPositionsPixels[1][0]+=QPointF(453,456);
    startPlatformPositionsPixels[1][1]+=QPointF(472,430);
    startPlatformPositionsPixels[1][2]+=QPointF(491,407);

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j<3; j++)
        {
            ROS_DEBUG("X: %d Y: %d", startPlatformPositionsPixels[i][j].x(), startPlatformPositionsPixels[i][j].y());
        }
    }

    scene[2].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/esb.jpg",            4.52580645, true, this, rosWorker));
    scene[3].reset(new QGraphicsSceneMapViewer(":/field_pictures/resources/wpi_practice.jpg",   4.65,       true, this, rosWorker));

    ui->fieldSelector->setCurrentIndex(startIndex);
    //on_fieldSelector_currentIndexChanged(startIndex);
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

    connect(this, &map_viewer::confirm_map_changes,
                scene[index].get(), &QGraphicsSceneMapViewer::on_confirm_map_changes);
    connect(this, &map_viewer::discard_map_changes,
                scene[index].get(), &QGraphicsSceneMapViewer::on_discard_map_changes);

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

void map_viewer::on_global_map_service_returned(messages::GlobalMapFull gridMapFull, map_viewer_enums::mapViewerLayers_t requestedLayer,
                                                bool wasSucessful)
{
    generic_ack_dialog dialog("Click the starting point of the robot.");

    if(!((QGraphicsSceneMapViewer*)ui->fieldDisplay->scene())->hasPredefinedStartingPositions || !wasSucessful)
    {
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
    else
    {
        ROS_DEBUG("Current Index %d", ui->fieldSelector->currentIndex());
        ROS_DEBUG("Starting Platform Num: %d", gridMapFull.response.startingPlatformNum);
        //ROS_DEBUG("X %d, Y %d", startPlatformPositionsPixels
        ((QGraphicsSceneMapViewer*)ui->fieldDisplay->scene())->setupMap(startPlatformPositionsPixels[ui->fieldSelector->currentIndex()][gridMapFull.response.startingPlatformNum-1]);
    }
}

void map_viewer::on_initMapButton_clicked()
{
    emit run_global_map_request(map_viewer_enums::PATH);
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

void map_viewer::on_commit_changes_button_clicked()
{
    emit confirm_map_changes();
}

void map_viewer::on_discard_changes_button_clicked()
{
    emit discard_map_changes();
}
