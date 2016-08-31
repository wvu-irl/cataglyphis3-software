#ifndef QGRAPHICSSCENEMAPVIEWER_H
#define QGRAPHICSSCENEMAPVIEWER_H

#include <QObject>
#include <QRectF>
#include <QImage>
#include <QPixmap>
#include <QPointF>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsItemGroup>
#include <QGraphicsItem>
#include <QList>
#include <QGraphicsPixmapItem>
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include <QTransform>
#include <QLinearGradient>
#include <QWidget>
#include <QPaintDevice>

#include <map_viewer_rect.h>

#include <generic_ack_dialog.h>

#include <map_viewer_enums.h>

#include <map_view_roi_ellipse.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <messages/RobotPose.h>
#include <messages/GlobalMapFull.h>

#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ROI.h>

#include <ros/ros.h>
#include <ros_workers.h>

#define MAP_CELL_MAX_VALUE 100.0
#define MAP_CELL_MIN_VALUE 0.0
#define MAP_CELL_NOOP_VALUE -5.0

class QGraphicsSceneMapViewer : public QGraphicsScene
{
    Q_OBJECT

signals:
    void request_global_map(map_viewer_enums::mapViewerLayers_t requestedLayer);
    void map_viewer_scene_init(bool reInit);
    void request_roi();
    void confirm_roi_changes();
    void discard_roi_changes();

public slots:
    void on_set_ignore_setup_flag(bool status);
    void on_map_manager_gridmap_service_returned(messages::GlobalMapFull gridMapFull, map_viewer_enums::mapViewerLayers_t requestedLayer, bool wasSucessful);
    void on_set_layer_visibility(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility);
    void on_hsm_global_pose_callback(const messages::RobotPose navInfo);
    void on_map_manager_roi_service_returned(const robot_control::RegionsOfInterest mapManagerResponse, bool wasSucessful);

    void on_confirm_map_changes();
    void on_discard_map_changes();

public:
    QGraphicsSceneMapViewer(QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>()):
        QGraphicsScene(parent) { ignoreSetup = true; mapSetup = false; worker = workerArg;}
    QGraphicsSceneMapViewer(const QRectF & sceneRect, bool ignoreSetupStep = false, QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>()):
        QGraphicsScene(sceneRect, parent) {ignoreSetup = ignoreSetupStep; mapSetup = false; worker = workerArg;}
    QGraphicsSceneMapViewer(qreal x, qreal y, qreal width, qreal height, bool ignoreSetupStep = false, QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>()):
        QGraphicsScene(x, y, width, height, parent) {ignoreSetup = ignoreSetupStep; mapSetup = false; worker = workerArg;}

    QGraphicsSceneMapViewer(const char *imagePath, float pixelsPerDist, bool ignoreSetupStep = false, QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>()):
        QGraphicsScene(parent),
        areaImage(new QImage(imagePath))
    {
        worker = workerArg;
        pixelsPerDistance = pixelsPerDist;
        ignoreSetup = ignoreSetupStep;
        mapSetup = false;
        areaImagePixmap = this->addPixmap(QPixmap::fromImage(*areaImage));
        _implInitPointers();
    }
    //this constructor takes ownership of the pointer
    QGraphicsSceneMapViewer(QImage *image, float pixelsPerDist, bool ignoreSetupStep = false, QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>()):
        QGraphicsScene(parent)
    {
        worker = workerArg;
        pixelsPerDistance = pixelsPerDist;
        ignoreSetup = ignoreSetupStep;
        mapSetup = false;
        *areaImage = *image;
        areaImagePixmap = this->addPixmap(QPixmap::fromImage(*areaImage));
        _implInitPointers();
    }

    void mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent);

    struct layerProperties_t { bool isLayerSetup = false, isLayerVisible = false; };
    struct mapLayer_t { layerProperties_t properties; boost::scoped_ptr<QGraphicsItemGroup> items; boost::scoped_ptr<QList<QGraphicsItem*> > itemList;
                            boost::scoped_ptr<QPixmap> gridPixmap; boost::scoped_ptr<QGraphicsPixmapItem> pixmapItem;};

    bool setupMap(QPointF scenePos);
    bool isMapSetup(){ return mapSetup; }
    bool drawRobot(QPointF position, qreal heading);
    const layerProperties_t & getLayerProperties(map_viewer_enums::mapViewerLayers_t mapLayer);
    mapLayer_t * getLayerFromEnum(const map_viewer_enums::mapViewerLayers_t &mapLayer);
    void setupLayer(map_viewer_enums::mapViewerLayers_t mapLayer);

    void redrawLayers();

    QPointF startPlatformCenter;

    mapLayer_t keyframeLayer;
    mapLayer_t roiLayer;
    mapLayer_t pathLayer;
    mapLayer_t gridMapLayer;
    mapLayer_t satDriveabilityLayer;

    void connectSignals(){ ROS_DEBUG("SCENE:: Connect signals"); _implConnectSignals(); }
    void disconnectSignals(){ ROS_DEBUG("SCENE:: Disconnect signals"); _implDisconnectSignals(); }

    void keyPressEvent(QKeyEvent * keyEvent)
    {
        if(keyEvent->key() == Qt::Key_Left)
        {
            for(int i = 0; i < roiLayer.itemList->size(); i++)
            {
                roiLayer.itemList->at(i)->setRotation(roiLayer.itemList->at(i)->rotation()+5);
            }
            keyEvent->accept();
            return;
        }
        QGraphicsScene::keyPressEvent(keyEvent);
    }

private:
    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    bool ignoreSetup;
    bool mapSetup;
    float pixelsPerDistance;

    boost::shared_ptr<ros_workers> worker;

    QTransform robotToObjTransform;
    messages::RobotPose lastRobotPose;
    boost::scoped_ptr<QImage> areaImage;
    QGraphicsPixmapItem *areaImagePixmap;

    QGraphicsRectItem *startingPlatform;
    QGraphicsEllipseItem *cataglyphis;


    grid_map::GridMap gridMapContainer;

    void _implSetupLayer(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility);
    void _implInitLayer(mapLayer_t *layer, bool reInit);
    void _implConnectSignals();
    void _implDisconnectSignals();

    bool _implSatelliteMapDisplay();
    bool _implHazardMapDisplay();
    bool _implGenericGridMapLayerDisplay();

    void _implInitPointers()
    {
        cataglyphis = 0;
        startingPlatform = 0;
    }

};

#endif // QGRAPHICSSCENEMAPVIEWER_H
