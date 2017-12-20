/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
#include <QVector3D>

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
#include <messages/SLAMPoseOut.h>

#include <robot_control/RegionsOfInterest.h>
#include <robot_control/ROI.h>

#include <ros/ros.h>
#include <ros_workers.h>

#define MAP_CELL_MAX_VALUE 100.0
#define MAP_CELL_MIN_VALUE 0.0
#define MAP_CELL_NOOP_VALUE -5.0

#define SLAM_PATH_Z_VAL 102.9
#define NAV_PATH_Z_VAL 103.0

const double pi = std::acos(-1);
#define DEG_2_RAD (pi/180.0)
#define RAD_2_DEG (180.0/pi)

class QGraphicsSceneMapViewer : public QGraphicsScene
{
    Q_OBJECT

signals:
    void request_global_map(map_viewer_enums::mapViewerLayers_t requestedLayer);
    void map_viewer_scene_init(bool reInit);
    void start_nav_info_sub();
    void start_slam_info_sub();
    void request_roi();
    void confirm_roi_changes();
    void discard_roi_changes();

public slots:
    void on_set_ignore_setup_flag(bool status);
    void on_map_manager_gridmap_service_returned(messages::GlobalMapFull gridMapFull, map_viewer_enums::mapViewerLayers_t requestedLayer, bool wasSucessful);
    void on_set_layer_visibility(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility);
    void on_hsm_global_pose_callback(const messages::RobotPose navInfo);
    void on_map_manager_roi_service_returned(const robot_control::RegionsOfInterest mapManagerResponse, bool wasSucessful);
    void on_nav_info_callback(const messages::NavFilterOut navInfo);
    void on_slam_info_callback(const messages::SLAMPoseOut slamInfo);

    void on_center_on_cataglyphis(bool status);
    void on_set_cataglyphis_path_length(int length);
    void on_set_cataglyphis_path_step_size(double stepSize);

    void on_confirm_map_changes();
    void on_discard_map_changes();

    void on_refresh_active_layers();

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
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>(),
                            bool hasPredefinedStartingPos = false):
        QGraphicsScene(parent),
        areaImage(new QImage(imagePath))
    {
        worker = workerArg;
        pixelsPerDistance = pixelsPerDist;
        ignoreSetup = ignoreSetupStep;
        mapSetup = false;
        hasPredefinedStartingPositions = hasPredefinedStartingPos;
        areaImagePixmap = this->addPixmap(QPixmap::fromImage(*areaImage));
        _implInitPointers();
    }
    //this constructor takes ownership of the pointer
    QGraphicsSceneMapViewer(QImage *image, float pixelsPerDist, bool ignoreSetupStep = false, QObject * parent = 0,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>(),
                            bool hasPredefinedStartingPos = false):
        QGraphicsScene(parent)
    {
        worker = workerArg;
        pixelsPerDistance = pixelsPerDist;
        ignoreSetup = ignoreSetupStep;
        mapSetup = false;
        *areaImage = *image;
        areaImagePixmap = this->addPixmap(QPixmap::fromImage(*areaImage));
        hasPredefinedStartingPositions = hasPredefinedStartingPos;
        _implInitPointers();
    }

    void mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent);

    struct layerProperties_t { bool isLayerSetup = false, isLayerVisible = false; };
    struct mapLayer_t { layerProperties_t properties; QGraphicsItemGroup *items; boost::scoped_ptr<QList<QGraphicsItem*> > itemList;
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

    QVector3D lastNavPointPlotted;
    QVector3D lastSlamPointPlotted;

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

    bool hasPredefinedStartingPositions;

private:
    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    bool ignoreSetup;
    bool mapSetup;
    bool requestedMap;
    float pixelsPerDistance;
    bool centerOnCataglyphis;
    int cataglyphisPathLength;
    double cataglyphisPathStepSize;

    boost::shared_ptr<ros_workers> worker;

    QTransform robotToObjTransform;
    messages::RobotPose lastRobotPose;
    boost::scoped_ptr<QImage> areaImage;
    QGraphicsPixmapItem *areaImagePixmap;

    QGraphicsRectItem *startingPlatform;
    QGraphicsEllipseItem *cataglyphis;
    QGraphicsItemGroup *cataglyphisGroup;


    grid_map::GridMap gridMapContainer;

    void _implSetupLayer(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility, bool reInit = false);
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
        cataglyphisGroup = 0;
        requestedMap = false;
        centerOnCataglyphis = false;
        cataglyphisPathLength=0;
        cataglyphisPathStepSize=0;
        lastNavPointPlotted.setX(0);
        lastNavPointPlotted.setY(0);
        lastNavPointPlotted.setZ(0);
        lastSlamPointPlotted.setX(0);
        lastSlamPointPlotted.setY(0);
        lastSlamPointPlotted.setZ(0);
    }

};

#endif // QGRAPHICSSCENEMAPVIEWER_H
