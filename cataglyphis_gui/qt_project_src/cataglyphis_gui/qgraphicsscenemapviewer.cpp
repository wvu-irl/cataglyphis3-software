#include "qgraphicsscenemapviewer.h"

void QGraphicsSceneMapViewer::on_map_manager_gridmap_service_returned(messages::GlobalMapFull gridMapFull,
                                                                          map_viewer_enums::mapViewerLayers_t requestedLayer,
                                                                            bool wasSucessful)
{
    ROS_DEBUG("SCENE:: gridmap service returned");
    if(wasSucessful)
    {
        ROS_DEBUG("SCENE:: grid map Service call was sucessful");
        mapLayer_t *layer = getLayerFromEnum(requestedLayer);
        grid_map::GridMapRosConverter::fromMessage(gridMapFull.response.globalMap, gridMapContainer);
        ROS_WARN("SCENE:: Grid MAP size %d, %d", gridMapContainer.getSize()[0], gridMapContainer.getSize()[1]);
        std::string gridMapLayerName = map_viewer_enums::gridMapLayersToString.find(map_viewer_enums::mapViewerLayersToGridMapLayers.find(requestedLayer)->second)->second;
        //if(gridMapContainer.isValid())
        {
            for(grid_map::GridMapIterator it(gridMapContainer); !it.isPastEnd(); it= (++it))
            {
                ROS_DEBUG_THROTTLE(5, "SCENE:: Iterator past end? %d %d", it.isPastEnd(), it.getLinearIndex());
                map_viewer_rect *gridRectangle = new map_viewer_rect();
                gridRectangle->setCacheMode(QGraphicsItem::ItemCoordinateCache);
                gridRectangle->defaultCircleFill.setRgb(255,0,255,100);
                gridRectangle->fullTransparentColor.setRgbF(0,0,0,0);
                gridRectangle->setBrush(gridRectangle->defaultCircleFill);
                gridRectangle->setTransform(robotToObjTransform);
                float mapValue = gridMapContainer.at(gridMapLayerName, *it);
                grid_map::Position cellPosition;
                gridMapContainer.getPosition(*it, cellPosition);
                //ROS_DEBUG("Current it %d, %d", cellPosition[0], cellPosition[1]);
                //ROS_DEBUG("SCENE:: grid MAP value %2.3f", mapValue);

                gridRectangle->setRect(cellPosition[0]*pixelsPerDistance, cellPosition[1]*pixelsPerDistance, pixelsPerDistance, pixelsPerDistance);
                this->addItem(gridRectangle);
                layer->itemList->append(gridRectangle);
                /*cellPosition[0] = x*/
                if(mapValue == 0.0)
                {
                    /*passable*/
                }
                else
                {
                    /*not passable*/
                }
            }
            ROS_DEBUG("SCENE:: Finished Reading Global Map");
//            ROS_DEBUG("SCENE:: adding item group");
//            layer->items = this->createItemGroup(*(layer->itemList));
//            ROS_DEBUG("SCENE:: adding group to scene");
//            this->addItem(layer->items);
//            ROS_DEBUG("SCENE:: showing layer");
//            //layer->items->show();
//            ros::Duration pause(60);
//            pause.sleep();

//            ROS_DEBUG("HIDING LAYER");
//            for(int i = 0; i< layer->itemList->size(); i++)
//            {
//                if((*(layer->itemList))[i])
//                {
//                    (*(layer->itemList))[i]->hide();
//                }
//            }
                ROS_DEBUG("DONE");

            layer->properties.isLayerSetup = true;
            layer->properties.isLayerVisible = true;
        }
//        else
//        {
//            ROS_ERROR("SCENE:: Grid MAP Container is invalid!");
//        }
    }
    else
    {
        ROS_WARN("SCENE:: Service returned was not sucessful");
    }
}

void QGraphicsSceneMapViewer::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if(/*isActive() &&*/ !ignoreSetup)
    {
        if(mouseEvent->button() == Qt::LeftButton)
        {
            ROS_DEBUG("SCENE:: Left Mouse event");
            mouseEvent->accept();
            setupMap(mouseEvent->buttonDownScenePos(Qt::LeftButton));
            ignoreSetup = true;
            return;
        }
    }
    mouseEvent->ignore();
}

bool QGraphicsSceneMapViewer::setupMap(QPointF scenePos)
{
    if(!isMapSetup())
    {
        startPlatformCenter.setX(scenePos.x());
        startPlatformCenter.setY(scenePos.y());
        robotToObjTransform.translate(startPlatformCenter.x(), startPlatformCenter.y());
        robotToObjTransform.rotate(lastRobotPose.northAngle);
        this->setItemIndexMethod(QGraphicsScene::NoIndex);
    }
    else
    {
        /*handle re-init, probably clear obj containers and what not.*/
    }
    for(std::map<map_viewer_enums::mapViewerLayers_t, std::string>::iterator it = map_viewer_enums::mapViewerLayersToString.begin();
            it != map_viewer_enums::mapViewerLayersToString.end(); it++)
    {
        _implInitLayer(getLayerFromEnum(it->first), isMapSetup());
    }
    emit map_viewer_scene_init(isMapSetup());
    mapSetup = true;
    return isMapSetup();
}

void QGraphicsSceneMapViewer::_implSetupLayer(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility)
{
    ROS_DEBUG("SCENE:: _implSetupLayer");
    if(!isMapSetup())
    {
        ROS_ERROR("Map has been setup yet");
        return;
    }

    if(visibility)
    {
        generic_ack_dialog dialog("This is layer has not been activated before.\r\nIt could take a while to display the first time");
        switch(mapLayer)
        {
        case map_viewer_enums::keyframeDrive:
        case map_viewer_enums::SatDriveability:
            if(dialog.bringUpDialogModal() == QDialog::Accepted)
            {
                ROS_DEBUG("Send Global Map Request");
                emit request_global_map(mapLayer);
            }
            break;
        default:
            ROS_WARN("Layer drawing not implemented yet");
        }
    }
    else
    {
        mapLayer_t *layer = getLayerFromEnum(mapLayer);
        if(layer->properties.isLayerSetup)
        {
            layer->properties.isLayerVisible = visibility;
            layer->items->setVisible(visibility);
        }
    }
}

void QGraphicsSceneMapViewer::on_set_ignore_setup_flag(bool status)
{
    ROS_DEBUG("SCENE:: setup up flag %d", (int) ignoreSetup);
    ROS_DEBUG("SCENE:: am i active? %d", (int)this->isActive());
    //if(isActive())
    {
        ROS_DEBUG("Setting setup flag %d", (int) status);
        ignoreSetup = status;
    }
}

void QGraphicsSceneMapViewer::on_set_layer_visibility(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility)
{
    //if(isActive())
    {
        if(!isMapSetup())
        {
            ROS_DEBUG("SCENE:: Map is not setup yet");
        }
        ROS_DEBUG("Setting Layer %s to %d", map_viewer_enums::mapViewerLayersToString.find(mapLayer)->second.c_str(), (int)visibility);
        _implSetupLayer(mapLayer, visibility);


    }
}

QGraphicsSceneMapViewer::mapLayer_t * QGraphicsSceneMapViewer::getLayerFromEnum(const map_viewer_enums::mapViewerLayers_t &mapLayerEnum)
{
    switch(mapLayerEnum)
    {
    case map_viewer_enums::GRIDMAP:
        return &gridMapLayer;
    case map_viewer_enums::ROI:
        return &roiLayer;
    case map_viewer_enums::PATH:
        return &pathLayer;
    case map_viewer_enums::keyframeDrive:
        return &keyframeLayer;
    case map_viewer_enums::SatDriveability:
        return &satDriveabilityLayer;
    default:
        return 0;
    }
}

void QGraphicsSceneMapViewer::_implInitLayer(mapLayer_t *layer, bool reInit)
{
    layer->properties.isLayerSetup = false;
    layer->properties.isLayerVisible = false;
    //layer->items.reset(new QGraphicsItemGroup);
    //TODO need to check item group and remove it from the scene
    layer->itemList.reset(new QList<QGraphicsItem*>());
}

void QGraphicsSceneMapViewer::on_hsm_global_pose_callback(const messages::RobotPose navInfo)
{
    //ROS_DEBUG("SCENE:: HSM Global pose callback");
    lastRobotPose = navInfo;
}

void QGraphicsSceneMapViewer::_implConnectSignals()
{
    connect(this, &QGraphicsSceneMapViewer::request_global_map,
                worker.get(), &ros_workers::on_run_map_manager_global_map_request);

    connect(worker.get(), &ros_workers::map_manager_global_map_service_returned,
                this, &QGraphicsSceneMapViewer::on_map_manager_gridmap_service_returned);

    connect(worker.get(), &ros_workers::hsm_global_pose_callback,
                this, &QGraphicsSceneMapViewer::on_hsm_global_pose_callback);

    worker->on_run_hsm_global_pose_subscriber_start();
}

void QGraphicsSceneMapViewer::_implDisconnectSignals()
{
    disconnect(this, &QGraphicsSceneMapViewer::request_global_map,
                0,0);

    worker->disconnect(this);
}
