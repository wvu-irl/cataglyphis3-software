#include "qgraphicsscenemapviewer.h"

void QGraphicsSceneMapViewer::on_map_manager_gridmap_service_returned(messages::GlobalMapFull gridMapFull,
                                                                          map_viewer_enums::mapViewerLayers_t requestedLayer,
                                                                            bool wasSucessful)
{
    //generate a texture based on the gridmap, then draw the texture
    ROS_DEBUG("SCENE:: gridmap service returned");
    if(wasSucessful)
    {
        ROS_DEBUG("SCENE:: grid map Service call was sucessful");
        mapLayer_t *layer = getLayerFromEnum(requestedLayer);
        if(!layer->properties.isLayerSetup)
        {
            grid_map::GridMapRosConverter::fromMessage(gridMapFull.response.globalMap, gridMapContainer);
            ROS_WARN("SCENE:: Grid MAP size %d, %d", gridMapContainer.getSize()[0], gridMapContainer.getSize()[1]);
            std::string gridMapLayerName = map_viewer_enums::gridMapLayersToString.find(map_viewer_enums::mapViewerLayersToGridMapLayers.find(requestedLayer)->second)->second;
            //QGraphicsScene drawingScene;
            QGraphicsView drawingView;// = this->views().front();
            drawingView.setStyleSheet("background: transparent;");
            drawingView.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            drawingView.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            drawingView.setScene(this);
            drawingView.setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
            drawingView.setTransform(this->views().front()->transform());

            bool satVisible = false;
            if(getLayerFromEnum(map_viewer_enums::SatDriveability)->properties.isLayerSetup)
            {
                satVisible = getLayerFromEnum(map_viewer_enums::SatDriveability)->pixmapItem->isVisible();
                getLayerFromEnum(map_viewer_enums::SatDriveability)->pixmapItem->hide();
            }
            bool slamVisible = false;
            if(getLayerFromEnum(map_viewer_enums::keyframeDrive)->properties.isLayerSetup)
            {
                slamVisible = getLayerFromEnum(map_viewer_enums::keyframeDrive)->pixmapItem->isVisible();
                getLayerFromEnum(map_viewer_enums::keyframeDrive)->pixmapItem->hide();
            }

            for(grid_map::GridMapIterator it(gridMapContainer); !it.isPastEnd(); it= (++it))
            {
                float mapValue = gridMapContainer.at(gridMapLayerName, *it);
                grid_map::Position cellPosition;
                gridMapContainer.getPosition(*it, cellPosition);
                map_viewer_rect *gridRectangle;
                /*cellPosition[0] = x*/
//                if((mapValue != MAP_CELL_MAX_VALUE)
//                        && (mapValue != MAP_CELL_NOOP_VALUE))
                if(requestedLayer == map_viewer_enums::keyframeDrive)
                {
                    if(mapValue != 0.0)
                    {
                        ROS_DEBUG("Obstacle Located at x: %2.1f, y: %2.1f", cellPosition[0], cellPosition[1]);
                        gridRectangle = new map_viewer_rect(mapValue, MAP_CELL_MIN_VALUE, MAP_CELL_MAX_VALUE);
                        gridRectangle->setFillColor(QColor::fromRgb(255,0,0));
                        gridRectangle->setRect((float)cellPosition[0], (float)cellPosition[1], 1, 1);
                        gridRectangle->setTransformOriginPoint(startPlatformCenter);
                        gridRectangle->setTransform(robotToObjTransform);
                        gridRectangle->setGroup(layer->items.get());
                        layer->itemList->append(gridRectangle);
                    }
                }
                else
                {
                    gridRectangle = new map_viewer_rect(mapValue, MAP_CELL_MIN_VALUE, MAP_CELL_MAX_VALUE);
                    //ROS_DEBUG("Position %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",cellPosition[0], cellPosition[1], cellPosition[0], cellPosition[1], pixelsPerDistance, pixelsPerDistance);
                    gridRectangle->setRect((float)cellPosition[0], (float)cellPosition[1], 1, 1);
                    gridRectangle->setTransformOriginPoint(startPlatformCenter);
                    gridRectangle->setTransform(robotToObjTransform);
                    gridRectangle->setGroup(layer->items.get());
                    layer->itemList->append(gridRectangle);
                }
            }



            ROS_DEBUG("SCENE:: Finished Reading Global Map");
            ROS_DEBUG("Number of items in scene: %d", layer->itemList->size());
            ROS_DEBUG("SCENE:: adding group to scene");
            //drawingScene.addItem(layer->items.get());

            this->addItem(layer->items.get());
            areaImagePixmap->hide();
            //drawingScene.removeItem(tempitem.get());
            //drawingView.fitInView(layer->items->childrenBoundingRect(), Qt::KeepAspectRatio);
            drawingView.setFixedSize(areaImagePixmap->boundingRect().width(),
                                        areaImagePixmap->boundingRect().height());
            drawingView.setTransform(this->views().front()->transform());
            ROS_DEBUG("SCENE:: showing layer");
            layer->items->show();
            ROS_DEBUG("SCENE:: DONE");

            ROS_DEBUG("SCENE:: Generating drawingScene Texture");
            QPixmap temp = drawingView.grab();
            layer->gridPixmap->swap(temp);
            layer->pixmapItem.reset(new QGraphicsPixmapItem(*layer->gridPixmap));
            //pixmapItem->setFlag(QGraphicsItem::ItemIsMovable);
            ROS_DEBUG("SCENE:: Rendering texture of field display");
            layer->items->hide();
            areaImagePixmap->show();
            if(satVisible){getLayerFromEnum(map_viewer_enums::SatDriveability)->pixmapItem->setVisible(satVisible);}
            if(slamVisible){getLayerFromEnum(map_viewer_enums::keyframeDrive)->pixmapItem->setVisible(slamVisible);}
            this->addItem(layer->pixmapItem.get());
            //fix scrollbar on virtual scene
            layer->properties.isLayerSetup = true;

        }

        layer->properties.isLayerVisible = true;
    }
    else
    {
        ROS_WARN("SCENE:: Service returned was not sucessful");
    }
}

void QGraphicsSceneMapViewer::on_confirm_map_changes()
{
    emit confirm_roi_changes();
    roiLayer.properties.isLayerSetup = false;
    _implSetupLayer(map_viewer_enums::ROI, roiLayer.properties.isLayerVisible);
}

void QGraphicsSceneMapViewer::on_discard_map_changes()
{
    emit discard_roi_changes();
}

void QGraphicsSceneMapViewer::on_map_manager_roi_service_returned(const robot_control::RegionsOfInterest mapManagerResponse, bool wasSucessful)
{
    ROS_DEBUG("SCENE:: ROI service returned");

    if(wasSucessful)
    {
        ROS_DEBUG("SCENE:: service successful");
        for(int i = 0; i < roiLayer.itemList->size(); i++)
        {
            this->removeItem(roiLayer.itemList->at(i));
            ROS_DEBUG("Deleting Item");
            delete roiLayer.itemList->at(i);
        }
        roiLayer.itemList->clear();
        auto listPtr = &mapManagerResponse.response.ROIList;
        for(unsigned int i = 0; i < listPtr->size(); i++)
        {
            ROS_DEBUG("SCENE:: ROI at x:%2.3f, y:%2.3f", listPtr->at(i).x, listPtr->at(i).y);
            map_view_roi_ellipse *roiEllipse = new map_view_roi_ellipse(listPtr->at(i), i, pixelsPerDistance, robotToObjTransform,
                                                                            startPlatformCenter, worker);
            connect(this, &QGraphicsSceneMapViewer::confirm_roi_changes,
                        roiEllipse, &map_view_roi_ellipse::on_confirm_ROI_changes);
            connect(this, &QGraphicsSceneMapViewer::discard_roi_changes,
                        roiEllipse, &map_view_roi_ellipse::on_discard_ROI_changes);
            //roiEllipse->setGroup(roiLayer.items.get());
            roiLayer.itemList->append(roiEllipse);
        }
        for(int i = 0; i < roiLayer.itemList->size(); i++)
        {
            this->addItem(roiLayer.itemList->at(i));
            roiLayer.itemList->at(i)->show();
        }

        roiLayer.properties.isLayerSetup = true;
        roiLayer.properties.isLayerVisible = true;
    }
    else
    {
        ROS_WARN("SCENE:: Serivce was not successful!");
    }
}

bool QGraphicsSceneMapViewer::_implGenericGridMapLayerDisplay()
{
    return true;
}

bool QGraphicsSceneMapViewer::_implSatelliteMapDisplay()
{
    return true;
}

bool QGraphicsSceneMapViewer::_implHazardMapDisplay()
{
    return true;
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
            ROS_DEBUG("SCENE:: Setting New Cursor");
            //((map_viewer*)parent())->ui->fieldDisplay->setCursor(Qt::OpenHandCursor);
            return;
        }
    }
    mouseEvent->ignore();
}

bool QGraphicsSceneMapViewer::setupMap(QPointF scenePos)
{
    //if(!isMapSetup())
    {
        ROS_DEBUG("SCENE:: Robot start pos in pixels %2.3f, %2.3f", scenePos.x(), scenePos.y());
        ROS_DEBUG("SCENE:: Delta Start Pos pixels: %2.3f %2.3f... Meters %2.3f %2.3f", scenePos.x()- startPlatformCenter.x(), scenePos.y()- startPlatformCenter.y(), (scenePos.x()- startPlatformCenter.x())/pixelsPerDistance, (scenePos.y()-startPlatformCenter.y())/pixelsPerDistance);
        startPlatformCenter.setX(scenePos.x());
        startPlatformCenter.setY(scenePos.y());
        robotToObjTransform.reset();
        robotToObjTransform.translate(startPlatformCenter.x(), startPlatformCenter.y());
        robotToObjTransform.rotate(lastRobotPose.northAngle-90);
        robotToObjTransform.scale(pixelsPerDistance,pixelsPerDistance);
        this->setItemIndexMethod(QGraphicsScene::NoIndex);
    }
   // else
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
    mapLayer_t *layer = getLayerFromEnum(mapLayer);
    if(!layer->properties.isLayerSetup && visibility)
    {
        generic_ack_dialog dialog("This is layer has not been activated before.\r\nIt could take a while to display the first time");
        switch(mapLayer)
        {
        case map_viewer_enums::keyframeDrive:
        case map_viewer_enums::SatDriveability:
            if(dialog.bringUpDialogModal() == QDialog::Accepted)
            {
                ROS_DEBUG("SCENE:: Send Global Map Request");
                emit request_global_map(mapLayer);
            }
            break;
        case map_viewer_enums::ROI:
            ROS_DEBUG("SCENE::Requesting ROIs");
            emit request_roi();
            break;
        default:
            ROS_WARN("Layer drawing not implemented yet");
        }
    }
    else
    {
        if(layer->properties.isLayerSetup)
        {
            switch(mapLayer)
            {
            case map_viewer_enums::keyframeDrive:
            case map_viewer_enums::SatDriveability:
                layer->properties.isLayerVisible = visibility;
                //layer->items->setVisible(visibility);
                layer->pixmapItem->setVisible(visibility);
                break;
            case map_viewer_enums::ROI:
                for(int i = 0; i < roiLayer.itemList->size(); i++)
                {
                    roiLayer.itemList->at(i)->setVisible(visibility);
                }
                roiLayer.properties.isLayerSetup = false;
                break;
            default:
                layer->properties.isLayerVisible = visibility;
                layer->items->setVisible(visibility);
                //layer->pixmapItem->setVisible(visibility);
            }
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

void QGraphicsSceneMapViewer::redrawLayers()
{

}

void QGraphicsSceneMapViewer::_implInitLayer(mapLayer_t *layer, bool reInit)
{
    layer->properties.isLayerSetup = false;
    layer->properties.isLayerVisible = false;
    //layer->items.reset(new QGraphicsItemGroup);
    //TODO need to check item group and remove it from the scene
    layer->gridPixmap.reset(new QPixmap());
    layer->itemList.reset(new QList<QGraphicsItem*>());
    layer->items.reset(new QGraphicsItemGroup());
    layer->items->hide();

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

    connect(this, &QGraphicsSceneMapViewer::request_roi,
                worker.get(), &ros_workers::on_run_map_manager_ROI_service);

    connect(worker.get(), &ros_workers::map_manager_ROI_service_returned,
                this, &QGraphicsSceneMapViewer::on_map_manager_roi_service_returned);

    worker->on_run_hsm_global_pose_subscriber_start();
}

void QGraphicsSceneMapViewer::_implDisconnectSignals()
{
    disconnect(this, &QGraphicsSceneMapViewer::request_global_map,
                0,0);
    disconnect(this, &QGraphicsSceneMapViewer::request_roi,
                0,0);

    worker->disconnect(this);
}
