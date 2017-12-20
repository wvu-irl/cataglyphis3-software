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

#ifndef MAP_VIEWER_H
#define MAP_VIEWER_H

#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsItemGroup>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QWidget>
#include <QPixmapCache>
#include <map_viewer_enums.h>
#include <qgraphicsscenemapviewer.h>
#include <map_view_roi_ellipse.h>
#include <generic_error_dialog.h>
#include <roi_dialog.h>
#include <boost/smart_ptr.hpp>
#include <stdio.h>
#include <ros/ros.h>
#include <ros_workers.h>

#define NUM_OF_MAPS 4

namespace Ui {
class map_viewer_form;
}

class map_viewer : public QWidget
{
    Q_OBJECT

signals:
    void request_map_manager_ROI();
    void run_global_map_request(map_viewer_enums::mapViewerLayers_t requestedLayer);
    void set_map_layer_visibility(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility);
    void set_scene_setup_alert(bool status);
    void center_on_cataglyphis(bool status);
    void new_path_length(int length);
    void new_path_step_size(double stepSize);
    void confirm_map_changes();
    void discard_map_changes();

public slots:
    void on_global_map_service_returned(messages::GlobalMapFull gridMapFull, map_viewer_enums::mapViewerLayers_t requestedLayer,
                                        bool wasSucessful);

public:
    explicit map_viewer(QWidget *parent = 0, int startIndex = 0);
    ~map_viewer();

    //boost::shared_ptr<Ui::map_viewer> ui;
    Ui::map_viewer_form* ui;

private slots:
    void on_fieldSelector_currentIndexChanged(int index);

//    void on_drawTestShapesButton_clicked();

    void on_keyframeMapLayerButton_clicked(bool checked);

    void on_roiMapLayerButton_clicked(bool checked);

    void on_pathLayerButton_clicked(bool checked);

    void on_gridMapLayerButton_clicked(bool checked);

    void on_initMapButton_clicked();

    void on_satDriveMapLayerButton_clicked(bool checked);

    void on_refresh_layers_button_clicked();

    void on_commit_changes_button_clicked();

    void on_discard_changes_button_clicked();

    void on_center_on_cataglyphis_clicked(bool checked);

    void on_path_length_valueChanged(int arg1);

    void on_doubleSpinBox_valueChanged(double arg1);

private:
    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;

    QPointF startPlatformPositionsPixels[10][3];

    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    boost::shared_array<boost::scoped_ptr<QGraphicsSceneMapViewer> > scene;
};

#endif // MAP_VIEWER_H
