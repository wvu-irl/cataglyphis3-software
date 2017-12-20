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

#ifndef MAP_VIEW_ROI_ELLIPSE_H
#define MAP_VIEW_ROI_ELLIPSE_H

#include <QObject>
#include <QPen>
#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>

#include <robot_control/ROI.h>

#include <ros/ros.h>
#include <ros_workers.h>
#include <roi_dialog.h>
#include <boost/scoped_ptr.hpp>

class map_view_roi_ellipse : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT

signals:
    void confirm_ROI_changes();
    void discard_ROI_changes();

public:
    map_view_roi_ellipse(QGraphicsItem *parent = 0) :
                QGraphicsEllipseItem(parent) {roiDialog.reset(new ROI_dialog);}

    map_view_roi_ellipse(robot_control::ROI roiData, int roiNum, float pixelsPerDistance,
                            const QTransform &mapTransform, const QPointF &mapCenter,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>(),
                            QGraphicsItem *parent = 0);

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

    void setFillColor(QColor fill)
    {
        fillColor.setRgb(fill.rgba());
        fillBrush.setColor(fillColor);
        this->setBrush(fillBrush);
    }

    bool isModified(){ return roiDialog->isModified(); }

public slots:
    void on_confirm_ROI_changes();
    void on_discard_ROI_changes();
    void on_update_roi(robot_control::ROI roiData, bool modified);

private:
    boost::scoped_ptr<ROI_dialog> roiDialog;
    boost::shared_ptr<ros_workers> worker;

    QColor fillColor;
    QColor borderColor;
    QBrush fillBrush;
    QPen   borderPen;

    float pixelsPerDist;
    int roiNumber;
    QGraphicsTextItem roiTextNumber;

    void _implConnectSignals();
};

#endif // MAP_VIEW_ROI_ELLIPSE_H
