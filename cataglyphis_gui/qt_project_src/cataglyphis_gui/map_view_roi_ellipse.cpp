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

#include "map_view_roi_ellipse.h"


map_view_roi_ellipse::map_view_roi_ellipse(robot_control::ROI roiData, int roiNum, float pixelsPerDistance,
                                            const QTransform &mapTransform, const QPointF &mapCenter,
                                            boost::shared_ptr<ros_workers> workerArg, QGraphicsItem *parent):
   QGraphicsEllipseItem(parent),
   fillColor(QColor::fromRgb(255,0,255,100)),
   borderColor(QColor::fromRgb(0,0,0,0)),
   fillBrush(fillColor),
   borderPen(borderColor),
   roiTextNumber(QString::number(roiNum))
{
   worker = workerArg;
   pixelsPerDist = pixelsPerDistance;
   roiNumber = roiNum;
   this->setCacheMode(QGraphicsItem::ItemCoordinateCache);
   this->setBrush(fillBrush);
   this->setPen(borderPen);

   roiDialog.reset(new ROI_dialog(roiData, roiNum, worker));
   _implConnectSignals();
   on_update_roi(roiData, false);

   //this->setTransformOriginPoint(roiData.radialAxis,roiData.radialAxis);
//   roiTextNumber.setTransformOriginPoint(mapCenter);
   roiTextNumber.setParentItem(this);
   roiTextNumber.adjustSize();
   roiTextNumber.show();
   roiTextNumber.setPos(roiData.x-roiTextNumber.boundingRect().x(), roiData.y-roiTextNumber.boundingRect().y());
   //roiTextNumber.hide();
}

void map_view_roi_ellipse::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    ROS_DEBUG("Double Click Event");
    if(event->button() == Qt::LeftButton)
    {
        event->accept();
        ROS_WARN("ROI Clicked");
        roiDialog->open();
    }
    event->ignore();
}

void map_view_roi_ellipse::on_update_roi(robot_control::ROI roiData, bool modified)
{
    this->setRect(roiData.x-roiData.radialAxis,roiData.y-roiData.radialAxis,
                    roiData.radialAxis*2,roiData.tangentialAxis*2);
    QGraphicsLineItem *test = new QGraphicsLineItem(roiData.x,roiData.y,roiData.x+5,roiData.y,this);
//    QGraphicsLineItem *test = new QGraphicsLineItem(0,0,25,0,this);
//    test->resetTransform();
//    test->show();
}

void map_view_roi_ellipse::on_confirm_ROI_changes()
{
    emit confirm_ROI_changes();
}

void map_view_roi_ellipse::on_discard_ROI_changes()
{
    emit discard_ROI_changes();
}

void map_view_roi_ellipse::_implConnectSignals()
{
    connect(roiDialog.get(), &ROI_dialog::update_roi_ellipse,
                this, &map_view_roi_ellipse::on_update_roi);

    connect(this, &map_view_roi_ellipse::confirm_ROI_changes,
                roiDialog.get(), &ROI_dialog::on_confirm_changes);
    connect(this, &map_view_roi_ellipse::discard_ROI_changes,
                roiDialog.get(), &ROI_dialog::on_discard_changes);
}
