#include "map_view_roi_ellipse.h"


map_view_roi_ellipse::map_view_roi_ellipse(robot_control::ROI roiData, int roiNum, float pixelsPerDistance,
                                            const QTransform &mapTransform, const QPointF &mapCenter,
                                            boost::shared_ptr<ros_workers> workerArg):
   QGraphicsEllipseItem(0),
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

   this->setTransformOriginPoint(mapCenter);
   this->setTransform(mapTransform);
//   roiTextNumber.setTransformOriginPoint(mapCenter);
//   roiTextNumber.setTransform(mapTransform);
//   roiTextNumber.setPos(0,0/*roiTextNumber->parentItem()->boundingRect().width()/2-roiTextNumber->boundingRect().width()/2,
//                           roiTextNumber->parentItem()->boundingRect().height()/2-roiTextNumber->boundingRect().height()/2*/);
   roiTextNumber.show();
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
    this->setRect(roiData.x,roiData.y,
                    roiData.radialAxis,roiData.tangentialAxis);
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
