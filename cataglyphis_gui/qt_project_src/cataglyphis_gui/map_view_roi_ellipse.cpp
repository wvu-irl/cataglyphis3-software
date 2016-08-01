#include "map_view_roi_ellipse.h"


void map_view_roi_ellipse::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    event->accept();
    if(event->button() == Qt::LeftButton)
    {
        ROS_DEBUG("ROI Clicked");
        roiDialog->show();
        roiDialog->raise();
        roiDialog->activateWindow();
    }
}

void map_view_roi_ellipse::on_draw_roi()
{

}

void map_view_roi_ellipse::on_map_manager_service_return()
{

}
