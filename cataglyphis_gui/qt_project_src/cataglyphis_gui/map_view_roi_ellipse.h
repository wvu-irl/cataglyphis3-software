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
public:
    map_view_roi_ellipse(QGraphicsItem *parent = 0) :
                QGraphicsEllipseItem(parent) {roiDialog.reset(new ROI_dialog);}

    map_view_roi_ellipse(robot_control::ROI roiData, int roiNum, float pixelsPerDistance,
                            const QTransform &mapTransform, const QPointF &mapCenter,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>());

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
    void on_map_manager_service_return();
    void on_update_roi(robot_control::ROI roiData);

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
