#ifndef MAP_VIEW_ROI_ELLIPSE_H
#define MAP_VIEW_ROI_ELLIPSE_H

#include <QPen>
#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>

#include <ros/ros.h>
#include <roi_dialog.h>
#include <boost/scoped_ptr.hpp>

class map_view_roi_ellipse : public QGraphicsEllipseItem
{
public:
    map_view_roi_ellipse(QGraphicsEllipseItem *parent = 0) :
        QGraphicsEllipseItem(parent),
        roiDialog(new ROI_dialog()) {}
    map_view_roi_ellipse(const QRectF & rect, QGraphicsItem * parent = 0) :
        QGraphicsEllipseItem(rect, parent),
        roiDialog(new ROI_dialog()) {}
    map_view_roi_ellipse(qreal x, qreal y, qreal width, qreal height, QGraphicsItem * parent = 0) :
        QGraphicsEllipseItem(x, y, width, height, parent),
        roiDialog(new ROI_dialog()) {}
    map_view_roi_ellipse(qreal x, qreal y, qreal width, qreal height, QGraphicsItem *parent = 0,
                            const QPen & pen = QPen(), const QBrush & brush = QBrush()) :
        QGraphicsEllipseItem(x, y, width, height, parent),
        roiDialog(new ROI_dialog())
    {

        this->setBrush(brush);
        this->setPen(pen);
    }
    map_view_roi_ellipse(qreal x, qreal y, qreal width, qreal height,
                            const QPen & pen = QPen(), const QBrush & brush = QBrush()) :
        QGraphicsEllipseItem(x, y, width, height, 0),
        roiDialog(new ROI_dialog())
    {

        this->setBrush(brush);
        this->setPen(pen);
    }

    //TODO Need a constructor that takes some sort of gridmap object
    //TODO need to overide a mouse click event that can generate window popups or fill in a window dock window

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

public slots:
    void on_map_manager_service_return();
    void on_draw_roi();

private:
    boost::scoped_ptr<ROI_dialog> roiDialog;

};

#endif // MAP_VIEW_ROI_ELLIPSE_H
