#ifndef MAP_VIEWER_RECT_H
#define MAP_VIEWER_RECT_H

#include <QRectF>
#include <QGraphicsRectItem>
#include <ros/ros.h>

class map_viewer_rect : public QGraphicsRectItem
{
public:
    map_viewer_rect(QGraphicsItem * parent = 0): QGraphicsRectItem(parent) {}
    map_viewer_rect(const QRectF & rect, QGraphicsItem * parent = 0): QGraphicsRectItem(rect, parent) {}
    map_viewer_rect(qreal x, qreal y, qreal width, qreal height, QGraphicsItem * parent = 0):
                        QGraphicsRectItem(x, y, width, height, parent) {}

    ~map_viewer_rect(){ ROS_DEBUG("Rectangle Desctructor"); }

    QColor defaultCircleFill;
    QColor fullTransparentColor;

};

#endif // MAP_VIEWER_RECT_H
