#ifndef MAP_VIEWER_RECT_H
#define MAP_VIEWER_RECT_H

#include <QRectF>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <QGraphicsRectItem>
#include <ros/ros.h>

class map_viewer_rect : public QGraphicsRectItem
{
public:
    
    static float mapRange(const float &valToMap, const float &mir1, const float &mar1, const float &mir2, const float &mar2)
    {
        return (((valToMap - mir1)*(mar2-mir2))/(mar1-mir1)) + mir2;
    }
    
    map_viewer_rect(float mapValue = 1, float min = 0.0, float max = 10.0,
                    int hueMin = 120, int hueMax = 0,
                    int transparency = 100,
                    QGraphicsItem * parent = 0):
                        QGraphicsRectItem(parent),
                        fillColor(QColor::fromHsv(mapRange(mapValue, min, max, hueMin, hueMax),
                                                             200, 255, transparency)),
                        borderColor(QColor::fromRgb(0,0,0,0)),
                        fillBrush(fillColor),
                        borderPen(borderColor)
    {
        this->setCacheMode(QGraphicsItem::ItemCoordinateCache);

        this->setBrush(fillBrush);
        this->setPen(borderPen);
    }

    ~map_viewer_rect(){ /*ROS_DEBUG("Rectangle Desctructor"); */}

    QColor fillColor;
    QColor borderColor;
    QBrush fillBrush;
    QPen   borderPen;
};

#endif // MAP_VIEWER_RECT_H
