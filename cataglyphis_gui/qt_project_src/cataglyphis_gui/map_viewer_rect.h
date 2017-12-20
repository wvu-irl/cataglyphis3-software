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

#ifndef MAP_VIEWER_RECT_H
#define MAP_VIEWER_RECT_H

#include <QRectF>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <QGraphicsRectItem>
#include <ros/ros.h>

#define MAP_CELL_MAX_VALUE 100.0
#define MAP_CELL_MIN_VALUE 0.0
#define MAP_CELL_NOOP_VALUE -5.0

class map_viewer_rect : public QGraphicsRectItem
{
public:

    static float mapRange(const float &valToMap, const float &mir1, const float &mar1, const float &mir2, const float &mar2)
    {
        return (((valToMap - mir1)*(mar2-mir2))/(mar1-mir1)) + mir2;
    }

    map_viewer_rect(float mapValue = 1, float min = 0.0, float max = 10.0,
                    QGraphicsItem * parent = 0,
                    int hueMin = 120, int hueMax = 0,
                    int transparency = 160):
                        QGraphicsRectItem(parent),
                        fillColor(QColor::fromHsv(mapRange(mapValue, min, max, hueMin, hueMax),
                                                             200, 255, transparency)),
                        borderColor(QColor::fromRgb(0,0,0,0)),
                        fillBrush(fillColor),
                        borderPen(borderColor)
    {
        this->setCacheMode(QGraphicsItem::ItemCoordinateCache);

        if(mapValue >= MAP_CELL_MAX_VALUE)
        {
            fillColor.setRgb(0,0,0);
            fillBrush.setColor(fillColor);
        }
        this->setBrush(fillBrush);
        this->setPen(borderPen);
    }

    void setFillColor(QColor fill)
    {
        fillColor.setRgb(fill.rgba());
        fillBrush.setColor(fillColor);
        this->setBrush(fillBrush);
    }

    ~map_viewer_rect(){ /*ROS_DEBUG("Rectangle Desctructor"); */}

    QColor fillColor;
    QColor borderColor;
    QBrush fillBrush;
    QPen   borderPen;
};

#endif // MAP_VIEWER_RECT_H
