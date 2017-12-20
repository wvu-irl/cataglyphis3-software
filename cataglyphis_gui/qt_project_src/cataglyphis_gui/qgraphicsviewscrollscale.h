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

#ifndef QGRAPHICSVIEWSCROLLSCALE_H
#define QGRAPHICSVIEWSCROLLSCALE_H

#include <iostream>
#include <QWidget>
#include <QPoint>
#include <QWheelEvent>
#include <QGraphicsView>
#include <QGraphicsSceneWheelEvent>

class QGraphicsViewScrollScale : public QGraphicsView
{
public:
    QGraphicsViewScrollScale(QWidget * parent = 0) :
        QGraphicsView(parent) {}
    QGraphicsViewScrollScale(QGraphicsScene * scene, QWidget * parent = 0) :
        QGraphicsView(scene, parent) {}

    void wheelEvent(QWheelEvent *event)
    {
        QPoint numDegrees = event->angleDelta();
        if (!numDegrees.isNull()) {
            QPointF numSteps = (QPointF)numDegrees / 1200.0;
            scale((numSteps.y()<1)?numSteps.y()+1:numSteps.y(), (numSteps.y()<1)?numSteps.y()+1:numSteps.y());
        }
        event->accept();
        //event->ignore();
    }

protected:
    void enterEvent(QEvent *event)
    {
        QGraphicsView::enterEvent(event);
        viewport()->setCursor(Qt::CrossCursor);
    }

    void mousePressEvent(QMouseEvent *event)
    {
        QGraphicsView::mousePressEvent(event);
        viewport()->setCursor(Qt::CrossCursor);
    }

    void mouseReleaseEvent(QMouseEvent *event)
    {
        QGraphicsView::mouseReleaseEvent(event);
        viewport()->setCursor(Qt::CrossCursor);
    }

};

#endif // QGRAPHICSVIEWSCROLLSCALE_H
