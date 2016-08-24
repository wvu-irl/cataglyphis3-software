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
    }

};

#endif // QGRAPHICSVIEWSCROLLSCALE_H
