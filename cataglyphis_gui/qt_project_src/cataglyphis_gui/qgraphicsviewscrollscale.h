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
//        QPoint numDegrees = event->angleDelta();
//        if (!numDegrees.isNull()) {
//            QPointF numSteps = (QPointF)numDegrees / 1200.0;
//            scale((numSteps.y()<1)?numSteps.y()+1:numSteps.y(), (numSteps.y()<1)?numSteps.y()+1:numSteps.y());
//        }
//        event->accept();
        event->ignore();
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
