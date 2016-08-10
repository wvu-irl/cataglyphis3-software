#ifndef QGRAPHICSSCENEMAPVIEWER_H
#define QGRAPHICSSCENEMAPVIEWER_H

#include <QObject>
#include <QRectF>
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsItemGroup>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsRectItem>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

class QGraphicsSceneMapViewer : public QGraphicsScene
{
public:
    QGraphicsSceneMapViewer(QObject * parent = 0):
        QGraphicsScene(parent) {}
    QGraphicsSceneMapViewer(const QRectF & sceneRect, QObject * parent = 0):
        QGraphicsScene(sceneRect, parent) {}
    QGraphicsSceneMapViewer(qreal x, qreal y, qreal width, qreal height, QObject * parent = 0):
        QGraphicsScene(x, y, width, height, parent) {}

    QGraphicsSceneMapViewer(const char *imagePath, QObject * parent = 0):
        QGraphicsScene(parent)
    {
        QImage tempImage(imagePath);
        this->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(tempImage)));
    }
    QGraphicsSceneMapViewer(const QImage &image, QObject * parent = 0):
        QGraphicsScene(parent)
    {
        this->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(image)));
    }

    bool isMapSetup;

    boost::shared_array<boost::shared_ptr<QImage> > areaImage;
    boost::shared_ptr<QGraphicsRectItem> cataglyphisRect;
};

#endif // QGRAPHICSSCENEMAPVIEWER_H
