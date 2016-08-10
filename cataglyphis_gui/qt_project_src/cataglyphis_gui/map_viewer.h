#ifndef MAP_VIEWER_H
#define MAP_VIEWER_H

#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsItemGroup>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QWidget>
#include "qgraphicsscenemapviewer.h"
#include <map_view_roi_ellipse.h>
#include <generic_error_dialog.h>
#include <roi_dialog.h>
#include <boost/smart_ptr.hpp>
#include <stdio.h>

namespace Ui {
class map_viewer_form;
}

class map_viewer : public QWidget
{
    Q_OBJECT

signals:
    void request_map_manager_ROI();

public:
    explicit map_viewer(QWidget *parent = 0, int startIndex = 0);
    ~map_viewer();

    //boost::shared_ptr<Ui::map_viewer> ui;
    Ui::map_viewer_form* ui;

public slots:
    void draw_new_roi(/*put roi object here*/);

private slots:
    void on_fieldSelector_currentIndexChanged(int index);

    void on_drawTestShapesButton_clicked();

    void on_slamMapLayerButton_clicked(bool checked);

    void on_roiMapLayerButton_clicked(bool checked);

    void on_pathLayerButton_clicked(bool checked);

    void on_gridMapLayerButton_clicked(bool checked);

private:
    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    boost::shared_array<boost::shared_ptr<QGraphicsSceneMapViewer> > scene;
};

#endif // MAP_VIEWER_H
