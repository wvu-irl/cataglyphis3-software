#ifndef MAP_VIEWER_H
#define MAP_VIEWER_H

#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsItemGroup>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QWidget>
#include <QPixmapCache>
#include <map_viewer_enums.h>
#include <qgraphicsscenemapviewer.h>
#include <map_view_roi_ellipse.h>
#include <generic_error_dialog.h>
#include <roi_dialog.h>
#include <boost/smart_ptr.hpp>
#include <stdio.h>
#include <ros/ros.h>
#include <ros_workers.h>

#define NUM_OF_MAPS 4

namespace Ui {
class map_viewer_form;
}

class map_viewer : public QWidget
{
    Q_OBJECT

signals:
    void request_map_manager_ROI();
    void set_map_layer_visibility(map_viewer_enums::mapViewerLayers_t mapLayer, bool visibility);
    void set_scene_setup_alert(bool status);

public:
    explicit map_viewer(QWidget *parent = 0, int startIndex = 0, boost::shared_ptr<ros::NodeHandle> nhArg =
                                                                        boost::shared_ptr<ros::NodeHandle>());
    ~map_viewer();

    //boost::shared_ptr<Ui::map_viewer> ui;
    Ui::map_viewer_form* ui;

private slots:
    void on_fieldSelector_currentIndexChanged(int index);

//    void on_drawTestShapesButton_clicked();

    void on_keyframeMapLayerButton_clicked(bool checked);

    void on_roiMapLayerButton_clicked(bool checked);

    void on_pathLayerButton_clicked(bool checked);

    void on_gridMapLayerButton_clicked(bool checked);

    void on_initMapButton_clicked();

    void on_satDriveMapLayerButton_clicked(bool checked);

    void on_refresh_layers_button_clicked();

private:
    QThread rosWorkerThread;
    boost::shared_ptr<ros_workers> rosWorker;
    boost::shared_ptr<ros::NodeHandle> nh;

    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    boost::shared_array<boost::scoped_ptr<QGraphicsSceneMapViewer> > scene;
};

#endif // MAP_VIEWER_H
