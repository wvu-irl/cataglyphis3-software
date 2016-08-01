#ifndef MAP_VIEWER_H
#define MAP_VIEWER_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <map_view_roi_ellipse.h>
#include <generic_error_dialog.h>
#include <roi_dialog.h>
#include <QImage>
#include <boost/smart_ptr.hpp>
#include <stdio.h>

namespace Ui {
class map_viewer_form;
}

class map_viewer : public QWidget
{
    Q_OBJECT

public:
    explicit map_viewer(QWidget *parent = 0, int startIndex = 0);
    ~map_viewer();

    //boost::shared_ptr<Ui::map_viewer> ui;
    Ui::map_viewer_form* ui;

private slots:
    void on_fieldSelector_currentIndexChanged(int index);

    void on_pushButton_clicked();

private:
    boost::shared_ptr<QGraphicsView> view;                      //graphics view holds a scene
    const QColor defaultCircleFill;
    const QColor fullTransparentColor;
    boost::shared_array<boost::shared_ptr<QGraphicsScene> > scene;
    boost::shared_array<boost::shared_ptr<QImage> > fieldPictures;
    boost::shared_ptr<QGraphicsRectItem> cataglyphisRect;
};

#endif // MAP_VIEWER_H
