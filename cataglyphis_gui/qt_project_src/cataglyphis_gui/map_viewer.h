#ifndef MAP_VIEWER_H
#define MAP_VIEWER_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
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

private:
    boost::shared_ptr<QGraphicsScene> scene;
    boost::shared_ptr<QGraphicsView> view;
    boost::shared_ptr<QGraphicsPixmapItem> item;
    boost::shared_array<boost::shared_ptr<QImage> > field_pic;
    boost::shared_ptr<QGraphicsRectItem> cataglyphisRect;

};

#endif // MAP_VIEWER_H
