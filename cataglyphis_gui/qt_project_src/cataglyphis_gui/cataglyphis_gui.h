#ifndef CATAGLYPHIS_GUI_H
#define CATAGLYPHIS_GUI_H

#include <QMainWindow>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>

namespace Ui {
class Cataglyphis_Gui;
}

class Cataglyphis_Gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit Cataglyphis_Gui(QWidget *parent = 0);
    ~Cataglyphis_Gui();

private slots:
    void on_fieldSelector_currentIndexChanged(int index);

private:
    Ui::Cataglyphis_Gui *ui;
    QGraphicsScene* scene;
    //QGraphicsView* view;
    QGraphicsPixmapItem* item;
    QImage* field_pic[4];
    QGraphicsRectItem* cataglyphisRect;
};

#endif // CATAGLYPHIS_GUI_H
