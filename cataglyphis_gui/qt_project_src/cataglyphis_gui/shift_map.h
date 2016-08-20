#ifndef SHIFT_MAP_H
#define SHIFT_MAP_H

#include <QWidget>

namespace Ui {
class shift_map_form;
}

class shift_map : public QWidget
{
    Q_OBJECT

public:
    explicit shift_map(QWidget *parent = 0);
    ~shift_map();

private:
    Ui::shift_map_form *ui;
};

#endif // SHIFT_MAP_H
