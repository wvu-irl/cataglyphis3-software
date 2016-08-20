#ifndef NODE_STATUS_H
#define NODE_STATUS_H

#include <QWidget>

namespace Ui {
class node_status_form;
}

class node_status : public QWidget
{
    Q_OBJECT

public:
    explicit node_status(QWidget *parent = 0);
    ~node_status();

private:
    Ui::node_status_form *ui;
};

#endif // NODE_STATUS_H
