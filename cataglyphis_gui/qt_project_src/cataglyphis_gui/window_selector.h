#ifndef WINDOW_SELECTOR_H
#define WINDOW_SELECTOR_H

#include <QDockWidget>

namespace Ui {
class window_selector;
}

class window_selector : public QDockWidget
{
    Q_OBJECT

public:
    explicit window_selector(QWidget *parent = 0);
    ~window_selector();

    Ui::window_selector *ui;
private:

};

#endif // WINDOW_SELECTOR_H
