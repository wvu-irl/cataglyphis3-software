#ifndef SHIFT_MAP_H
#define SHIFT_MAP_H

#include <QWidget>
#include <boost/scoped_ptr.hpp>
#include <ros_workers.h>
#include <messages/SetStartingPlatform.h>

namespace Ui {
class shift_map_form;
}

class shift_map : public QWidget
{
    Q_OBJECT

signals:
    void set_starting_platform(messages::SetStartingPlatform);

public slots:
    void on_map_manager_start_platform_set_returned(messages::SetStartingPlatform response, bool wasSuccessful);

public:
    explicit shift_map(QWidget *parent = 0);
    ~shift_map();

private slots:
    void on_submit_map_shift_button_clicked();

    void on_reset_platform_adjustment_button_clicked();

private:
    Ui::shift_map_form *ui;

    ros_workers rosWorker;

    boost::scoped_ptr<messages::SetStartingPlatform> startingPlatformServiceRequestPtr;
};

#endif // SHIFT_MAP_H
