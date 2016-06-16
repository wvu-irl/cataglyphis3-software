#ifndef BIAS_REMOVAL_FORM_H
#define BIAS_REMOVAL_FORM_H

#include <QWidget>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

#define INIT_STEP2_ID 2

namespace Ui {
class bias_removal_form;
}

class bias_removal_form : public QWidget
{
    Q_OBJECT

signals:
    void start_bias_removal();
    void start_dead_reckoning();
    void bias_removal_finished();

public:
    explicit bias_removal_form(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                        boost::shared_ptr<ros_workers>());
    ~bias_removal_form();

    //Ui::bias_removal_form *ui;
    boost::shared_ptr<Ui::bias_removal_form> ui;

public slots:

    void update_bias_removal_display(messages::NavFilterControl serviceResponse,
                                        bool wasSucessful);

private slots:
    void on_begin_dead_reckoning_button_clicked();

    void on_perform_bias_removal_button_clicked();

private:

    boost::shared_ptr<ros_workers> worker;
    messages::NavFilterControl navServiceResponse;

};

#endif // BIAS_REMOVAL_FORM_H
