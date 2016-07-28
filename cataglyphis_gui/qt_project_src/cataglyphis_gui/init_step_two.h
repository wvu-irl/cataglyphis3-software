#ifndef BIAS_REMOVAL_FORM_H
#define BIAS_REMOVAL_FORM_H

#include <QWidget>
#include <QKeyEvent>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <messages/NavFilterOut.h>
#include <messages/NavFilterControl.h>

#include <ros_workers.h>

#define INIT_STEP2_ID 2

namespace Ui {
class init_step_two_form;
}

class init_step_two : public QWidget
{
    Q_OBJECT

signals:
    void startBiasRemoval();
    void startDeadReckoning();
    void biasRemovalFinished();
    void startNavInfoSubscriber();
    void stopNavInfoSubscriber();

public:
    explicit init_step_two(QWidget *parent = 0, boost::shared_ptr<ros_workers> workerArg =
                                                            boost::shared_ptr<ros_workers>());
    ~init_step_two();

    Ui::init_step_two_form *ui;
    //boost::shared_ptr<Ui::bias_removal_form> ui;

public slots:

    void onUpdateBiasRemovalDisplay(messages::NavFilterControl serviceResponse,
                                        bool wasSucessful);
    void onNavInfoCallback(const messages::NavFilterOut navInfo);

private slots:
    void onBeginDeadReckoningButtonClicked();

    void onPerformBiasRemovalButtonClicked();

private:

    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<ros_workers> worker;
    messages::NavFilterControl navServiceResponse;

    bool previousDeadReckButtonEnabled;

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);


};

#endif // BIAS_REMOVAL_FORM_H
