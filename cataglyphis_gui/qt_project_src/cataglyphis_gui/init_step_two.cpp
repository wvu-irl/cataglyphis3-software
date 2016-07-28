#include "init_step_two.h"
#include "ui_init_step_two_form.h"

init_step_two::init_step_two(QWidget *parent, boost::shared_ptr<ros_workers> workerArg) :
    QWidget(parent),
    ui(new Ui::init_step_two_form)
{
    ui->setupUi(this);
    worker = workerArg;
    previousDeadReckButtonEnabled = false;

    connect(this, &init_step_two::startBiasRemoval,
                worker.get(), &ros_workers::runBiasRemovalService);
    connect(this, &init_step_two::startDeadReckoning,
                worker.get(), &ros_workers::runStartDeadReckoningService);

    connect(this, &init_step_two::startNavInfoSubscriber,
                worker.get(), &ros_workers::runNavInfoSubscriberStart);
    connect(this, &init_step_two::stopNavInfoSubscriber,
                worker.get(), &ros_workers::runNavInfoSubscriberStop);

    connect(worker.get(), &ros_workers::biasRemovalReturned,
                this, &init_step_two::onUpdateBiasRemovalDisplay);
    connect(worker.get(), &ros_workers::deadReckoningServiceReturned,
                this, &init_step_two::onUpdateBiasRemovalDisplay);

    connect(worker.get(), &ros_workers::navInfoCallback,
                this, &init_step_two::onNavInfoCallback);
}

init_step_two::~init_step_two()
{
    delete ui;
}

void init_step_two::onBeginDeadReckoningButtonClicked()
{
    ROS_DEBUG("bias_removal_form:: bias removal finished, starting dead reckoning");
    //send service
    emit startDeadReckoning();
    emit biasRemovalFinished();
    emit stopNavInfoSubscriber();
}

void init_step_two::onPerformBiasRemovalButtonClicked()
{
    ROS_DEBUG("bias_removal_form:: starting bias removal");
    //when min and max of a progress bar are the same, it simply sits there and moves
    ui->progressBar->setMaximum(0);
    emit startBiasRemoval();
    emit startNavInfoSubscriber();
}

void init_step_two::onUpdateBiasRemovalDisplay(messages::NavFilterControl serviceResponse,
                                                        bool wasSucessful)
{
    ROS_DEBUG("bias_removal_form:: Updating Bias Removal Display");
    navServiceResponse = serviceResponse;
    ui->p1_offset_spinbox->setValue(navServiceResponse.response.p1Offset);
    ui->q1_offset_spinbox->setValue(navServiceResponse.response.q1Offset);
    ui->r1_offset_spinbox->setValue(navServiceResponse.response.r1Offset);

    ui->p2_offset_spinbox->setValue(navServiceResponse.response.p2Offset);
    ui->q2_offset_spinbox->setValue(navServiceResponse.response.q2Offset);
    ui->r2_offset_spinbox->setValue(navServiceResponse.response.r2Offset);

    ui->p3_offset_spinbox->setValue(navServiceResponse.response.p3Offset);
    ui->q3_offset_spinbox->setValue(navServiceResponse.response.q3Offset);
    ui->r3_offset_spinbox->setValue(navServiceResponse.response.r3Offset);
    ui->progressBar->setMaximum(100);

    if(wasSucessful)
    {
        ui->begin_dead_reckoning_button->setDisabled(false);
    }
}

void init_step_two::onNavInfoCallback(const messages::NavFilterOut navInfo)
{
    ROS_DEBUG("bias_removal_form:: nav_info_callback");
    ui->p1_offset_spinbox->setValue(navInfo.p1_offset);
    ui->q1_offset_spinbox->setValue(navInfo.q1_offset);
    ui->r1_offset_spinbox->setValue(navInfo.r1_offset);

    ui->p2_offset_spinbox->setValue(navInfo.p2_offset);
    ui->q2_offset_spinbox->setValue(navInfo.q2_offset);
    ui->r2_offset_spinbox->setValue(navInfo.r2_offset);

    ui->p3_offset_spinbox->setValue(navInfo.p3_offset);
    ui->q3_offset_spinbox->setValue(navInfo.q3_offset);
    ui->r3_offset_spinbox->setValue(navInfo.r3_offset);
}

void init_step_two::keyPressEvent(QKeyEvent *event)
{
    ROS_DEBUG("bias_removal_form:: Key Pressed");
    if((event->modifiers() & Qt::ControlModifier) == Qt::ControlModifier)
    {
        ROS_DEBUG("bias_removal_form:: Control pressed");
        previousDeadReckButtonEnabled = ui->begin_dead_reckoning_button->isEnabled();
        ui->begin_dead_reckoning_button->setEnabled(true);
    }

}

void init_step_two::keyReleaseEvent(QKeyEvent *event)
{
    if((event->modifiers() & Qt::ControlModifier) != Qt::ControlModifier)
    {
        ROS_DEBUG("bias_removal_form:: Control released");
        ui->begin_dead_reckoning_button->setEnabled(previousDeadReckButtonEnabled);
    }
}
