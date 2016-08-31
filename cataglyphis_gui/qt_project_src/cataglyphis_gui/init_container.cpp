#include "init_container.h"
#include "ui_init_container_form.h"
#include <QLabel>

init_container::init_container(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::init_container_form)
{
    ui->setupUi(this);

    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers());
    rosWorker->moveToThread(&rosWorkerThread);

    //no need to schedule deletion of rosWorker as the boost pointer automatically does
    //connect(&rosWorkerThread, &QThread::finished, rosWorker.get(), &QObject::deleteLater);

    rosWorkerThread.start();
}

init_container::~init_container()
{
    ui->input_tabber->clear();
    rosWorkerThread.quit();
    rosWorkerThread.wait();
    std::printf("Init container destructor\r\n");
    delete ui;
}

void init_container::on_start_up_button_clicked()
{
    ROS_DEBUG("Startup Form:: Startup Button Clicked");

    _implResetTabberIfNeccesary();

    procedureInProgress = true;

    northAngleTab = boost::shared_ptr<init_step_one>(new init_step_one(ui->input_tabber, rosWorker));
    northAngleTab->hide();
    biasRemovalTab = boost::shared_ptr<init_step_two>(new init_step_two(ui->input_tabber, rosWorker));
    biasRemovalTab->hide();
    shiftMapTab.reset(new shift_map(ui->input_tabber));
    connect(northAngleTab.get(), &init_step_one::step_one_finished,
                this, &init_container::on_step_one_returned);
    connect(northAngleTab.get(), &init_step_one::procedure_finished,
                this, &init_container::on_procedure_returned);
    connect(biasRemovalTab.get(), &init_step_two::bias_removal_finished,
                this, &init_container::on_step_two_returned);
    ui->input_tabber->addTab(shiftMapTab.get(), "Shift Map");
    ui->input_tabber->addTab(northAngleTab.get(), "Nav Init");
    //ui->input_tabber->setCurrentWidget(northAngleTab.get());
    ui->input_tabber->setCurrentWidget(shiftMapTab.get());
}

void init_container::on_step_one_returned()
{
    ROS_DEBUG("Startup form:: step one finished");
    ui->input_tabber->addTab(biasRemovalTab.get(), "Bias Removal");
    ui->input_tabber->setCurrentWidget(biasRemovalTab.get());
}

void init_container::on_step_two_returned()
{
    ROS_DEBUG("Startup form:: step two finished");
    northAngleTab.reset();
    biasRemovalTab.reset();
    on_procedure_returned();
}

void init_container::on_reboot_recovery_button_clicked()
{
    ROS_DEBUG("Startup Form:: reboot recovery clicked");
}

void init_container::on_input_tabber_currentChanged(int index)
{
    std::printf("CurrentIndex:: %p\r\n", ui);
    Q_CHECK_PTR(ui->input_tabber);
    if(index >= 0 &&
            ui != 0 &&
            ui->input_tabber != 0 )
    {
        if(ui->input_tabber->currentWidget() != 0)
        {
            ui->input_tabber->resize(ui->input_tabber->currentWidget()->size());
        }
    }
}

void init_container::on_shift_map_button_clicked()
{
    ROS_DEBUG("Shift Map Button Clicked");
    on_procedure_returned();
    procedureInProgress = true;
    shiftMapTab.reset(new shift_map(ui->input_tabber));
    northAngleTab.reset(new init_step_one(ui->input_tabber, rosWorker, false));
    ui->input_tabber->addTab(shiftMapTab.get(), "Shift Map");
    ui->input_tabber->addTab(northAngleTab.get(), "North Angle");
    ui->input_tabber->setCurrentWidget(shiftMapTab.get());
}

void init_container::on_teleport_button_clicked()
{
    ROS_DEBUG("Teleport button clicked");
    on_procedure_returned();
    procedureInProgress = true;
    teleportTab.reset(new teleport(ui->input_tabber));
    ui->input_tabber->addTab(teleportTab.get(), "Teleport");
    ui->input_tabber->setCurrentWidget(teleportTab.get());
}

void init_container::on_procedure_returned()
{
    //reset pointers to tabs
    shiftMapTab.reset();
    northAngleTab.reset();
    biasRemovalTab.reset();
    teleportTab.reset();
    ui->input_tabber->clear();
    procedureInProgress = false;
}

bool init_container::_implIsProcedureInProgress()
{
    //check if procedure is in progress
    //if true, call procedure returned
    return procedureInProgress;
}

void init_container::_implResetTabberIfNeccesary()
{
    if(_implIsProcedureInProgress())
    {
        on_procedure_returned();
    }
}


