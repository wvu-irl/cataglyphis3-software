#include "init_container.h"
#include "ui_init_container_form.h"
#include <QLabel>

init_container::init_container(QWidget *parent,
                                    boost::shared_ptr<ros::NodeHandle> nhArg) :
    QWidget(parent),
    ui(new Ui::init_container_form)
{
    ui->setupUi(this);

    qRegisterMetaType<messages::NavFilterControl>("messages::NavFilterControl");

    if(nhArg.get() != 0)
    {
        nh = nhArg;
    }
    else
    {
        ROS_WARN("Startup Form:: Input Nodehandle is Null!");

    }

    rosWorker = boost::shared_ptr<ros_workers>(new ros_workers(nhArg));
    rosWorker->moveToThread(&rosWorkerThread);

    //no need to schedule deletion of rosWorker as the boost pointer automatically does
    //connect(&rosWorkerThread, &QThread::finished, rosWorker.get(), &QObject::deleteLater);

    stepOneTab = boost::shared_ptr<init_step_one>(new init_step_one(ui->input_tabber, rosWorker));
    stepOneTab->hide();
    stepTwoTab = boost::shared_ptr<init_step_two>(new init_step_two(ui->input_tabber, rosWorker));
    stepTwoTab->hide();
    connect(stepOneTab.get(), &init_step_one::step_one_finished,
                this, &init_container::on_step_one_returned);
    connect(stepTwoTab.get(), &init_step_two::bias_removal_finished,
                this, &init_container::on_step_two_returned);

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

    //even more defunct, using QThreads now which have signals and slots

    //because run_bias_removal_thread is a class member of an object, it is unresolved (non existent)
    //until an object of the class is made, this is unknown at compile time.
    //a class reference is bound to the function and used to create a functor with boost::bind,
    //this is passed into a boost::thread constructor and the bias_removal_thread is started.
//    boost::thread biasRemovalThread(boost::bind(
//                                        &cataglyphis_startup_form_main::run_bias_removal_thread,
//                                        this)
//                                   );
//    //ROS_DEBUG("Starting boost::thread %s", biasRemovalThread.get_id());
//    biasRemovalThread.detach();

    //emit start_bias_removal();

    ui->input_tabber->addTab(stepOneTab.get(), "Nav Init");
    ui->input_tabber->setCurrentWidget(stepOneTab.get());
}

void init_container::on_step_one_returned()
{
    ROS_DEBUG("Startup form:: step one finished");
    ui->input_tabber->addTab(stepTwoTab.get(), "Bias Removal");
    ui->input_tabber->setCurrentWidget(stepTwoTab.get());
}

void init_container::on_step_two_returned()
{
    ROS_DEBUG("Startup form:: step two finished");
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
