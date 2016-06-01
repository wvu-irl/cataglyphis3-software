#include "cataglyphis_startup_form_main.h"
#include "ui_cataglyphis_startup_form_main.h"
#include <QLabel>

cataglyphis_startup_form_main::cataglyphis_startup_form_main(QWidget *parent,
                                                             boost::shared_ptr<ros::NodeHandle> nhArg) :
    QWidget(parent),
    ui(new Ui::cataglyphis_startup_form_main)
{
    ui->setupUi(this);

    qRegisterMetaType<messages::NavFilterControl>("messages::NavFilterControl");

    if(nhArg != 0)
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

    rosWorkerThread.start();
}

cataglyphis_startup_form_main::~cataglyphis_startup_form_main()
{
    rosWorkerThread.quit();
    rosWorkerThread.wait();
}

void cataglyphis_startup_form_main::on_start_up_button_clicked()
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

    stepOneTab = boost::shared_ptr<init_step_one>(new init_step_one(ui->input_tabber, rosWorker));

    stepTwoTab = boost::shared_ptr<bias_removal_form>(new bias_removal_form(ui->input_tabber, rosWorker));
    connect(stepOneTab.get(), &init_step_one::step_one_finished,
                this, &cataglyphis_startup_form_main::step_one_returned);
    connect(stepTwoTab.get(), &bias_removal_form::bias_removal_finished,
                this, &cataglyphis_startup_form_main::step_two_returned);
    ui->input_tabber->addTab(stepOneTab.get(), "Nav Init");
}

void cataglyphis_startup_form_main::step_one_returned()
{
    ROS_DEBUG("Startup form:: step one finished");
    ui->input_tabber->addTab(stepTwoTab.get(), "Bias Removal");
    ui->input_tabber->setCurrentWidget(stepTwoTab.get());
}

void cataglyphis_startup_form_main::step_two_returned()
{
    ROS_DEBUG("Startup form:: step two finished");
}

void cataglyphis_startup_form_main::on_reboot_recovery_button_clicked()
{
    ROS_DEBUG("Startup Form:: reboot recovery clicked");
}


