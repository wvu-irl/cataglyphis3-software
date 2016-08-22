#include "core_app.h"
#include "ui_core_app_form.h"

core_app::core_app(QWidget *parent, boost::shared_ptr<ros::NodeHandle> nh) :
    QMainWindow(parent),
    ui(new Ui::core_app_form)
{
    ROS_DEBUG("Cataglyphis_GUI:: Core app init");
    ui->setupUi(this);

    guiNhPtr = nh;
    ROS_DEBUG("Cataglypis_GUI:: Starting %d callback theads", NUM_MSG_CALLBACK_THREADS);
    asyncSpinnerPtr = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(NUM_MSG_CALLBACK_THREADS));
    if(asyncSpinnerPtr->canStart())
    {
        asyncSpinnerPtr->start();
    }
    else
    {
        ROS_INFO("Cataglypis_GUI:: Cannot start callback threads. Is another GUI instance running?");
    }

    if(!nh->ok())
    {
        ROS_ERROR("Node Invalid, Cannot connect to ROS Master");
    }

    //set title bar to a recongizable name
    //sharedMenuBar = boost::shared_ptr<QMenuBar>(new QMenuBar(0));
#ifdef STATIC_BUILD
    setWindowTitle(tr("Cataglyphis GUI - STATIC"));
#elif TEST_RELEASE_BUILD
    setWindowTitle(tr("Cataglyphis GUI - TEST_RELEASE"));
#elif DEBUG_BUILD
    setWindowTitle(tr("Cataglyphis GUI - DEBUG"));
#else
    setWindowTitle(tr("Cataglyphis GUI - UNKOWN_BUILD"));
#endif


    cataglyphisStartupFormPtr.reset(new init_container(ui->guiTabber, nh));
    mapViewFormPtr.reset(new map_viewer(ui->guiTabber, 0, nh));
    manualControlFormPtr.reset(new manual_control(ui->guiTabber));
    execInfoFormPtr.reset(new exec_info_queue(ui->guiTabber));

    ui->guiTabber->addTab(cataglyphisStartupFormPtr.get(), "Startup");
    ui->guiTabber->addTab(mapViewFormPtr.get(), "Map");
    ui->guiTabber->addTab(manualControlFormPtr.get(), "Manual Control");
    ui->guiTabber->addTab(execInfoFormPtr.get(), "Exec Queue");
}

core_app::~core_app()
{
    asyncSpinnerPtr->stop();
    delete ui;
    //ui.reset();
}




