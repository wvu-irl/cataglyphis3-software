#include "cataglyphis_gui.h"
#include "ui_cataglyphis_gui.h"

cataglyphis_gui::cataglyphis_gui(QWidget *parent, boost::shared_ptr<ros::NodeHandle> nh) :
    QMainWindow(parent),
    ui(new Ui::cataglyphis_gui)
{
    ROS_DEBUG("Cataglyphis_GUI:: Core app init");
    ui->setupUi(this);



    cataglyphis_gui::gui_nh = nh;
    ROS_DEBUG("Cataglypis_GUI:: Starting %d callback theads", NUM_MSG_CALLBACK_THREADS);
    cataglyphis_gui::async_spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(NUM_MSG_CALLBACK_THREADS));
    if(async_spinner->canStart())
    {
        async_spinner->start();
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


    cataglyphis_gui::cataglyphis_startup_form =
            boost::shared_ptr<cataglyphis_startup_form_main>
                            (new cataglyphis_startup_form_main(ui->guiTabber,
                                                                    nh));

    cataglyphis_gui::map_view_form =
            boost::shared_ptr<map_viewer>(new map_viewer(ui->guiTabber, 0));

    ui->guiTabber->addTab(cataglyphis_gui::cataglyphis_startup_form.get(), "Startup");
    ui->guiTabber->addTab(cataglyphis_gui::map_view_form.get(), "Map");
}

cataglyphis_gui::~cataglyphis_gui()
{
    //delete ui;
    cataglyphis_gui::async_spinner->stop();
    //ui.reset();
}




