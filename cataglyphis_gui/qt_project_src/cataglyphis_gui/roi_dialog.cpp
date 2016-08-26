#include "roi_dialog.h"
#include "ui_roi_dialog_form.h"

ROI_dialog::ROI_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ROI_dialog)
{
    ui->setupUi(this);
    this->setModal(false);

    QStandardItemModel *model = new QStandardItemModel(5,2,this);
    model->setHorizontalHeaderItem(0,new QStandardItem(QString("Sample\r\nType")));
    model->setHorizontalHeaderItem(1,new QStandardItem(QString("Probability")));

    ui->tableView->setModel(model);
}

ROI_dialog::ROI_dialog(robot_control::ROI data, int roiNum,
                        boost::shared_ptr<ros_workers> workerArg):
    QDialog(0),
    ui(new Ui::ROI_dialog)
{
    ui->setupUi(this);
    this->setModal(false);

    QStandardItemModel *model = new QStandardItemModel(5,2,this);
    model->setHorizontalHeaderItem(0,new QStandardItem(QString("Sample\r\nType")));
    model->setHorizontalHeaderItem(1,new QStandardItem(QString("Probability")));

    ui->tableView->setModel(model);
    worker = workerArg;
    ROIData = data;
    roiNumber = roiNum;
    stagedROIData = data;

    this->setWindowTitle(QString::fromStdString(std::string("ROI Number: ")) + QString::number(roiNum));

    on_discard_changes();

    connect(this, &ROI_dialog::modify_roi_request,
                worker.get(), &ros_workers::on_run_modify_roi);
    connect(this, &ROI_dialog::add_wait_to_exec,
                worker.get(), &ros_workers::on_add_pause_to_exec_queue);
}

ROI_dialog::~ROI_dialog()
{
    delete ui;
}

void ROI_dialog::open()
{
    ROS_DEBUG("Dialog:: Open");
    _implMsgToUi(_implCurrentROISet());
    show();
    activateWindow();
    raise();
    QDialog::open();
}
void ROI_dialog::accept()
{
    ROS_DEBUG("ROI Dialog:: Accepted");
    ROS_DEBUG("ROI Edited? %d", (int)isModified());
    _implUiToMsg(_implCurrentROISet());
    emit update_roi_ellipse(*_implCurrentROISet(), isModified());
    /*//on_confirm_changes(); //remove later*/
    QDialog::accept();
}

void ROI_dialog::reject()
{   generic_ack_dialog dialog("Discard Changes?");
    if(isModified() && (dialog.bringUpDialogModal() == QDialog::Accepted))
    {
        ROS_DEBUG("ROI Dialog:: Changes Rejected");
        on_discard_changes();
        emit update_roi_ellipse(*_implCurrentROISet(), isModified());
        QDialog::reject();
        return;
    }
    if(!isModified())
    {
        QDialog::reject();
        return;
    }
}

void ROI_dialog::on_confirm_changes()
{
    ROS_DEBUG("ROI Dialog:: Committing Changes");
    ROIData = *_implCurrentROISet();
    //call ros service here
    robot_control::ModifyROI modRoiMsg;
    modRoiMsg.request.modROIIndex = roiNumber;
    modRoiMsg.request.setHardLockoutROI = true;
    modRoiMsg.request.hardLockoutROIState = ROIData.hardLockout;
    modRoiMsg.request.setPosXY = true;
    modRoiMsg.request.x = ROIData.x;
    modRoiMsg.request.y = ROIData.y;
    modRoiMsg.request.setROISize = true;
    modRoiMsg.request.radialAxis = ROIData.radialAxis;
    modRoiMsg.request.tangentialAxis = ROIData.tangentialAxis;
    modRoiMsg.request.setSampleProps = true;
    modRoiMsg.request.sampleProb = ROIData.sampleProb;
    modRoiMsg.request.sampleSig = ROIData.sampleSig;
    modRoiMsg.request.whiteProb = ROIData.whiteProb;
    modRoiMsg.request.silverProb = ROIData.silverProb;
    modRoiMsg.request.blueOrPurpleProb = ROIData.blueOrPurpleProb;
    modRoiMsg.request.pinkProb = ROIData.pinkProb;
    modRoiMsg.request.redProb = ROIData.redProb;
    modRoiMsg.request.orangeProb = ROIData.orangeProb;
    modRoiMsg.request.yellowProb = ROIData.yellowProb;
    modRoiMsg.request.editGroup = ui->edit_roi_group_indicator->isChecked();
    emit modify_roi_request(modRoiMsg);

    if(ui->add_pause_action_indicator->isChecked())
    {
        emit add_wait_to_exec(3); //add 3 second wait to exec
    }
}
void ROI_dialog::on_discard_changes()
{
    modified = false;
    ui->edit_roi_button->setChecked(false);
    ui->edit_roi_group_indicator->setChecked(true);
    ui->add_pause_action_indicator->setChecked(true);
    emit update_roi_ellipse(*_implCurrentROISet(), isModified());
    _implSetButtonReadOnly(true);
    _implMsgToUi(_implCurrentROISet());
}

void ROI_dialog::on_edit_roi_button_clicked(bool checked)
{
    if(checked)
    {
        ROS_DEBUG("ROI Dialog:: Edit button clicked %d", (int)checked);
        modified = checked;
        _implSetButtonReadOnly(!checked);
    }
    ui->edit_roi_button->setChecked(true);
}

void ROI_dialog::on_reset_roi_button_clicked()
{
    on_discard_changes();
}

robot_control::ROI * ROI_dialog::_implCurrentROISet()
{
    if(isModified())
    {
        ROS_DEBUG("ROI Dialog:: Current data set is: staged");
        return &stagedROIData;
    }
    ROS_DEBUG("ROI Dialog:: Current data set is: original");
    return &ROIData;
}

void ROI_dialog::_implMsgToUi(robot_control::ROI *data)
{
    ui->allocated_time_spinbox->setValue(data->allocatedTime);
    ui->blue_purple_prob_spinbox->setValue(data->blueOrPurpleProb);
    ui->high_risk_indicator->setChecked(data->highRisk!=0);
    ui->lockout_indicator->setChecked(data->hardLockout);
    ui->orange_prob_spinbox->setValue(data->orangeProb);
    ui->pink_prob_spinbox->setValue(data->pinkProb);
    ui->red_prob_spinbox->setValue(data->redProb);
    ui->roi_radius->setValue(data->radialAxis);
    ui->sample_prob_spinbox->setValue(data->sampleProb);
    ui->sample_sig_spinbox->setValue(data->sampleSig);
    ui->silver_prob_spinbox->setValue(data->silverProb);
    ui->white_prob_spinbox->setValue(data->whiteProb);
    ui->x_spinbox->setValue(data->x);
    ui->yellow_prob_spinbox->setValue(data->yellowProb);
    ui->y_spinbox->setValue(data->y);
}

void ROI_dialog::_implUiToMsg(robot_control::ROI *data)
{
    data->allocatedTime=ui->allocated_time_spinbox->value();//data->allocatedTime);
    data->blueOrPurpleProb=ui->blue_purple_prob_spinbox->value();//data->blueOrPurpleProb);
    data->highRisk=(int)ui->high_risk_indicator->isChecked();
    data->hardLockout=ui->lockout_indicator->isChecked();
    data->orangeProb=ui->orange_prob_spinbox->value();//data->orangeProb);
    data->pinkProb=ui->pink_prob_spinbox->value();//data->pinkProb);
    data->redProb=ui->red_prob_spinbox->value();//data->redProb);
    data->radialAxis=ui->roi_radius->value();//data->radialAxis);
    data->tangentialAxis=ui->roi_radius->value();
    data->sampleProb=ui->sample_prob_spinbox->value();//data->sampleProb);
    data->sampleSig=ui->sample_sig_spinbox->value();//data->sampleSig);
    data->silverProb=ui->silver_prob_spinbox->value();//data->silverProb);
    data->whiteProb=ui->white_prob_spinbox->value();//data->whiteProb);
    data->x=ui->x_spinbox->value();//data->x);
    data->yellowProb=ui->yellow_prob_spinbox->value();//data->yellowProb);
    data->y=ui->y_spinbox->value();//data->y);
}

void ROI_dialog::_implSetButtonReadOnly(bool readOnly)
{
    ui->allocated_time_spinbox->setReadOnly(readOnly);//data->allocatedTime);
    ui->blue_purple_prob_spinbox->setReadOnly(readOnly);//data->blueOrPurpleProb);
    ui->edit_roi_group_indicator->setReadOnly(readOnly);
    ui->add_pause_action_indicator->setReadOnly(readOnly);
    ui->high_risk_indicator->setReadOnly(readOnly);
    ui->lockout_indicator->setReadOnly(readOnly);
    ui->orange_prob_spinbox->setReadOnly(readOnly);//data->orangeProb);
    ui->pink_prob_spinbox->setReadOnly(readOnly);//data->pinkProb);
    ui->red_prob_spinbox->setReadOnly(readOnly);//data->redProb);
    ui->roi_radius->setReadOnly(readOnly);//data->radialAxis);
    ui->sample_prob_spinbox->setReadOnly(readOnly);//data->sampleProb);
    ui->sample_sig_spinbox->setReadOnly(readOnly);//data->sampleSig);
    ui->silver_prob_spinbox->setReadOnly(readOnly);//data->silverProb);
    ui->white_prob_spinbox->setReadOnly(readOnly);//data->whiteProb);
    ui->x_spinbox->setReadOnly(readOnly);//data->x);
    ui->yellow_prob_spinbox->setReadOnly(readOnly);//data->yellowProb);
    ui->y_spinbox->setReadOnly(readOnly);//data->y);
}
