#ifndef ROI_DIALOG_H
#define ROI_DIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <qledindicator.h>
#include <boost/scoped_ptr.hpp>

#include <robot_control/ROI.h>
#include <robot_control/ModifyROI.h>

#include <generic_ack_dialog.h>

#include <ros_workers.h>

namespace Ui {
class ROI_dialog;
}

class ROI_dialog : public QDialog
{
    Q_OBJECT

signals:
    void update_roi_ellipse(robot_control::ROI roiData);
    void modify_roi(robot_control::ModifyROI service);

public slots:
    void accept();
    void reject();
    void open();
    void on_confirm_changes();
    void on_discard_changes();

public:
    explicit ROI_dialog(QWidget *parent = 0);
    explicit ROI_dialog(robot_control::ROI data, int roiNum,
                            boost::shared_ptr<ros_workers> workerArg = boost::shared_ptr<ros_workers>());
    ~ROI_dialog();

    bool isModified(){return modified;}
    robot_control::ROI internalROI();

private slots:
    void on_edit_roi_button_clicked(bool checked);

    void on_reset_roi_button_clicked();

private:
    Ui::ROI_dialog *ui;
    boost::shared_ptr<ros_workers> worker;

    robot_control::ROI * _implCurrentROISet();
    void _implMsgToUi(robot_control::ROI *data);
    void _implUiToMsg(robot_control::ROI *data);
    void _implSetButtonReadOnly(bool readOnly);

    bool modified;
    int roiNumber;
    robot_control::ROI ROIData;
    robot_control::ROI stagedROIData;
};

#endif // ROI_DIALOG_H
