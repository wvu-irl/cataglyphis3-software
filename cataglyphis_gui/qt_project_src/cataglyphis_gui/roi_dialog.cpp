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

ROI_dialog::~ROI_dialog()
{
    delete ui;
}

void ROI_dialog::on_edit_roi_button_clicked(bool checked)
{

}
