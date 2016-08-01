#ifndef ROI_DIALOG_H
#define ROI_DIALOG_H

#include <QDialog>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <boost/scoped_ptr.hpp>

namespace Ui {
class ROI_dialog;
}

class ROI_dialog : public QDialog
{
    Q_OBJECT

public:
    explicit ROI_dialog(QWidget *parent = 0);
    ~ROI_dialog();

private:
    Ui::ROI_dialog *ui;

};

#endif // ROI_DIALOG_H
