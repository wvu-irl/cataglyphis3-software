#include "qcomboboxdelegatebool.h"

#include "qcomboboxdelegate.h"

QWidget *qcomboboxdelegatebool::createEditor(QWidget *parent,
    const QStyleOptionViewItem &/* option */,
    const QModelIndex &/* index */) const
{
    ROS_DEBUG("Ading editor");
    std::cout << "Adding editor \r\n";
    QComboBox *editor = new QComboBox(parent);
    editor->setEditable(false);
    editor->addItem("False");
    editor->addItem("True");
    editor->setFrame(true);
    editor->setCurrentIndex(0);
    return editor;
}

void qcomboboxdelegatebool::setEditorData(QWidget *editor,
                    const QModelIndex &index) const
{
//    QString value = index.model()->data(index, Qt::EditRole).toString();

    QComboBox *comboBox = static_cast<QComboBox*>(editor);
//    std::printf("SetEditorData::str %s\r\n",index.data(Qt::EditRole).toString().toStdString().c_str());
//    std::printf("SetEditorData::int %d\r\n",index.data(Qt::EditRole+1).toInt());
    //QString currentText = index.data(Qt::EditRole).toString();
    comboBox->setCurrentIndex(index.data(Qt::EditRole+1).toInt());
}

void qcomboboxdelegatebool::setModelData(QWidget *editor, QAbstractItemModel *model,
                                   const QModelIndex &index) const
{
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
//    std::printf("setModelData:: %d\r\n",index.data(Qt::EditRole+1).toInt());
    model->setData(index, comboBox->currentText(), Qt::EditRole);
    model->setData(index, comboBox->currentIndex(), Qt::EditRole+1);
}

void qcomboboxdelegatebool::updateEditorGeometry(QWidget *editor,
    const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}


