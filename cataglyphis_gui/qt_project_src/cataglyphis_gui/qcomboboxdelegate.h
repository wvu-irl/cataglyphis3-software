#ifndef QCOMBOBOXDELEGATE_H
#define QCOMBOBOXDELEGATE_H

#include <ros/ros.h>
#include <iostream>
#include <QObject>
#include <QStyledItemDelegate>
#include <QComboBox>
#include <QBrush>
#include <mission_planning_enums.h>

class ComboBoxDelegate : public QStyledItemDelegate
{
public:
    ComboBoxDelegate(QObject *parent = 0): QStyledItemDelegate(parent){std::printf("delegate\r\n");}
    ComboBoxDelegate(QList<QString> comboOpts,
                        QObject *parent = 0):
        QStyledItemDelegate(parent),
        comboOptions(comboOpts)
    {std::printf("list delegate\r\n");}

    QWidget *createEditor(QWidget *parent,
        const QStyleOptionViewItem &/* option */,
        const QModelIndex &/* index */) const;

    void setEditorData(QWidget *editor,
                        const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                                       const QModelIndex &index) const;
    void updateEditorGeometry(QWidget *editor,
        const QStyleOptionViewItem &option, const QModelIndex &index) const;
private:
    QList<QString> comboOptions;
};

#endif // QCOMBOBOXDELEGATE_H
