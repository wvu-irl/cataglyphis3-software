#ifndef QCOMBOBOXDELEGATEBOOL_H
#define QCOMBOBOXDELEGATEBOOL_H

#include <QObject>
#include <QStyledItemDelegate>

class qcomboboxdelegatebool : public QStyledItemDelegate
{
public:
    qcomboboxdelegatebool(QWidget *parent = 0):QStyledItemDelegate(parent){}

    QWidget *createEditor(QWidget *parent,
        const QStyleOptionViewItem &/* option */,
        const QModelIndex &/* index */) const;

    void setEditorData(QWidget *editor,
                        const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                                       const QModelIndex &index) const;
    void updateEditorGeometry(QWidget *editor,
        const QStyleOptionViewItem &option, const QModelIndex &index) const;
};

#endif // QCOMBOBOXDELEGATEBOOL_H
