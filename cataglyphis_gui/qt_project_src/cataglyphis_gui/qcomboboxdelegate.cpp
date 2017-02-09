#include "qcomboboxdelegate.h"

#include <QGuiApplication>

QWidget *ComboBoxDelegate::createEditor(QWidget *parent,
    const QStyleOptionViewItem &/* option */,
    const QModelIndex &/* index */) const
{
    ROS_DEBUG("Ading editor");
    //std::cout << "Adding editor \r\n";
    QComboBox *editor = new QComboBox(parent);
    editor->setEditable(false);
    for(int i = 0; i < comboOptions.length(); i++)
    {
        editor->addItem(comboOptions.at(i));
    }
    editor->setFrame(true);
    editor->setCurrentIndex(0);
    return editor;
}

void ComboBoxDelegate::setEditorData(QWidget *editor,
                    const QModelIndex &index) const
{
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    //std::printf("SeteditorData %d\r\n", index.data(QT_MISSION_DATA_ROLE).toInt());
    comboBox->setCurrentIndex(index.data(QT_MISSION_DATA_ROLE).toInt());
    if(comboBox->currentIndex()!=0)
    {
//        std::printf("SeteditorData2 %d\r\n", index.data(QT_READ_ONLY_ROLE).toBool());
//        std::printf("SeteditorData3 %d\r\n", index.data(QT_READ_ONLY_ROLE+1).toBool());
        if(!index.data(QT_READ_ONLY_ROLE).toBool() && !comboBox->itemData(0,QT_READ_ONLY_ROLE+1).toBool())
        {
            comboBox->setPalette(QPalette(Qt::red));
        }
        else
        {
            comboBox->setPalette( QPalette( Qt::blue ) );
        }
    }
    else
    {
        comboBox->setPalette( QGuiApplication::palette());
    }
    comboBox->setItemData(0,index.data(QT_READ_ONLY_ROLE).toBool(),QT_READ_ONLY_ROLE+1);
    comboBox->setAttribute(Qt::WA_TransparentForMouseEvents, index.data(QT_READ_ONLY_ROLE).toBool());
    comboBox->setFocusPolicy(index.data(QT_READ_ONLY_ROLE).toBool() ? Qt::NoFocus : Qt::StrongFocus);
}

void ComboBoxDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                   const QModelIndex &index) const
{
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    //std::printf("SetModelData %d\r\n", comboBox->currentIndex());
    model->setData(index, comboBox->currentIndex(), QT_MISSION_DATA_ROLE);
    //std::printf("SetModelData2 %d\r\n", comboBox->currentIndex());
    model->setData(index, comboBox->currentText(), Qt::EditRole);
}

void ComboBoxDelegate::updateEditorGeometry(QWidget *editor,
    const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}
