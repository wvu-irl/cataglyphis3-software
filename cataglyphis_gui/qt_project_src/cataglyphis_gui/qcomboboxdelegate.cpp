/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
