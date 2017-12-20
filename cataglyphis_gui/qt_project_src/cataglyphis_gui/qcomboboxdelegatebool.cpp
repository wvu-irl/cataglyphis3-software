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
