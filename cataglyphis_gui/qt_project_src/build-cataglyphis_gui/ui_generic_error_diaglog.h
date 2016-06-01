/********************************************************************************
** Form generated from reading UI file 'generic_error_diaglog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GENERIC_ERROR_DIAGLOG_H
#define UI_GENERIC_ERROR_DIAGLOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_generic_error_diaglog
{
public:
    QDialogButtonBox *buttonBox;
    QFrame *frame;

    void setupUi(QDialog *generic_error_diaglog)
    {
        if (generic_error_diaglog->objectName().isEmpty())
            generic_error_diaglog->setObjectName(QStringLiteral("generic_error_diaglog"));
        generic_error_diaglog->resize(400, 300);
        buttonBox = new QDialogButtonBox(generic_error_diaglog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(30, 240, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        frame = new QFrame(generic_error_diaglog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 10, 381, 231));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        retranslateUi(generic_error_diaglog);
        QObject::connect(buttonBox, SIGNAL(accepted()), generic_error_diaglog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), generic_error_diaglog, SLOT(reject()));

        QMetaObject::connectSlotsByName(generic_error_diaglog);
    } // setupUi

    void retranslateUi(QDialog *generic_error_diaglog)
    {
        generic_error_diaglog->setWindowTitle(QApplication::translate("generic_error_diaglog", "Dialog", 0));
    } // retranslateUi

};

namespace Ui {
    class generic_error_diaglog: public Ui_generic_error_diaglog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GENERIC_ERROR_DIAGLOG_H
