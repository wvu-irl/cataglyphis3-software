/********************************************************************************
** Form generated from reading UI file 'generic_error_dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GENERIC_ERROR_DIALOG_H
#define UI_GENERIC_ERROR_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_generic_error_diaglog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *dialogSubMessage;
    QLabel *dialogMessage;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *generic_error_diaglog)
    {
        if (generic_error_diaglog->objectName().isEmpty())
            generic_error_diaglog->setObjectName(QStringLiteral("generic_error_diaglog"));
        generic_error_diaglog->resize(400, 269);
        verticalLayout = new QVBoxLayout(generic_error_diaglog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(generic_error_diaglog);
        label->setObjectName(QStringLiteral("label"));
        QFont font;
        font.setPointSize(32);
        label->setFont(font);

        verticalLayout->addWidget(label);

        dialogSubMessage = new QLabel(generic_error_diaglog);
        dialogSubMessage->setObjectName(QStringLiteral("dialogSubMessage"));
        QFont font1;
        font1.setPointSize(16);
        dialogSubMessage->setFont(font1);

        verticalLayout->addWidget(dialogSubMessage);

        dialogMessage = new QLabel(generic_error_diaglog);
        dialogMessage->setObjectName(QStringLiteral("dialogMessage"));
        QFont font2;
        font2.setPointSize(12);
        dialogMessage->setFont(font2);

        verticalLayout->addWidget(dialogMessage);

        buttonBox = new QDialogButtonBox(generic_error_diaglog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);
        buttonBox->setCenterButtons(true);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(generic_error_diaglog);
        QObject::connect(buttonBox, SIGNAL(rejected()), generic_error_diaglog, SLOT(reject()));
        QObject::connect(buttonBox, SIGNAL(accepted()), generic_error_diaglog, SLOT(accept()));

        QMetaObject::connectSlotsByName(generic_error_diaglog);
    } // setupUi

    void retranslateUi(QDialog *generic_error_diaglog)
    {
        generic_error_diaglog->setWindowTitle(QApplication::translate("generic_error_diaglog", "Dialog", 0));
        label->setText(QApplication::translate("generic_error_diaglog", "ERROR", 0));
        dialogSubMessage->setText(QApplication::translate("generic_error_diaglog", "Sub Error", 0));
        dialogMessage->setText(QApplication::translate("generic_error_diaglog", "Sub Sub Error", 0));
    } // retranslateUi

};

namespace Ui {
    class generic_error_diaglog: public Ui_generic_error_diaglog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GENERIC_ERROR_DIALOG_H
