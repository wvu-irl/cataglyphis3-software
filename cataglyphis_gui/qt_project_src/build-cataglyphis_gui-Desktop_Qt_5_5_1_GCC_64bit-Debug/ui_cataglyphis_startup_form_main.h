/********************************************************************************
** Form generated from reading UI file 'cataglyphis_startup_form_main.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CATAGLYPHIS_STARTUP_FORM_MAIN_H
#define UI_CATAGLYPHIS_STARTUP_FORM_MAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cataglyphis_startup_form_main
{
public:
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton;

    void setupUi(QWidget *cataglyphis_startup_form_main)
    {
        if (cataglyphis_startup_form_main->objectName().isEmpty())
            cataglyphis_startup_form_main->setObjectName(QStringLiteral("cataglyphis_startup_form_main"));
        cataglyphis_startup_form_main->resize(528, 300);
        horizontalLayout = new QHBoxLayout(cataglyphis_startup_form_main);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        pushButton = new QPushButton(cataglyphis_startup_form_main);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        horizontalLayout->addWidget(pushButton, 0, Qt::AlignHCenter|Qt::AlignTop);


        retranslateUi(cataglyphis_startup_form_main);

        QMetaObject::connectSlotsByName(cataglyphis_startup_form_main);
    } // setupUi

    void retranslateUi(QWidget *cataglyphis_startup_form_main)
    {
        cataglyphis_startup_form_main->setWindowTitle(QApplication::translate("cataglyphis_startup_form_main", "Form", 0));
        pushButton->setText(QApplication::translate("cataglyphis_startup_form_main", "Press for StartUp Procedure", 0));
    } // retranslateUi

};

namespace Ui {
    class cataglyphis_startup_form_main: public Ui_cataglyphis_startup_form_main {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATAGLYPHIS_STARTUP_FORM_MAIN_H
