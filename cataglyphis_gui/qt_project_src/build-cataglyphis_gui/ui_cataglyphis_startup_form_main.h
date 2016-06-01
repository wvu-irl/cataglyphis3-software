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
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cataglyphis_startup_form_main
{
public:
    QVBoxLayout *verticalLayout_2;
    QFrame *widget_frame;
    QVBoxLayout *verticalLayout;
    QFrame *button_frame;
    QHBoxLayout *horizontalLayout;
    QPushButton *start_up_button;
    QPushButton *lost_recovery_button;
    QPushButton *reboot_recovery_button;
    QPushButton *teleport_button;
    QTabWidget *input_tabber;

    void setupUi(QWidget *cataglyphis_startup_form_main)
    {
        if (cataglyphis_startup_form_main->objectName().isEmpty())
            cataglyphis_startup_form_main->setObjectName(QStringLiteral("cataglyphis_startup_form_main"));
        cataglyphis_startup_form_main->resize(528, 300);
        verticalLayout_2 = new QVBoxLayout(cataglyphis_startup_form_main);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        widget_frame = new QFrame(cataglyphis_startup_form_main);
        widget_frame->setObjectName(QStringLiteral("widget_frame"));
        widget_frame->setFrameShape(QFrame::StyledPanel);
        widget_frame->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(widget_frame);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        button_frame = new QFrame(widget_frame);
        button_frame->setObjectName(QStringLiteral("button_frame"));
        button_frame->setFrameShape(QFrame::StyledPanel);
        button_frame->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(button_frame);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        start_up_button = new QPushButton(button_frame);
        start_up_button->setObjectName(QStringLiteral("start_up_button"));
        start_up_button->setMaximumSize(QSize(132, 16));

        horizontalLayout->addWidget(start_up_button);

        lost_recovery_button = new QPushButton(button_frame);
        lost_recovery_button->setObjectName(QStringLiteral("lost_recovery_button"));
        lost_recovery_button->setMaximumSize(QSize(132, 16));

        horizontalLayout->addWidget(lost_recovery_button);

        reboot_recovery_button = new QPushButton(button_frame);
        reboot_recovery_button->setObjectName(QStringLiteral("reboot_recovery_button"));
        reboot_recovery_button->setMaximumSize(QSize(132, 16));

        horizontalLayout->addWidget(reboot_recovery_button);

        teleport_button = new QPushButton(button_frame);
        teleport_button->setObjectName(QStringLiteral("teleport_button"));
        teleport_button->setMaximumSize(QSize(132, 16));

        horizontalLayout->addWidget(teleport_button);


        verticalLayout->addWidget(button_frame);

        input_tabber = new QTabWidget(widget_frame);
        input_tabber->setObjectName(QStringLiteral("input_tabber"));

        verticalLayout->addWidget(input_tabber);


        verticalLayout_2->addWidget(widget_frame);


        retranslateUi(cataglyphis_startup_form_main);

        input_tabber->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(cataglyphis_startup_form_main);
    } // setupUi

    void retranslateUi(QWidget *cataglyphis_startup_form_main)
    {
        cataglyphis_startup_form_main->setWindowTitle(QApplication::translate("cataglyphis_startup_form_main", "Form", 0));
        start_up_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Press to run StartUp Procedure", 0));
        lost_recovery_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Press to run Lost Recovery", 0));
        reboot_recovery_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Press for Reboot Recovery", 0));
        teleport_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Press to Teleport", 0));
    } // retranslateUi

};

namespace Ui {
    class cataglyphis_startup_form_main: public Ui_cataglyphis_startup_form_main {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATAGLYPHIS_STARTUP_FORM_MAIN_H
