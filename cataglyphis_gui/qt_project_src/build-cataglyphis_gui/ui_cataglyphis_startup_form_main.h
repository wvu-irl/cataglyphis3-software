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
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cataglyphis_startup_form_main
{
public:
    QTabWidget *input_tabber;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer;
    QLabel *label;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *start_up_button;
    QPushButton *lost_recovery_button;
    QPushButton *reboot_recovery_button;
    QPushButton *teleport_button;

    void setupUi(QWidget *cataglyphis_startup_form_main)
    {
        if (cataglyphis_startup_form_main->objectName().isEmpty())
            cataglyphis_startup_form_main->setObjectName(QStringLiteral("cataglyphis_startup_form_main"));
        cataglyphis_startup_form_main->resize(563, 300);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(cataglyphis_startup_form_main->sizePolicy().hasHeightForWidth());
        cataglyphis_startup_form_main->setSizePolicy(sizePolicy);
        input_tabber = new QTabWidget(cataglyphis_startup_form_main);
        input_tabber->setObjectName(QStringLiteral("input_tabber"));
        input_tabber->setGeometry(QRect(10, 72, 544, 218));
        sizePolicy.setHeightForWidth(input_tabber->sizePolicy().hasHeightForWidth());
        input_tabber->setSizePolicy(sizePolicy);
        input_tabber->setMinimumSize(QSize(544, 218));
        input_tabber->setTabShape(QTabWidget::Rounded);
        widget = new QWidget(cataglyphis_startup_form_main);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 11, 546, 57));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setMinimumSize(QSize(91, 20));

        horizontalLayout_3->addWidget(label);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        start_up_button = new QPushButton(widget);
        start_up_button->setObjectName(QStringLiteral("start_up_button"));
        start_up_button->setMinimumSize(QSize(132, 21));

        horizontalLayout->addWidget(start_up_button);

        lost_recovery_button = new QPushButton(widget);
        lost_recovery_button->setObjectName(QStringLiteral("lost_recovery_button"));
        lost_recovery_button->setMinimumSize(QSize(130, 21));

        horizontalLayout->addWidget(lost_recovery_button);

        reboot_recovery_button = new QPushButton(widget);
        reboot_recovery_button->setObjectName(QStringLiteral("reboot_recovery_button"));
        reboot_recovery_button->setMinimumSize(QSize(129, 21));

        horizontalLayout->addWidget(reboot_recovery_button);

        teleport_button = new QPushButton(widget);
        teleport_button->setObjectName(QStringLiteral("teleport_button"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(teleport_button->sizePolicy().hasHeightForWidth());
        teleport_button->setSizePolicy(sizePolicy2);
        teleport_button->setMinimumSize(QSize(130, 21));

        horizontalLayout->addWidget(teleport_button);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(cataglyphis_startup_form_main);

        input_tabber->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(cataglyphis_startup_form_main);
    } // setupUi

    void retranslateUi(QWidget *cataglyphis_startup_form_main)
    {
        cataglyphis_startup_form_main->setWindowTitle(QApplication::translate("cataglyphis_startup_form_main", "Form", 0));
        label->setText(QApplication::translate("cataglyphis_startup_form_main", "Press to Run", 0));
        start_up_button->setText(QApplication::translate("cataglyphis_startup_form_main", "StartUp Procedure", 0));
        lost_recovery_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Lost Recovery", 0));
        reboot_recovery_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Reboot Recovery", 0));
        teleport_button->setText(QApplication::translate("cataglyphis_startup_form_main", "Teleport", 0));
    } // retranslateUi

};

namespace Ui {
    class cataglyphis_startup_form_main: public Ui_cataglyphis_startup_form_main {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATAGLYPHIS_STARTUP_FORM_MAIN_H
