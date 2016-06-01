/********************************************************************************
** Form generated from reading UI file 'init_step_one.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INIT_STEP_ONE_H
#define UI_INIT_STEP_ONE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_init_step_one
{
public:
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *input_NA_label;
    QDoubleSpinBox *input_NA_spinbox;
    QSpacerItem *horizontalSpacer;
    QLabel *current_NA_label;
    QDoubleSpinBox *current_NA_spinbox;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QCheckBox *sunny_day_checkbox;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *skip_init_button;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *continue_button;

    void setupUi(QWidget *init_step_one)
    {
        if (init_step_one->objectName().isEmpty())
            init_step_one->setObjectName(QStringLiteral("init_step_one"));
        init_step_one->resize(350, 236);
        widget = new QWidget(init_step_one);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 10, 330, 133));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetMinimumSize);
        input_NA_label = new QLabel(widget);
        input_NA_label->setObjectName(QStringLiteral("input_NA_label"));

        horizontalLayout->addWidget(input_NA_label);

        input_NA_spinbox = new QDoubleSpinBox(widget);
        input_NA_spinbox->setObjectName(QStringLiteral("input_NA_spinbox"));

        horizontalLayout->addWidget(input_NA_spinbox);

        horizontalSpacer = new QSpacerItem(58, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        current_NA_label = new QLabel(widget);
        current_NA_label->setObjectName(QStringLiteral("current_NA_label"));

        horizontalLayout->addWidget(current_NA_label);

        current_NA_spinbox = new QDoubleSpinBox(widget);
        current_NA_spinbox->setObjectName(QStringLiteral("current_NA_spinbox"));
        current_NA_spinbox->setAlignment(Qt::AlignCenter);
        current_NA_spinbox->setReadOnly(true);
        current_NA_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);

        horizontalLayout->addWidget(current_NA_spinbox);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer = new QSpacerItem(20, 18, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setSizeConstraint(QLayout::SetMinimumSize);
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMinimumSize(QSize(94, 16));

        horizontalLayout_3->addWidget(label);

        sunny_day_checkbox = new QCheckBox(widget);
        sunny_day_checkbox->setObjectName(QStringLiteral("sunny_day_checkbox"));
        sunny_day_checkbox->setMinimumSize(QSize(16, 16));

        horizontalLayout_3->addWidget(sunny_day_checkbox);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalSpacer_2 = new QSpacerItem(20, 18, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        skip_init_button = new QPushButton(widget);
        skip_init_button->setObjectName(QStringLiteral("skip_init_button"));
        skip_init_button->setMinimumSize(QSize(80, 21));

        horizontalLayout_4->addWidget(skip_init_button);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_3);

        continue_button = new QPushButton(widget);
        continue_button->setObjectName(QStringLiteral("continue_button"));
        continue_button->setMinimumSize(QSize(80, 41));

        horizontalLayout_4->addWidget(continue_button);


        verticalLayout->addLayout(horizontalLayout_4);

        input_NA_spinbox->raise();
        input_NA_label->raise();
        current_NA_spinbox->raise();
        current_NA_label->raise();
        current_NA_spinbox->raise();
        sunny_day_checkbox->raise();
        label->raise();
        continue_button->raise();
        skip_init_button->raise();
        input_NA_label->raise();

        retranslateUi(init_step_one);

        QMetaObject::connectSlotsByName(init_step_one);
    } // setupUi

    void retranslateUi(QWidget *init_step_one)
    {
        init_step_one->setWindowTitle(QApplication::translate("init_step_one", "Form", 0));
        input_NA_label->setText(QApplication::translate("init_step_one", "Input North Angle", 0));
        current_NA_label->setText(QApplication::translate("init_step_one", "Current North Angle", 0));
        label->setText(QApplication::translate("init_step_one", "Sunny Day? ", 0));
        sunny_day_checkbox->setText(QString());
        skip_init_button->setText(QApplication::translate("init_step_one", "Skip Init", 0));
        continue_button->setText(QApplication::translate("init_step_one", "Submit", 0));
    } // retranslateUi

};

namespace Ui {
    class init_step_one: public Ui_init_step_one {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INIT_STEP_ONE_H
