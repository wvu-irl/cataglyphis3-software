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
#include <QtWidgets/QFormLayout>
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
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *input_NA_label;
    QDoubleSpinBox *input_NA_spinbox;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout;
    QLabel *current_NA_label;
    QDoubleSpinBox *current_NA_spinbox;
    QSpacerItem *verticalSpacer;
    QFormLayout *formLayout_3;
    QLabel *label;
    QCheckBox *sunny_day_checkbox;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *skip_init_button;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *continue_button;

    void setupUi(QWidget *init_step_one)
    {
        if (init_step_one->objectName().isEmpty())
            init_step_one->setObjectName(QStringLiteral("init_step_one"));
        init_step_one->resize(448, 188);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(init_step_one->sizePolicy().hasHeightForWidth());
        init_step_one->setSizePolicy(sizePolicy);
        init_step_one->setMinimumSize(QSize(448, 188));
        widget = new QWidget(init_step_one);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 10, 425, 167));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        input_NA_label = new QLabel(widget);
        input_NA_label->setObjectName(QStringLiteral("input_NA_label"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(input_NA_label->sizePolicy().hasHeightForWidth());
        input_NA_label->setSizePolicy(sizePolicy1);
        input_NA_label->setMinimumSize(QSize(123, 17));

        verticalLayout_2->addWidget(input_NA_label);

        input_NA_spinbox = new QDoubleSpinBox(widget);
        input_NA_spinbox->setObjectName(QStringLiteral("input_NA_spinbox"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(input_NA_spinbox->sizePolicy().hasHeightForWidth());
        input_NA_spinbox->setSizePolicy(sizePolicy2);
        input_NA_spinbox->setMinimumSize(QSize(123, 26));

        verticalLayout_2->addWidget(input_NA_spinbox);


        horizontalLayout_2->addLayout(verticalLayout_2);

        horizontalSpacer = new QSpacerItem(58, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        current_NA_label = new QLabel(widget);
        current_NA_label->setObjectName(QStringLiteral("current_NA_label"));
        sizePolicy2.setHeightForWidth(current_NA_label->sizePolicy().hasHeightForWidth());
        current_NA_label->setSizePolicy(sizePolicy2);
        current_NA_label->setMinimumSize(QSize(139, 17));

        verticalLayout->addWidget(current_NA_label);

        current_NA_spinbox = new QDoubleSpinBox(widget);
        current_NA_spinbox->setObjectName(QStringLiteral("current_NA_spinbox"));
        current_NA_spinbox->setEnabled(true);
        sizePolicy2.setHeightForWidth(current_NA_spinbox->sizePolicy().hasHeightForWidth());
        current_NA_spinbox->setSizePolicy(sizePolicy2);
        current_NA_spinbox->setMinimumSize(QSize(139, 26));
        current_NA_spinbox->setAlignment(Qt::AlignCenter);
        current_NA_spinbox->setReadOnly(true);
        current_NA_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);

        verticalLayout->addWidget(current_NA_spinbox);


        horizontalLayout_2->addLayout(verticalLayout);


        verticalLayout_3->addLayout(horizontalLayout_2);

        verticalSpacer = new QSpacerItem(423, 13, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        formLayout_3 = new QFormLayout();
        formLayout_3->setObjectName(QStringLiteral("formLayout_3"));
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy3);
        label->setMinimumSize(QSize(94, 16));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, label);

        sunny_day_checkbox = new QCheckBox(widget);
        sunny_day_checkbox->setObjectName(QStringLiteral("sunny_day_checkbox"));
        sunny_day_checkbox->setMinimumSize(QSize(16, 16));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, sunny_day_checkbox);


        verticalLayout_3->addLayout(formLayout_3);

        verticalSpacer_2 = new QSpacerItem(423, 13, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        skip_init_button = new QPushButton(widget);
        skip_init_button->setObjectName(QStringLiteral("skip_init_button"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(skip_init_button->sizePolicy().hasHeightForWidth());
        skip_init_button->setSizePolicy(sizePolicy4);
        skip_init_button->setMinimumSize(QSize(80, 21));

        horizontalLayout->addWidget(skip_init_button);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        continue_button = new QPushButton(widget);
        continue_button->setObjectName(QStringLiteral("continue_button"));
        sizePolicy4.setHeightForWidth(continue_button->sizePolicy().hasHeightForWidth());
        continue_button->setSizePolicy(sizePolicy4);
        continue_button->setMinimumSize(QSize(80, 41));

        horizontalLayout->addWidget(continue_button);


        verticalLayout_3->addLayout(horizontalLayout);

        current_NA_label->raise();
        input_NA_spinbox->raise();
        current_NA_label->raise();
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
