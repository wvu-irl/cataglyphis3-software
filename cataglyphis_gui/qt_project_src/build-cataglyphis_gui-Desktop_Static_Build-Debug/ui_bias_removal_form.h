/********************************************************************************
** Form generated from reading UI file 'bias_removal_form.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BIAS_REMOVAL_FORM_H
#define UI_BIAS_REMOVAL_FORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bias_removal_form
{
public:
    QWidget *widget;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QFormLayout *formLayout;
    QLabel *label;
    QDoubleSpinBox *p1_offset_spinbox;
    QLabel *label_2;
    QDoubleSpinBox *q1_offset_spinbox;
    QLabel *label_3;
    QDoubleSpinBox *r1_offset_spinbox;
    QLabel *label_4;
    QDoubleSpinBox *p2_offset_spinbox;
    QLabel *label_5;
    QDoubleSpinBox *q2_offset_spinbox;
    QLabel *label_6;
    QDoubleSpinBox *r2_offset_spinbox;
    QLabel *label_7;
    QDoubleSpinBox *p3_offset_spinbox;
    QLabel *label_8;
    QDoubleSpinBox *q3_offset_spinbox;
    QLabel *label_9;
    QDoubleSpinBox *r3_offset_spinbox;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer_2;
    QPushButton *perform_bias_removal_button;
    QSpacerItem *verticalSpacer;
    QPushButton *begin_dead_reckoning_button;
    QSpacerItem *verticalSpacer_3;
    QProgressBar *progressBar;

    void setupUi(QWidget *bias_removal_form)
    {
        if (bias_removal_form->objectName().isEmpty())
            bias_removal_form->setObjectName(QStringLiteral("bias_removal_form"));
        bias_removal_form->resize(352, 374);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(bias_removal_form->sizePolicy().hasHeightForWidth());
        bias_removal_form->setSizePolicy(sizePolicy);
        bias_removal_form->setMinimumSize(QSize(352, 374));
        widget = new QWidget(bias_removal_form);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 10, 332, 319));
        verticalLayout_2 = new QVBoxLayout(widget);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));
        label->setEnabled(true);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        p1_offset_spinbox = new QDoubleSpinBox(widget);
        p1_offset_spinbox->setObjectName(QStringLiteral("p1_offset_spinbox"));
        sizePolicy.setHeightForWidth(p1_offset_spinbox->sizePolicy().hasHeightForWidth());
        p1_offset_spinbox->setSizePolicy(sizePolicy);
        p1_offset_spinbox->setFrame(true);
        p1_offset_spinbox->setAlignment(Qt::AlignCenter);
        p1_offset_spinbox->setReadOnly(true);
        p1_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        p1_offset_spinbox->setDecimals(9);
        p1_offset_spinbox->setMinimum(-99.99);

        formLayout->setWidget(0, QFormLayout::FieldRole, p1_offset_spinbox);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        q1_offset_spinbox = new QDoubleSpinBox(widget);
        q1_offset_spinbox->setObjectName(QStringLiteral("q1_offset_spinbox"));
        sizePolicy.setHeightForWidth(q1_offset_spinbox->sizePolicy().hasHeightForWidth());
        q1_offset_spinbox->setSizePolicy(sizePolicy);
        q1_offset_spinbox->setFrame(true);
        q1_offset_spinbox->setAlignment(Qt::AlignCenter);
        q1_offset_spinbox->setReadOnly(true);
        q1_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q1_offset_spinbox->setDecimals(9);
        q1_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(1, QFormLayout::FieldRole, q1_offset_spinbox);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        r1_offset_spinbox = new QDoubleSpinBox(widget);
        r1_offset_spinbox->setObjectName(QStringLiteral("r1_offset_spinbox"));
        sizePolicy.setHeightForWidth(r1_offset_spinbox->sizePolicy().hasHeightForWidth());
        r1_offset_spinbox->setSizePolicy(sizePolicy);
        r1_offset_spinbox->setFrame(true);
        r1_offset_spinbox->setAlignment(Qt::AlignCenter);
        r1_offset_spinbox->setReadOnly(true);
        r1_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r1_offset_spinbox->setDecimals(9);
        r1_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(2, QFormLayout::FieldRole, r1_offset_spinbox);

        label_4 = new QLabel(widget);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);

        formLayout->setWidget(3, QFormLayout::LabelRole, label_4);

        p2_offset_spinbox = new QDoubleSpinBox(widget);
        p2_offset_spinbox->setObjectName(QStringLiteral("p2_offset_spinbox"));
        sizePolicy.setHeightForWidth(p2_offset_spinbox->sizePolicy().hasHeightForWidth());
        p2_offset_spinbox->setSizePolicy(sizePolicy);
        p2_offset_spinbox->setFrame(true);
        p2_offset_spinbox->setAlignment(Qt::AlignCenter);
        p2_offset_spinbox->setReadOnly(true);
        p2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        p2_offset_spinbox->setDecimals(9);
        p2_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(3, QFormLayout::FieldRole, p2_offset_spinbox);

        label_5 = new QLabel(widget);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        formLayout->setWidget(4, QFormLayout::LabelRole, label_5);

        q2_offset_spinbox = new QDoubleSpinBox(widget);
        q2_offset_spinbox->setObjectName(QStringLiteral("q2_offset_spinbox"));
        sizePolicy.setHeightForWidth(q2_offset_spinbox->sizePolicy().hasHeightForWidth());
        q2_offset_spinbox->setSizePolicy(sizePolicy);
        q2_offset_spinbox->setFrame(true);
        q2_offset_spinbox->setAlignment(Qt::AlignCenter);
        q2_offset_spinbox->setReadOnly(true);
        q2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q2_offset_spinbox->setDecimals(9);
        q2_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(4, QFormLayout::FieldRole, q2_offset_spinbox);

        label_6 = new QLabel(widget);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        formLayout->setWidget(5, QFormLayout::LabelRole, label_6);

        r2_offset_spinbox = new QDoubleSpinBox(widget);
        r2_offset_spinbox->setObjectName(QStringLiteral("r2_offset_spinbox"));
        sizePolicy.setHeightForWidth(r2_offset_spinbox->sizePolicy().hasHeightForWidth());
        r2_offset_spinbox->setSizePolicy(sizePolicy);
        r2_offset_spinbox->setFrame(true);
        r2_offset_spinbox->setAlignment(Qt::AlignCenter);
        r2_offset_spinbox->setReadOnly(true);
        r2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r2_offset_spinbox->setDecimals(9);
        r2_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(5, QFormLayout::FieldRole, r2_offset_spinbox);

        label_7 = new QLabel(widget);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);

        formLayout->setWidget(6, QFormLayout::LabelRole, label_7);

        p3_offset_spinbox = new QDoubleSpinBox(widget);
        p3_offset_spinbox->setObjectName(QStringLiteral("p3_offset_spinbox"));
        sizePolicy.setHeightForWidth(p3_offset_spinbox->sizePolicy().hasHeightForWidth());
        p3_offset_spinbox->setSizePolicy(sizePolicy);
        p3_offset_spinbox->setFrame(true);
        p3_offset_spinbox->setAlignment(Qt::AlignCenter);
        p3_offset_spinbox->setReadOnly(true);
        p3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        p3_offset_spinbox->setDecimals(9);
        p3_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(6, QFormLayout::FieldRole, p3_offset_spinbox);

        label_8 = new QLabel(widget);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);

        formLayout->setWidget(7, QFormLayout::LabelRole, label_8);

        q3_offset_spinbox = new QDoubleSpinBox(widget);
        q3_offset_spinbox->setObjectName(QStringLiteral("q3_offset_spinbox"));
        sizePolicy.setHeightForWidth(q3_offset_spinbox->sizePolicy().hasHeightForWidth());
        q3_offset_spinbox->setSizePolicy(sizePolicy);
        q3_offset_spinbox->setFrame(true);
        q3_offset_spinbox->setAlignment(Qt::AlignCenter);
        q3_offset_spinbox->setReadOnly(true);
        q3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q3_offset_spinbox->setDecimals(9);
        q3_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(7, QFormLayout::FieldRole, q3_offset_spinbox);

        label_9 = new QLabel(widget);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);

        formLayout->setWidget(8, QFormLayout::LabelRole, label_9);

        r3_offset_spinbox = new QDoubleSpinBox(widget);
        r3_offset_spinbox->setObjectName(QStringLiteral("r3_offset_spinbox"));
        sizePolicy.setHeightForWidth(r3_offset_spinbox->sizePolicy().hasHeightForWidth());
        r3_offset_spinbox->setSizePolicy(sizePolicy);
        r3_offset_spinbox->setFrame(true);
        r3_offset_spinbox->setAlignment(Qt::AlignCenter);
        r3_offset_spinbox->setReadOnly(true);
        r3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r3_offset_spinbox->setDecimals(9);
        r3_offset_spinbox->setMinimum(-99);

        formLayout->setWidget(8, QFormLayout::FieldRole, r3_offset_spinbox);


        horizontalLayout->addLayout(formLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        perform_bias_removal_button = new QPushButton(widget);
        perform_bias_removal_button->setObjectName(QStringLiteral("perform_bias_removal_button"));
        QFont font;
        font.setPointSize(13);
        perform_bias_removal_button->setFont(font);

        verticalLayout->addWidget(perform_bias_removal_button);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        begin_dead_reckoning_button = new QPushButton(widget);
        begin_dead_reckoning_button->setObjectName(QStringLiteral("begin_dead_reckoning_button"));
        begin_dead_reckoning_button->setEnabled(false);
        QFont font1;
        font1.setPointSize(10);
        begin_dead_reckoning_button->setFont(font1);

        verticalLayout->addWidget(begin_dead_reckoning_button);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout);

        progressBar = new QProgressBar(widget);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(progressBar->sizePolicy().hasHeightForWidth());
        progressBar->setSizePolicy(sizePolicy1);
        progressBar->setValue(0);
        progressBar->setTextVisible(false);
        progressBar->setOrientation(Qt::Horizontal);
        progressBar->setInvertedAppearance(false);

        verticalLayout_2->addWidget(progressBar);


        retranslateUi(bias_removal_form);

        QMetaObject::connectSlotsByName(bias_removal_form);
    } // setupUi

    void retranslateUi(QWidget *bias_removal_form)
    {
        bias_removal_form->setWindowTitle(QApplication::translate("bias_removal_form", "Form", 0));
        label->setText(QApplication::translate("bias_removal_form", "P1 Offset", 0));
        label_2->setText(QApplication::translate("bias_removal_form", "Q1 Offset", 0));
        label_3->setText(QApplication::translate("bias_removal_form", "R1 Offset", 0));
        label_4->setText(QApplication::translate("bias_removal_form", "P2 Offset", 0));
        label_5->setText(QApplication::translate("bias_removal_form", "Q2 Offset", 0));
        label_6->setText(QApplication::translate("bias_removal_form", "R2 Offset", 0));
        label_7->setText(QApplication::translate("bias_removal_form", "P3 Offset", 0));
        label_8->setText(QApplication::translate("bias_removal_form", "Q3 Offset", 0));
        label_9->setText(QApplication::translate("bias_removal_form", "R3 Offset", 0));
        perform_bias_removal_button->setText(QApplication::translate("bias_removal_form", "Perform\n"
"Bias Removal", 0));
        begin_dead_reckoning_button->setText(QApplication::translate("bias_removal_form", "Begin\n"
"Dead Reckoning", 0));
    } // retranslateUi

};

namespace Ui {
    class bias_removal_form: public Ui_bias_removal_form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BIAS_REMOVAL_FORM_H
