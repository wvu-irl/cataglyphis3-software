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
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_bias_removal_form
{
public:
    QPushButton *begin_dead_reckoning_button;
    QPushButton *perform_bias_removal_button;
    QProgressBar *progressBar;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QLabel *label;
    QDoubleSpinBox *p1_offset_spinbox;
    QFormLayout *formLayout_2;
    QLabel *label_2;
    QDoubleSpinBox *q1_offset_spinbox;
    QFormLayout *formLayout_3;
    QLabel *label_3;
    QDoubleSpinBox *r1_offset_spinbox;
    QFormLayout *formLayout_4;
    QLabel *label_4;
    QDoubleSpinBox *p2_offset_spinbox;
    QFormLayout *formLayout_5;
    QLabel *label_5;
    QDoubleSpinBox *q2_offset_spinbox;
    QFormLayout *formLayout_6;
    QLabel *label_6;
    QDoubleSpinBox *r2_offset_spinbox;
    QFormLayout *formLayout_7;
    QLabel *label_7;
    QDoubleSpinBox *p3_offset_spinbox;
    QFormLayout *formLayout_8;
    QLabel *label_8;
    QDoubleSpinBox *q3_offset_spinbox;
    QFormLayout *formLayout_9;
    QLabel *label_9;
    QDoubleSpinBox *r3_offset_spinbox;

    void setupUi(QWidget *bias_removal_form)
    {
        if (bias_removal_form->objectName().isEmpty())
            bias_removal_form->setObjectName(QStringLiteral("bias_removal_form"));
        bias_removal_form->resize(320, 258);
        begin_dead_reckoning_button = new QPushButton(bias_removal_form);
        begin_dead_reckoning_button->setObjectName(QStringLiteral("begin_dead_reckoning_button"));
        begin_dead_reckoning_button->setEnabled(false);
        begin_dead_reckoning_button->setGeometry(QRect(170, 170, 111, 41));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(begin_dead_reckoning_button->sizePolicy().hasHeightForWidth());
        begin_dead_reckoning_button->setSizePolicy(sizePolicy);
        perform_bias_removal_button = new QPushButton(bias_removal_form);
        perform_bias_removal_button->setObjectName(QStringLiteral("perform_bias_removal_button"));
        perform_bias_removal_button->setGeometry(QRect(170, 30, 111, 61));
        sizePolicy.setHeightForWidth(perform_bias_removal_button->sizePolicy().hasHeightForWidth());
        perform_bias_removal_button->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(19);
        perform_bias_removal_button->setFont(font);
        progressBar = new QProgressBar(bias_removal_form);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setGeometry(QRect(10, 230, 301, 23));
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(progressBar->sizePolicy().hasHeightForWidth());
        progressBar->setSizePolicy(sizePolicy1);
        progressBar->setValue(0);
        progressBar->setTextVisible(false);
        progressBar->setOrientation(Qt::Horizontal);
        progressBar->setInvertedAppearance(false);
        layoutWidget = new QWidget(bias_removal_form);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 136, 215));
        sizePolicy.setHeightForWidth(layoutWidget->sizePolicy().hasHeightForWidth());
        layoutWidget->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setEnabled(true);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        p1_offset_spinbox = new QDoubleSpinBox(layoutWidget);
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


        verticalLayout->addLayout(formLayout);

        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QStringLiteral("formLayout_2"));
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_2);

        q1_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        q1_offset_spinbox->setObjectName(QStringLiteral("q1_offset_spinbox"));
        sizePolicy.setHeightForWidth(q1_offset_spinbox->sizePolicy().hasHeightForWidth());
        q1_offset_spinbox->setSizePolicy(sizePolicy);
        q1_offset_spinbox->setFrame(true);
        q1_offset_spinbox->setAlignment(Qt::AlignCenter);
        q1_offset_spinbox->setReadOnly(true);
        q1_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q1_offset_spinbox->setDecimals(9);
        q1_offset_spinbox->setMinimum(-99);

        formLayout_2->setWidget(0, QFormLayout::FieldRole, q1_offset_spinbox);


        verticalLayout->addLayout(formLayout_2);

        formLayout_3 = new QFormLayout();
        formLayout_3->setObjectName(QStringLiteral("formLayout_3"));
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        formLayout_3->setWidget(0, QFormLayout::LabelRole, label_3);

        r1_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        r1_offset_spinbox->setObjectName(QStringLiteral("r1_offset_spinbox"));
        sizePolicy.setHeightForWidth(r1_offset_spinbox->sizePolicy().hasHeightForWidth());
        r1_offset_spinbox->setSizePolicy(sizePolicy);
        r1_offset_spinbox->setFrame(true);
        r1_offset_spinbox->setAlignment(Qt::AlignCenter);
        r1_offset_spinbox->setReadOnly(true);
        r1_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r1_offset_spinbox->setDecimals(9);
        r1_offset_spinbox->setMinimum(-99);

        formLayout_3->setWidget(0, QFormLayout::FieldRole, r1_offset_spinbox);


        verticalLayout->addLayout(formLayout_3);

        formLayout_4 = new QFormLayout();
        formLayout_4->setObjectName(QStringLiteral("formLayout_4"));
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);

        formLayout_4->setWidget(0, QFormLayout::LabelRole, label_4);

        p2_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        p2_offset_spinbox->setObjectName(QStringLiteral("p2_offset_spinbox"));
        sizePolicy.setHeightForWidth(p2_offset_spinbox->sizePolicy().hasHeightForWidth());
        p2_offset_spinbox->setSizePolicy(sizePolicy);
        p2_offset_spinbox->setFrame(true);
        p2_offset_spinbox->setAlignment(Qt::AlignCenter);
        p2_offset_spinbox->setReadOnly(true);
        p2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        p2_offset_spinbox->setDecimals(9);
        p2_offset_spinbox->setMinimum(-99);

        formLayout_4->setWidget(0, QFormLayout::FieldRole, p2_offset_spinbox);


        verticalLayout->addLayout(formLayout_4);

        formLayout_5 = new QFormLayout();
        formLayout_5->setObjectName(QStringLiteral("formLayout_5"));
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        formLayout_5->setWidget(0, QFormLayout::LabelRole, label_5);

        q2_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        q2_offset_spinbox->setObjectName(QStringLiteral("q2_offset_spinbox"));
        sizePolicy.setHeightForWidth(q2_offset_spinbox->sizePolicy().hasHeightForWidth());
        q2_offset_spinbox->setSizePolicy(sizePolicy);
        q2_offset_spinbox->setFrame(true);
        q2_offset_spinbox->setAlignment(Qt::AlignCenter);
        q2_offset_spinbox->setReadOnly(true);
        q2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q2_offset_spinbox->setDecimals(9);
        q2_offset_spinbox->setMinimum(-99);

        formLayout_5->setWidget(0, QFormLayout::FieldRole, q2_offset_spinbox);


        verticalLayout->addLayout(formLayout_5);

        formLayout_6 = new QFormLayout();
        formLayout_6->setObjectName(QStringLiteral("formLayout_6"));
        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        formLayout_6->setWidget(0, QFormLayout::LabelRole, label_6);

        r2_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        r2_offset_spinbox->setObjectName(QStringLiteral("r2_offset_spinbox"));
        sizePolicy.setHeightForWidth(r2_offset_spinbox->sizePolicy().hasHeightForWidth());
        r2_offset_spinbox->setSizePolicy(sizePolicy);
        r2_offset_spinbox->setFrame(true);
        r2_offset_spinbox->setAlignment(Qt::AlignCenter);
        r2_offset_spinbox->setReadOnly(true);
        r2_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r2_offset_spinbox->setDecimals(9);
        r2_offset_spinbox->setMinimum(-99);

        formLayout_6->setWidget(0, QFormLayout::FieldRole, r2_offset_spinbox);


        verticalLayout->addLayout(formLayout_6);

        formLayout_7 = new QFormLayout();
        formLayout_7->setObjectName(QStringLiteral("formLayout_7"));
        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);

        formLayout_7->setWidget(0, QFormLayout::LabelRole, label_7);

        p3_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        p3_offset_spinbox->setObjectName(QStringLiteral("p3_offset_spinbox"));
        sizePolicy.setHeightForWidth(p3_offset_spinbox->sizePolicy().hasHeightForWidth());
        p3_offset_spinbox->setSizePolicy(sizePolicy);
        p3_offset_spinbox->setFrame(true);
        p3_offset_spinbox->setAlignment(Qt::AlignCenter);
        p3_offset_spinbox->setReadOnly(true);
        p3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        p3_offset_spinbox->setDecimals(9);
        p3_offset_spinbox->setMinimum(-99);

        formLayout_7->setWidget(0, QFormLayout::FieldRole, p3_offset_spinbox);


        verticalLayout->addLayout(formLayout_7);

        formLayout_8 = new QFormLayout();
        formLayout_8->setObjectName(QStringLiteral("formLayout_8"));
        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);

        formLayout_8->setWidget(0, QFormLayout::LabelRole, label_8);

        q3_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        q3_offset_spinbox->setObjectName(QStringLiteral("q3_offset_spinbox"));
        sizePolicy.setHeightForWidth(q3_offset_spinbox->sizePolicy().hasHeightForWidth());
        q3_offset_spinbox->setSizePolicy(sizePolicy);
        q3_offset_spinbox->setFrame(true);
        q3_offset_spinbox->setAlignment(Qt::AlignCenter);
        q3_offset_spinbox->setReadOnly(true);
        q3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        q3_offset_spinbox->setDecimals(9);
        q3_offset_spinbox->setMinimum(-99);

        formLayout_8->setWidget(0, QFormLayout::FieldRole, q3_offset_spinbox);


        verticalLayout->addLayout(formLayout_8);

        formLayout_9 = new QFormLayout();
        formLayout_9->setObjectName(QStringLiteral("formLayout_9"));
        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);

        formLayout_9->setWidget(0, QFormLayout::LabelRole, label_9);

        r3_offset_spinbox = new QDoubleSpinBox(layoutWidget);
        r3_offset_spinbox->setObjectName(QStringLiteral("r3_offset_spinbox"));
        sizePolicy.setHeightForWidth(r3_offset_spinbox->sizePolicy().hasHeightForWidth());
        r3_offset_spinbox->setSizePolicy(sizePolicy);
        r3_offset_spinbox->setFrame(true);
        r3_offset_spinbox->setAlignment(Qt::AlignCenter);
        r3_offset_spinbox->setReadOnly(true);
        r3_offset_spinbox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        r3_offset_spinbox->setDecimals(9);
        r3_offset_spinbox->setMinimum(-99);

        formLayout_9->setWidget(0, QFormLayout::FieldRole, r3_offset_spinbox);


        verticalLayout->addLayout(formLayout_9);


        retranslateUi(bias_removal_form);

        QMetaObject::connectSlotsByName(bias_removal_form);
    } // setupUi

    void retranslateUi(QWidget *bias_removal_form)
    {
        bias_removal_form->setWindowTitle(QApplication::translate("bias_removal_form", "Form", 0));
        begin_dead_reckoning_button->setText(QApplication::translate("bias_removal_form", "Begin\n"
"Dead Reckoning", 0));
        perform_bias_removal_button->setText(QApplication::translate("bias_removal_form", "Perform\n"
"Bias Removal", 0));
        label->setText(QApplication::translate("bias_removal_form", "P1 Offset", 0));
        label_2->setText(QApplication::translate("bias_removal_form", "Q1 Offset", 0));
        label_3->setText(QApplication::translate("bias_removal_form", "R1 Offset", 0));
        label_4->setText(QApplication::translate("bias_removal_form", "P2 Offset", 0));
        label_5->setText(QApplication::translate("bias_removal_form", "Q2 Offset", 0));
        label_6->setText(QApplication::translate("bias_removal_form", "R2 Offset", 0));
        label_7->setText(QApplication::translate("bias_removal_form", "P3 Offset", 0));
        label_8->setText(QApplication::translate("bias_removal_form", "Q3 Offset", 0));
        label_9->setText(QApplication::translate("bias_removal_form", "R3 Offset", 0));
    } // retranslateUi

};

namespace Ui {
    class bias_removal_form: public Ui_bias_removal_form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BIAS_REMOVAL_FORM_H
