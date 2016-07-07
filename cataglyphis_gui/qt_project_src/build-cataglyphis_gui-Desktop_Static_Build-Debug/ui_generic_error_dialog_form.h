/********************************************************************************
** Form generated from reading UI file 'generic_error_dialog_form.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GENERIC_ERROR_DIALOG_FORM_H
#define UI_GENERIC_ERROR_DIALOG_FORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPlainTextEdit>

QT_BEGIN_NAMESPACE

class Ui_generic_error_dialog_form
{
public:
    QFrame *frame;
    QDialogButtonBox *buttonBox;
    QPlainTextEdit *plainTextEdit;

    void setupUi(QDialog *generic_error_dialog_form)
    {
        if (generic_error_dialog_form->objectName().isEmpty())
            generic_error_dialog_form->setObjectName(QStringLiteral("generic_error_dialog_form"));
        generic_error_dialog_form->resize(405, 298);
        frame = new QFrame(generic_error_dialog_form);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 10, 381, 281));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        buttonBox = new QDialogButtonBox(frame);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(10, 240, 361, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);
        plainTextEdit = new QPlainTextEdit(frame);
        plainTextEdit->setObjectName(QStringLiteral("plainTextEdit"));
        plainTextEdit->setGeometry(QRect(10, 10, 361, 51));
        QFont font;
        font.setPointSize(32);
        font.setBold(true);
        font.setItalic(false);
        font.setWeight(75);
        font.setKerning(true);
        plainTextEdit->setFont(font);
        plainTextEdit->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
        plainTextEdit->setUndoRedoEnabled(false);
        plainTextEdit->setReadOnly(true);
        plainTextEdit->setTextInteractionFlags(Qt::NoTextInteraction);

        retranslateUi(generic_error_dialog_form);
        QObject::connect(buttonBox, SIGNAL(accepted()), generic_error_dialog_form, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), generic_error_dialog_form, SLOT(reject()));

        QMetaObject::connectSlotsByName(generic_error_dialog_form);
    } // setupUi

    void retranslateUi(QDialog *generic_error_dialog_form)
    {
        generic_error_dialog_form->setWindowTitle(QApplication::translate("generic_error_dialog_form", "Dialog", 0));
        plainTextEdit->setPlainText(QApplication::translate("generic_error_dialog_form", "SIR, PROBLEM!", 0));
    } // retranslateUi

};

namespace Ui {
    class generic_error_dialog_form: public Ui_generic_error_dialog_form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GENERIC_ERROR_DIALOG_FORM_H
