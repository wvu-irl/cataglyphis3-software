/********************************************************************************
** Form generated from reading UI file 'window_selector.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOW_SELECTOR_H
#define UI_WINDOW_SELECTOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_window_selector
{
public:
    QWidget *dockWidgetContents;

    void setupUi(QDockWidget *window_selector)
    {
        if (window_selector->objectName().isEmpty())
            window_selector->setObjectName(QStringLiteral("window_selector"));
        window_selector->resize(209, 300);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        window_selector->setWidget(dockWidgetContents);

        retranslateUi(window_selector);

        QMetaObject::connectSlotsByName(window_selector);
    } // setupUi

    void retranslateUi(QDockWidget *window_selector)
    {
        window_selector->setWindowTitle(QApplication::translate("window_selector", "DockWidget", 0));
    } // retranslateUi

};

namespace Ui {
    class window_selector: public Ui_window_selector {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOW_SELECTOR_H
