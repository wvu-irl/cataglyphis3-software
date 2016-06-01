/********************************************************************************
** Form generated from reading UI file 'cataglyphis_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CATAGLYPHIS_GUI_H
#define UI_CATAGLYPHIS_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cataglyphis_gui
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QTabWidget *guiTabber;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *cataglyphis_gui)
    {
        if (cataglyphis_gui->objectName().isEmpty())
            cataglyphis_gui->setObjectName(QStringLiteral("cataglyphis_gui"));
        cataglyphis_gui->resize(800, 600);
        centralwidget = new QWidget(cataglyphis_gui);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        guiTabber = new QTabWidget(centralwidget);
        guiTabber->setObjectName(QStringLiteral("guiTabber"));

        verticalLayout->addWidget(guiTabber);

        cataglyphis_gui->setCentralWidget(centralwidget);
        menubar = new QMenuBar(cataglyphis_gui);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 25));
        cataglyphis_gui->setMenuBar(menubar);
        statusbar = new QStatusBar(cataglyphis_gui);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        cataglyphis_gui->setStatusBar(statusbar);

        retranslateUi(cataglyphis_gui);

        QMetaObject::connectSlotsByName(cataglyphis_gui);
    } // setupUi

    void retranslateUi(QMainWindow *cataglyphis_gui)
    {
        cataglyphis_gui->setWindowTitle(QApplication::translate("cataglyphis_gui", "MainWindow", 0));
    } // retranslateUi

};

namespace Ui {
    class cataglyphis_gui: public Ui_cataglyphis_gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATAGLYPHIS_GUI_H
