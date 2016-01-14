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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Cataglyphis_Gui
{
public:
    QAction *actionConnect;
    QAction *actionPlaceholder;
    QAction *actionPlaceholder2;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_3;
    QTabWidget *tabWidget;
    QWidget *fieldViewTab;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QGraphicsView *fieldDisplay;
    QComboBox *fieldSelector;
    QWidget *dataViewTab;
    QMenuBar *menuBar;
    QMenu *menuCommands;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Cataglyphis_Gui)
    {
        if (Cataglyphis_Gui->objectName().isEmpty())
            Cataglyphis_Gui->setObjectName(QStringLiteral("Cataglyphis_Gui"));
        Cataglyphis_Gui->resize(916, 555);
        actionConnect = new QAction(Cataglyphis_Gui);
        actionConnect->setObjectName(QStringLiteral("actionConnect"));
        actionPlaceholder = new QAction(Cataglyphis_Gui);
        actionPlaceholder->setObjectName(QStringLiteral("actionPlaceholder"));
        actionPlaceholder2 = new QAction(Cataglyphis_Gui);
        actionPlaceholder2->setObjectName(QStringLiteral("actionPlaceholder2"));
        centralWidget = new QWidget(Cataglyphis_Gui);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout_3 = new QVBoxLayout(centralWidget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        fieldViewTab = new QWidget();
        fieldViewTab->setObjectName(QStringLiteral("fieldViewTab"));
        verticalLayout_2 = new QVBoxLayout(fieldViewTab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        fieldDisplay = new QGraphicsView(fieldViewTab);
        fieldDisplay->setObjectName(QStringLiteral("fieldDisplay"));
        fieldDisplay->setRenderHints(QPainter::Antialiasing);
        fieldDisplay->setDragMode(QGraphicsView::ScrollHandDrag);

        verticalLayout->addWidget(fieldDisplay);

        fieldSelector = new QComboBox(fieldViewTab);
        fieldSelector->setObjectName(QStringLiteral("fieldSelector"));

        verticalLayout->addWidget(fieldSelector);


        verticalLayout_2->addLayout(verticalLayout);

        tabWidget->addTab(fieldViewTab, QString());
        dataViewTab = new QWidget();
        dataViewTab->setObjectName(QStringLiteral("dataViewTab"));
        tabWidget->addTab(dataViewTab, QString());

        verticalLayout_3->addWidget(tabWidget);

        Cataglyphis_Gui->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Cataglyphis_Gui);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 916, 25));
        menuCommands = new QMenu(menuBar);
        menuCommands->setObjectName(QStringLiteral("menuCommands"));
        Cataglyphis_Gui->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Cataglyphis_Gui);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Cataglyphis_Gui->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Cataglyphis_Gui);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Cataglyphis_Gui->setStatusBar(statusBar);

        menuBar->addAction(menuCommands->menuAction());
        menuCommands->addAction(actionConnect);
        menuCommands->addAction(actionPlaceholder);
        menuCommands->addSeparator();
        menuCommands->addAction(actionPlaceholder2);

        retranslateUi(Cataglyphis_Gui);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Cataglyphis_Gui);
    } // setupUi

    void retranslateUi(QMainWindow *Cataglyphis_Gui)
    {
        Cataglyphis_Gui->setWindowTitle(QApplication::translate("Cataglyphis_Gui", "Cataglyphis_Gui", 0));
        actionConnect->setText(QApplication::translate("Cataglyphis_Gui", "Connect", 0));
        actionPlaceholder->setText(QApplication::translate("Cataglyphis_Gui", "Placeholder", 0));
        actionPlaceholder2->setText(QApplication::translate("Cataglyphis_Gui", "Placeholder2", 0));
        fieldSelector->clear();
        fieldSelector->insertItems(0, QStringList()
         << QString()
         << QApplication::translate("Cataglyphis_Gui", "Institute Park", 0)
         << QApplication::translate("Cataglyphis_Gui", "MRB-Library Field", 0)
        );
        tabWidget->setTabText(tabWidget->indexOf(fieldViewTab), QApplication::translate("Cataglyphis_Gui", "Field View", 0));
        tabWidget->setTabText(tabWidget->indexOf(dataViewTab), QApplication::translate("Cataglyphis_Gui", "Data View", 0));
        menuCommands->setTitle(QApplication::translate("Cataglyphis_Gui", "Commands", 0));
    } // retranslateUi

};

namespace Ui {
    class Cataglyphis_Gui: public Ui_Cataglyphis_Gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CATAGLYPHIS_GUI_H
