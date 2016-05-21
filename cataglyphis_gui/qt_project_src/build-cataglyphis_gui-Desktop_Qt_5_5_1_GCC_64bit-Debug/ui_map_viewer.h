/********************************************************************************
** Form generated from reading UI file 'map_viewer.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAP_VIEWER_H
#define UI_MAP_VIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_map_viewer
{
public:
    QVBoxLayout *verticalLayout;
    QGraphicsView *fieldDisplay;
    QComboBox *fieldSelector;

    void setupUi(QWidget *map_viewer)
    {
        if (map_viewer->objectName().isEmpty())
            map_viewer->setObjectName(QStringLiteral("map_viewer"));
        map_viewer->resize(597, 377);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(map_viewer->sizePolicy().hasHeightForWidth());
        map_viewer->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(map_viewer);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        fieldDisplay = new QGraphicsView(map_viewer);
        fieldDisplay->setObjectName(QStringLiteral("fieldDisplay"));
        QSizePolicy sizePolicy1(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(fieldDisplay->sizePolicy().hasHeightForWidth());
        fieldDisplay->setSizePolicy(sizePolicy1);
        fieldDisplay->setMinimumSize(QSize(579, 326));
        fieldDisplay->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
        fieldDisplay->setRenderHints(QPainter::Antialiasing);
        fieldDisplay->setDragMode(QGraphicsView::RubberBandDrag);

        verticalLayout->addWidget(fieldDisplay);

        fieldSelector = new QComboBox(map_viewer);
        fieldSelector->setObjectName(QStringLiteral("fieldSelector"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(fieldSelector->sizePolicy().hasHeightForWidth());
        fieldSelector->setSizePolicy(sizePolicy2);

        verticalLayout->addWidget(fieldSelector);

        fieldDisplay->raise();
        fieldSelector->raise();
        fieldDisplay->raise();
        fieldDisplay->raise();
        fieldSelector->raise();

        retranslateUi(map_viewer);

        QMetaObject::connectSlotsByName(map_viewer);
    } // setupUi

    void retranslateUi(QWidget *map_viewer)
    {
        map_viewer->setWindowTitle(QApplication::translate("map_viewer", "Form", 0));
        fieldSelector->clear();
        fieldSelector->insertItems(0, QStringList()
         << QString()
         << QApplication::translate("map_viewer", "Institute Park", 0)
         << QApplication::translate("map_viewer", "MRB-Library Field", 0)
        );
    } // retranslateUi

};

namespace Ui {
    class map_viewer: public Ui_map_viewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAP_VIEWER_H
