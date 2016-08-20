#-------------------------------------------------
#
# Project created by QtCreator 2016-01-13T12:13:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = cataglyphis_gui
TEMPLATE = app

CONFIG += c++11

CONFIG(static, debug|release|static):{
    CONFIG += static
    DEFINES += STATIC
    DEFINES += STATIC_BUILD
    message("Static Build.")
}
CONFIG(release, debug|release|static):{
    message("Release Build.")
    DEFINES += TEST_RELEASE_BUILD
}
CONFIG(debug, debug|release|static):{
    DEFINES += DEBUG_BUILD
    message("Debug Build.")
}

SOURCES +=\
    map_viewer.cpp \
    init_step_one.cpp \
    ros_workers.cpp \
    ../../src/cataglyphis_gui_node.cpp \
    init_step_two.cpp \
    core_app.cpp \
    generic_error_dialog.cpp \
    init_container.cpp \
    map_view_roi_ellipse.cpp \
    roi_dialog.cpp \
    qgraphicsviewscrollscale.cpp \
    qgraphicsscenemapviewer.cpp \
    generic_ack_dialog.cpp \
    map_viewer_rect.cpp \
    shift_map.cpp \
    node_status.cpp \
    manual_control.cpp \
    teleport.cpp

HEADERS  += \
    map_viewer.h \
    init_step_one.h \
    ros_workers.h \
    init_step_two.h \
    core_app.h \
    generic_error_dialog.h \
    init_container.h \
    map_view_roi_ellipse.h \
    roi_dialog.h \
    qgraphicsviewscrollscale.h \
    qgraphicsscenemapviewer.h \
    generic_ack_dialog.h \
    map_viewer_enums.h \
    ../../../robot_control/include/robot_control/map_layers.h \
    ../../../robot_control/include/robot_control/task_type_enum.h \
    map_viewer_rect.h \
    shift_map.h \
    node_status.h \
    manual_control.h \
    teleport.h

FORMS    += \
    generic_error_dialog_form.ui \
    init_step_two_form.ui \
    init_step_one_form.ui \
    map_viewer_form.ui \
    core_app_form.ui \
    init_container_form.ui \
    roi_dialog_form.ui \
    generic_ack_dialog_form.ui \
    shift_map_form.ui \
    node_status_form.ui \
    manual_control_form.ui \
    teleport_form.ui

RESOURCES += \
    resources.qrc

DISTFILES +=

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += roscpp
unix: PKGCONFIG += roslib
unix: PKGCONFIG += grid_map_ros
unix: PKGCONFIG += grid_map_msgs
#unix: PKGCONFIG += grid_map_loader
#unix: PKGCONFIG += grid_map_visualization

unix:!macx: LIBS += -lgrid_map_core

#unix:!macx: LIBS += -lgrid_map_cv

#unix:!macx: LIBS += -lgrid_map_filters

unix:!macx: LIBS += -lgrid_map_ros
