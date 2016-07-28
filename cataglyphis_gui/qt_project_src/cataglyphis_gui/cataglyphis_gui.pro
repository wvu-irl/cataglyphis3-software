#-------------------------------------------------
#
# Project created by QtCreator 2016-01-13T12:13:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = cataglyphis_gui
TEMPLATE = app


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
    init_container.cpp

HEADERS  += \
    map_viewer.h \
    init_step_one.h \
    ros_workers.h \
    init_step_two.h \
    core_app.h \
    generic_error_dialog.h \
    init_container.h

FORMS    += \
    generic_error_dialog_form.ui \
    init_step_two_form.ui \
    init_step_one_form.ui \
    map_viewer_form.ui \
    core_app_form.ui \
    init_container_form.ui

RESOURCES += \
    resources.qrc

DISTFILES +=

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += roscpp

unix: PKGCONFIG += roslib

