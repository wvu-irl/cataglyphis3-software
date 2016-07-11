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
    cataglyphis_startup_form_main.cpp \
    map_viewer.cpp \
    cataglyphis_gui.cpp \
    init_step_one.cpp \
    bias_removal_form.cpp \
    ros_workers.cpp \
    generic_error_dialog_form.cpp \
    ../../src/cataglyphis_gui_node.cpp

HEADERS  += \
    cataglyphis_startup_form_main.h \
    map_viewer.h \
    cataglyphis_gui.h \
    init_step_one.h \
    bias_removal_form.h \
    ros_workers.h \
    generic_error_dialog_form.h

FORMS    += \
    cataglyphis_startup_form_main.ui \
    map_viewer.ui \
    cataglyphis_gui.ui \
    init_step_one.ui \
    bias_removal_form.ui \
    generic_error_dialog_form.ui

RESOURCES += \
    resources.qrc

DISTFILES +=

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += roscpp

unix: PKGCONFIG += roslib

