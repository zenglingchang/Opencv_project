#-------------------------------------------------
#
# Project created by QtCreator 2018-04-20T14:24:03
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tree_dimentional_reconstruction
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    feature_deal.cpp \
    camera.cpp \
    delaunay.cpp \
    structure.cpp \
    leastsquare.cpp

HEADERS += \
        mainwindow.h \
    feature_deal.h \
    help.h \
    camera.h \
    delaunay.h \
    structure.h \
    leastsquare.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += D:\Qopencv\Eigen
INCLUDEPATH += \
        D:\Qopencv\install\include
        D:\Qopencv\install\include\opencv
        D:\Qopencv\install\include\opencv2


LIBS += -L D:\Qopencv\lib\libopencv_*.a
