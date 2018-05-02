#-------------------------------------------------
#
# Project created by QtCreator 2018-04-11T16:19:17
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = opencv_1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    histogram.cpp

HEADERS  += mainwindow.h \
    help.h \
    histogram.h

FORMS    += mainwindow.ui

INCLUDEPATH += \
        D:\Qopencv\install\include
        D:\Qopencv\install\include\opencv
        D:\Qopencv\install\include\opencv2

LIBS +=  D:\Qopencv\lib\libopencv_*.a
