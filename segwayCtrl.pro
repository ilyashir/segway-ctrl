TRIKCONTROL_BINDIR = ../trikRuntime/trikControl/bin/release
TRIKCONTROL_DIR = ../trikRuntime/trikControl

QT       += core
QT       -= gui
QT       += network
TARGET = segwayCtrl
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH = \
        $$PWD \
        $$TRIKCONTROL_DIR/include \

LIBS += -L$$TRIKCONTROL_BINDIR -ltrikControl

SOURCES += main.cpp \
    segway.cpp

HEADERS += \
    segway.h
