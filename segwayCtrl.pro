QMAKE_CXXFLAGS += -std=c++11 -pg -Wno-reorder
QMAKE_LFLAGS += -pg
TRIKCONTROL_BINDIR = ../trikRuntime/bin/release
TRIKCONTROL_DIR = ../trikRuntime/trikControl

QT       += core
QT       += gui
QT       += network
TARGET = segwayCtrl
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH = \
        $$PWD \
        $$TRIKCONTROL_DIR/include \

LIBS += -L$$TRIKCONTROL_BINDIR -ltrikControl

!macx {
        QMAKE_LFLAGS += -Wl,-O2,-rpath,.
}

HEADERS += \
    segway.h \
    encodersReader.h \
    stabilizer.h

SOURCES += main.cpp \
    segway.cpp \
    encodersReader.cpp \
    stabilizer.cpp
