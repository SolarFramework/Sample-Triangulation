TARGET = SolARTriangulationSample
VERSION=0.5.2

CONFIG += c++1z
CONFIG -= qt
CONFIG += console

DEFINES += MYVERSION=$${VERSION}

CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += NDEBUG=1
}

win32:CONFIG -= static
win32:CONFIG += shared

DEPENDENCIESCONFIG = sharedlib recurse
#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include (../builddefs/qmake/templateappconfig.pri)

HEADERS += \

SOURCES += \
    main.cpp

unix {
    LIBS += -ldl
    QMAKE_CXXFLAGS += -DBOOST_LOG_DYN_LINK
}

macx {
    QMAKE_MAC_SDK= macosx
    QMAKE_CXXFLAGS += -fasm-blocks -x objective-c++
}

linux {
    DEFINES += SOLAR_USE_OPENGL
}

win32 {
    QMAKE_LFLAGS += /MACHINE:X64
    DEFINES += WIN64 UNICODE _UNICODE
    #DEFINES += SOLAR_USE_OPENGL
    QMAKE_COMPILER_DEFINES += _WIN64

    # Windows Kit (msvc2013 64)
#    LIBS += -L$$(WINDOWSSDKDIR)lib/winv6.3/um/x64 -lshell32 -lgdi32 -lComdlg32
#   INCLUDEPATH += $$(WINDOWSSDKDIR)lib/winv6.3/um/x64

}


message($${QMAKE_CXXFLAGS})

DISTFILES += \
    conf_Triangulation.xml

xpcf_xml_files.path = $$(HOME)/.xpcf
xpcf_xml_files.files=$$files($${PWD}/conf_Triangulation.xml)

INSTALLS += xpcf_xml_files
