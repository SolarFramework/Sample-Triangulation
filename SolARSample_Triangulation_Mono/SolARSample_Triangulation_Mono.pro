## remove Qt dependencies
QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARSample_Triangulation_Mono
VERSION=0.8.2

DEFINES += MYVERSION=$${VERSION}
CONFIG += c++1z
CONFIG += console

include(findremakenrules.pri)

CONFIG(debug,debug|release) {
    TARGETDEPLOYDIR = $${PWD}/../bin/Debug
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    TARGETDEPLOYDIR = $${PWD}/../bin/Release
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

win32:CONFIG -= static
win32:CONFIG += shared

DEPENDENCIESCONFIG = sharedlib recursive install_recurse
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib, QMAKE_TARGET.arch and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/templateappconfig.pri)))  # Shell_quote & shell_path required for visual on windows

DEFINES += BOOST_ALL_NO_LIB
DEFINES += BOOST_ALL_DYN_LINK

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
    QMAKE_COMPILER_DEFINES += _WIN64
}

android {
    ANDROID_ABIS="arm64-v8a"
}


message($${QMAKE_CXXFLAGS})

config_files.path = $${TARGETDEPLOYDIR}
config_files.files=$$files($${PWD}/SolARSample_Triangulation_Mono_conf.xml)\
                     $$files($${PWD}/Image1.png)\
                     $$files($${PWD}/Image2.png)\
                     $$files($${PWD}/camera_calibration.yml)

INSTALLS += config_files

linux {
  run_install.path = $${TARGETDEPLOYDIR}
  run_install.files = $${PWD}/../run.sh
  CONFIG(release,debug|release) {
    run_install.extra = cp $$files($${PWD}/../runRelease.sh) $${PWD}/../run.sh
  }
  CONFIG(debug,debug|release) {
    run_install.extra = cp $$files($${PWD}/../runDebug.sh) $${PWD}/../run.sh
  }
  INSTALLS += run_install
}

config_files.path = $${TARGETDEPLOYDIR}

OTHER_FILES += \
    packagedependencies.txt


#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows

