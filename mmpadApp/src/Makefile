TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_Linux = mmpadDetector
LIBRARY_IOC_Darwin = mmpadDetector
PROD_IOC_Linux += mmpadDetectorApp
PROD_IOC_Darwin += mmpadDetectorApp

USR_INCLUDES += -I$(TOP)/ADApp/cbfSrc

LIB_SRCS += mmpadDetector.cpp
# <name>.dbd will be created from <name>Include.dbd
DBD += mmpadDetectorApp.dbd
DBD += mmpadDetectorSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += mmpadDetectorApp_registerRecordDeviceDriver.cpp mmpadDetectorAppMain.cpp

# Add locally compiled object code
PROD_LIBS += mmpadDetector
PROD_LIBS += cbfad

include $(TOP)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

