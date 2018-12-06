TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_Linux = mmpadDetector
LIBRARY_IOC_Darwin = mmpadDetector

USR_INCLUDES += -I$(TOP)/ADApp/cbfSrc

LIB_SRCS += mmpadDetector.cpp
DBD += mmpadDetectorSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

