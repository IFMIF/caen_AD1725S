TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library


DBD += ADCAEN1725S.dbd
PROD_NAME = ADCAEN1725SApp

ifeq ($(OS_CLASS), Linux)
ifeq ($(ARCH_CLASS), x86_64)
  LIBRARY_IOC_Linux += ADCAEN1725S
  PROD_IOC_Linux  += $(PROD_NAME)
endif
ifeq ($(ARCH_CLASS), aarch64)
  LIBRARY_IOC_Linux += ADCAEN1725S
  PROD_IOC_Linux  += $(PROD_NAME)
endif
endif
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  LIBRARY_IOC_WIN32   += ADCAEN1725S
  PROD_IOC_WIN32  += $(PROD_NAME)
endif

LIB_SRCS += ADCAEN1725S.cpp

LIB_LIBS += asyn
LIB_LIBS += ADBase
LIB_LIBS += CAENComm
LIB_LIBS += CAENVME
LIB_LIBS += CAENDigitizer
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ADCAEN1725S.dbd

## <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS   += $(PROD_NAME)_registerRecordDeviceDriver.cpp
$(PROD_NAME)_SRCS   += $(PROD_NAME)Main.cpp
$(PROD_NAME)_LIBS   += ADCAEN1725S


include $(ADCORE)/ADApp/commonDriverMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

