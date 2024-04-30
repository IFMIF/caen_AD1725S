TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += CAENComm.h
INC += CAENVMElib.h
INC += CAENVMEoslib.h
INC += CAENVMEtypes.h
INC += CAENDigitizer.h
INC += CAENDigitizerType.h

ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86_64/libCAENVME.so
LIB_INSTALLS    += ../os/linux-x86_64/libCAENComm.so
LIB_INSTALLS    += ../os/linux-x86_64/libCAENDigitizer.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86/libCAENVME.so
LIB_INSTALLS    += ../os/linux-x86/libCAENComm.so
LIB_INSTALLS    += ../os/linux-x86/libCAENDigitizer.so

else ifeq (linux-aarch64, $(findstring linux-aarch64, $(T_A)))
LIB_INSTALLS    += ../os/linux-aarch64/libCAENVME.so
LIB_INSTALLS    += ../os/linux-aarch64/libCAENComm.so
LIB_INSTALLS    += ../os/linux-aarch64/libCAENDigitizer.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

