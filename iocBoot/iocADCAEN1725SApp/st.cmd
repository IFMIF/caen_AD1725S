< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/ADCAEN1725SApp.dbd")
ADCAEN1725SApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "DIAG:")
# R for all records
epicsEnvSet("RADIX", "CAEN_DET1")
# The port name for the detector
epicsEnvSet("PORT",   "mod0")
# The PID of the CAEN CONET adaptor
epicsEnvSet("PID", "16437")
# The PID of the CAEN CONET adaptor
epicsEnvSet("NODE", "2")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")

# Maximum time points
epicsEnvSet("YSIZE",  "144")
epicsEnvSet("TSPOINTS", "2048")

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("T1", "Chan1")
epicsEnvSet("T2", "Chan2")
epicsEnvSet("T3", "Chan3")
epicsEnvSet("T4", "Chan4")
epicsEnvSet("T5", "Chan5")
epicsEnvSet("T6", "Chan6")
epicsEnvSet("T7", "Chan7")
epicsEnvSet("T8", "Chan8")

# Create an ADCAEN1725S driver
ADCAEN1725SConfig("$(PORT)", 5,$(PID), $(NODE), 0, 0, 3)

dbLoadRecords("$(TOP)/db/ADCAEN1725S.template",  "P=$(PREFIX),R=$(RADIX):,  PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_0:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(T1)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_1:,PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(T2)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_2:,PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(T3)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_3:,PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(T4)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_4:,PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(T5)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_5:,PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(T6)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_6:,PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(T7)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=$(RADIX)_7:,PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(T8)")

# Create a standard arrays plugin, set it to get data from ADCSDetector driver.
NDStdArraysConfigure("Array1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 100000x8 arrays.
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=Array1:,PORT=Array1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=1152")

iocInit()