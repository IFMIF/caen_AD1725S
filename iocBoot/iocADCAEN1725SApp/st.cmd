< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/ADCAEN1725SApp.dbd")
ADCAEN1725SApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "1725CAEN:")
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
# ADCAEN1725SConfig(const char *portName, int numTimePoints, int dataType,
#                      int maxBuffers, int maxMemory, int priority, int stackSize)
ADCAEN1725SConfig("$(PORT)", 5,$(PID), $(NODE), 0, 0, 3)

dbLoadRecords("$(TOP)/db/ADCAEN1725S.template",  "P=$(PREFIX),R=det1:,  PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:0:,PORT=$(PORT),ADDR=0,TIMEOUT=1,NAME=$(T1)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:1:,PORT=$(PORT),ADDR=1,TIMEOUT=1,NAME=$(T2)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:2:,PORT=$(PORT),ADDR=2,TIMEOUT=1,NAME=$(T3)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:3:,PORT=$(PORT),ADDR=3,TIMEOUT=1,NAME=$(T4)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:4:,PORT=$(PORT),ADDR=4,TIMEOUT=1,NAME=$(T5)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:5:,PORT=$(PORT),ADDR=5,TIMEOUT=1,NAME=$(T6)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:6:,PORT=$(PORT),ADDR=6,TIMEOUT=1,NAME=$(T7)")
dbLoadRecords("$(TOP)/db/ADCAEN1725SN.template", "P=$(PREFIX),R=det1:7:,PORT=$(PORT),ADDR=7,TIMEOUT=1,NAME=$(T8)")

# Create a standard arrays plugin, set it to get data from ADCSDetector driver.
NDStdArraysConfigure("Array1", 3, 0, "$(PORT)", 0)
# This creates a waveform large enough for 100000x8 arrays.
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=Array1:,PORT=Array1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=1152")

#NDTimeSeriesConfigure("TS1", $(QSIZE), 0, "$(PORT)", 0, 8)
#dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template",  "P=$(PREFIX),R=TS:,   PORT=TS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,NCHANS=$(TSPOINTS),TIME_LINK=$(PREFIX)det1:TimeStep CP MS,ENABLED=1")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:1:, PORT=TS1,ADDR=0,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T1)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:2:, PORT=TS1,ADDR=1,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T2)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:3:, PORT=TS1,ADDR=2,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T3)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:4:, PORT=TS1,ADDR=3,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T4)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:5:, PORT=TS1,ADDR=4,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T5)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:6:, PORT=TS1,ADDR=5,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T6)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:7:, PORT=TS1,ADDR=6,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T7)")
#dbLoadRecords("$(ADCORE)/db/NDTimeSeriesN.template", "P=$(PREFIX),R=TS:8:, PORT=TS1,ADDR=7,TIMEOUT=1,NCHANS=$(TSPOINTS),NAME=$(T8)")

iocInit()
