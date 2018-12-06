< envPaths
errlogInit(20000)

dbLoadDatabase("$(AREA_DETECTOR)/dbd/mmpadDetectorApp.dbd")
mmpadDetectorApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "MMPAD3x2:")
epicsEnvSet("PORT",   "MMPAD")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "487")
epicsEnvSet("YSIZE",  "195")
epicsEnvSet("NCHANS", "2048")

###
# Create the asyn port to talk to the Pilatus on port 41234.
drvAsynIPPortConfigure("camserver","127.0.0.1:41234")
#drvAsynIPPortConfigure("camserver","10.0.1.150:41234")
#drvAsynIPPortConfigure("camserver","gse-pilatus1:41234")
# Set the input and output terminators.
asynOctetSetInputEos("camserver", 0, "\030")
asynOctetSetOutputEos("camserver", 0, "\n")

mmpadDetectorConfig("$(PORT)", "camserver", $(XSIZE), $(YSIZE), 50, 200000000)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/ADBase.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/mmpad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,CAMSERVER_PORT=camserver")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, -1)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=94965")

# Load all other plugins using commonPlugins.cmd
< ../commonPlugins.cmd

#asynSetTraceMask("$(PORT)",0,255)
#asynSetTraceMask("$(PORT)",0,3)


iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
