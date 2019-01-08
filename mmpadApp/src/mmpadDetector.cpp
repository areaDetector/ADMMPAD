/* mmpadDetector.cpp
 *
 * This is a modification of the pilatus detector source code to support the
 * mmpad detector.
 * Fresh Modification Begins:  26 Jan 2012
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <tiffio.h>
#include <iostream>
#include <sstream>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <cantProceed.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

#include <epicsExport.h>

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

/** Messages to/from camserver */
#define MAX_MESSAGE_SIZE 1024
#define MAX_FILENAME_LEN 256
/** Time to poll when reading from camserver */
#define ASYN_POLL_TIME .01
#define CAMSERVER_DEFAULT_TIMEOUT 1.0
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

// MMPAD readout time.  This is for the model at APS
#define MMPAD_READOUT_TIME 858e-6

using namespace std;

/** Trigger modes */
typedef enum
{
    TMeth,
    TMbnc,
    TMcamlink
//  TMInternal,
//  TMExternalEnable,
//  TMExternalTrigger,
//  TMMultipleExternalTrigger
} mmpadTriggerMode_t;

/** Image modes */
typedef enum
{
    mmpadImageModeNormal,
    mmpadImageModeVideo
} mmpadImageMode_t;

/** File format */
typedef enum
{
    mmpadFileFormatRaw,
    mmpadFileFormatTIFF
} mmpadFileFormat_t;

static const char *driverName = "mmpadDetector";

#define mmpadDelayTimeString      "DELAY_TIME"
#define mmpadArmedString          "ARMED"
#define mmpadImageFileTmotString  "IMAGE_FILE_TMOT"

#define mmpadAVGAcquireString     "AVGACQUIRE"
#define mmpadAVGFileString        "AVGFILE"
#define mmpadAVGCountString       "AVGCOUNT"

#define mmpadBGSubtractString     "BG_SUBTRACT"
#define mmpadBGFileString         "BGFILE"
#define mmpadBackSubFlagString    "BACKSUB_FLAG"

#define mmpadMilDispBitShiftString "MILDISP_BITSHIFT"
#define mmpadMilDispOnString      "MILDISP_ON"
#define mmpadMilDispLogString     "MILDISP_LOG"
#define mmpadMilDispOffsetString  "MILDISP_OFFSET"
#define mmpadMilDispScaleString   "MILDISP_SCALE"

#define mmpadRoiSumString         "ROI_SUM"
#define mmpadRoiULString          "ROI_UL"
#define mmpadRoiURString          "ROI_UR"
#define mmpadRoiLLString          "ROI_LL"
#define mmpadRoiLRString          "ROI_LR"

/** Driver for mmpad pixel array detectors using their camserver server over TCP/IP socket */
class mmpadDetector : public ADDriver
{
  public:
    mmpadDetector(const char *portName, const char *camserverPort,
                  int maxSizeX, int maxSizeY,
                  int maxBuffers, size_t maxMemory,
                  int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value,
                                  size_t nChars, size_t *nActual);
    void report(FILE *fp, int details);
    void mmpadTask(); /* This should be private but is called from C so must be public */

  protected:
    int mmpadDelayTime;
#define FIRST_mmpad_PARAM mmpadDelayTime
    int mmpadArmed;
    int mmpadImageFileTmot;

    int mmpadBGSubtract;
    int mmpadBGFile;
    int mmpadBackSubFlag;

    int mmpadMilDispBitShift;
    int mmpadMilDispOn;
    int mmpadMilDispLog;
    int mmpadMilDispOffset;
    int mmpadMilDispScale;

    int mmpadAVGAcquire;
    int mmpadAVGFile;
    int mmpadAVGCount;
    
    int mmpadRoiSum;
    int mmpadRoiUL;
    int mmpadRoiUR;
    int mmpadRoiLL;
    int mmpadRoiLR;

  private:
    /* These are the methods that are new to this class */
    void makeMultipleFileFormat(const char *baseFileName);
    asynStatus waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    int stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase);
    asynStatus readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus writeCamserver(double timeout);
    asynStatus readCamserver(double timeout);
    asynStatus writeReadCamserver(double timeout);
    asynStatus setAcquireParams();
    void readFlatFieldFile(const char *flatFieldFile);

    /* Our data */
    int imagesRemaining;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    char toCamserver[MAX_MESSAGE_SIZE];
    char fromCamserver[MAX_MESSAGE_SIZE];
    NDArray *pFlatField;
    char multipleFileFormat[MAX_FILENAME_LEN];
    int multipleFileNumber;
    asynUser *pasynUserCamserver;
    double averageFlatField;
    string camserveInfo;
    //stringstream *sstr;
};

void mmpadDetector::makeMultipleFileFormat(const char *baseFileName)
{
    /* This function uses the code from camserver */
    char *p, *q;
    int fmt;
    char mfTempFormat[MAX_FILENAME_LEN];
    char mfExtension[10];
    int numImages;
    
    /* baseFilename has been built by the caller.
     * Copy to temp */
    strncpy(mfTempFormat, baseFileName, sizeof(mfTempFormat));
    getIntegerParam(ADNumImages, &numImages);
    p = mfTempFormat + strlen(mfTempFormat) - 5; /* look for extension */
    if ( (q=strrchr(p, '.')) ) {
        strcpy(mfExtension, q);
        *q = '\0';
    } else {
        strcpy(mfExtension, ""); /* default is raw image */
    }
    multipleFileNumber=0;   /* start number */
    fmt=5;        /* format length */
    if ( !(p=strrchr(mfTempFormat, '/')) ) {
        p=mfTempFormat;
    }
    if ( (q=strrchr(p, '_')) ) {
        q++;
        if (isdigit(*q) && isdigit(*(q+1)) && isdigit(*(q+2))) {
            multipleFileNumber=atoi(q);
            fmt=0;
            p=q;
            while(isdigit(*q)) {
                fmt++;
                q++;
            }
            *p='\0';
            if (((fmt<3)  || ((fmt==3) && (numImages>999))) || 
                ((fmt==4) && (numImages>9999))) { 
                fmt=5;
            }
        } else if (*q) {
            strcat(p, "_"); /* force '_' ending */
        }
    } else {
        strcat(p, "_"); /* force '_' ending */
    }
    /* Build the final format string */
    epicsSnprintf(this->multipleFileFormat, sizeof(this->multipleFileFormat), "%s%%.%dd%s",
                  mfTempFormat, fmt, mfExtension);
}


/** This function waits for the specified file to exist.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus mmpadDetector::waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    int fd=-1;
    int fileExists=0;
    struct stat statBuff;
    epicsTimeStamp tStart, tCheck;
    time_t acqStartTime;
    double deltaTime=0.;
    int status=-1;
    const char *functionName = "waitForFileToExist";

    if (pStartTime) epicsTimeToTime_t(&acqStartTime, pStartTime);
    epicsTimeGetCurrent(&tStart);

    while (deltaTime <= timeout) {
        fd = open(fileName, O_RDONLY, 0);
        if ((fd >= 0) && (timeout != 0.)) {
            fileExists = 1;
            /* The file exists.  Make sure it is a new file, not an old one.
             * We don't do this check if timeout==0, which is used for reading flat field files */
            status = fstat(fd, &statBuff);
            if (status){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s error calling fstat, errno=%d %s\n",
                    driverName, functionName, errno, fileName);
                close(fd);
                return(asynError);
            }
            /* We allow up to 10 second clock skew between time on machine running this IOC
             * and the machine with the file system returning modification time */
            if (difftime(statBuff.st_mtime, acqStartTime) > -10) break;
            close(fd);
            fd = -1;
        }
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        lock();
        if (status == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }
    if (fd < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s timeout waiting for file to be created %s\n",
            driverName, functionName, fileName);
        if (fileExists) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "  file exists but is more than 10 seconds old, possible clock synchronization problem\n");
            setStringParam(ADStatusMessage, "Image file is more than 10 seconds old");
        } else
            setStringParam(ADStatusMessage, "Timeout waiting for file to be created");
        return(asynError);
    }
    close(fd);
    return(asynSuccess);
}


int mmpadDetector::stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase)
{
    int i, j;

    i = strlen(aString) - 1;
    j = strlen(aSubstring) - 1;
    while (i >= 0 && j >= 0) {
        if (shouldIgnoreCase) {
            if (tolower(aString[i]) != tolower(aSubstring[j])) return 0;
        } else {
            if (aString[i] != aSubstring[j]) return 0;
        }
        i--; j--;
    }
    return j < 0;
}


/** This function reads TIFF or RAW image files.  It is not intended to be general, it
 * is intended to read the TIFF or RAW files that camserver creates.  It checks to make
 * sure that the creation time of the file is after a start time passed to it, to force
 * it to wait for a new file to be created.
 */
asynStatus mmpadDetector::readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    //static const char *functionName = "readImageFile";
    char tifFileName[MAX_FILENAME_LEN];

    if (stringEndsWith(fileName, ".tif", 1)) {
        return readTiff(fileName, pStartTime, timeout, pImage);
    } else {
        strcpy(tifFileName,fileName);
        strncat(tifFileName,".tif",MAX_FILENAME_LEN);
        return readTiff(tifFileName, pStartTime, timeout, pImage);
    }
}

/** This function reads TIFF files using libTiff.  It is not intended to be general,
 * it is intended to read the TIFF files that camserver creates.  It checks to make sure
 * that the creation time of the file is after a start time passed to it, to force it to
 * wait for a new file to be created.
 */
asynStatus mmpadDetector::readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    int status=-1;
    const char *functionName = "readTiff";
    size_t totalSize;
    int size;
    int numStrips, strip;
    char *buffer;
    char *imageDescription;
    char tempBuffer[2048];
    TIFF *tiff=NULL;
    epicsUInt32 uval;

    deltaTime = 0.;
    epicsTimeGetCurrent(&tStart);

    /* Suppress error messages from the TIFF library */
    TIFFSetErrorHandler(NULL);
    TIFFSetWarningHandler(NULL);

    status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
    if (status != asynSuccess) {
        return((asynStatus)status);
    }

    while (deltaTime <= timeout) {
        /* At this point we know the file exists, but it may not be completely written yet.
         * If we get errors then try again */
        tiff = TIFFOpen(fileName, "rc");
        if (tiff == NULL) {
            status = asynError;
            goto retry;
        }
        
        /* Do some basic checking that the image size is what we expect */
        status = TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[0].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image width incorrect =%u, should be %u\n",
                driverName, functionName, uval, (epicsUInt32)pImage->dims[0].size);
            goto retry;
        }
        status = TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[1].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image length incorrect =%u, should be %u\n",
                driverName, functionName, uval, (epicsUInt32)pImage->dims[1].size);
            goto retry;
        }
        numStrips= TIFFNumberOfStrips(tiff);
        buffer = (char *)pImage->pData;
        totalSize = 0;
        for (strip=0; (strip < numStrips) && (totalSize < pImage->dataSize); strip++) {
            size = TIFFReadEncodedStrip(tiff, 0, buffer, pImage->dataSize-totalSize);
            if (size == -1) {
                /* There was an error reading the file.  Most commonly this is because the file
                 * was not yet completely written.  Try again. */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s, error reading TIFF file %s\n",
                    driverName, functionName, fileName);
                goto retry;
            }
            buffer += size;
            totalSize += size;
        }
        if (totalSize != pImage->dataSize) {
            status = asynError;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, file size incorrect =%lu, should be %lu\n",
                driverName, functionName, (unsigned long)totalSize, (unsigned long)pImage->dataSize);
            goto retry;
        }
        /* Sucesss! Read the IMAGEDESCRIPTION tag if it exists */
        status = TIFFGetField(tiff, TIFFTAG_IMAGEDESCRIPTION, &imageDescription);
        // Make sure the string is null terminated
        
        if (status == 1) {
            strncpy(tempBuffer, imageDescription, sizeof(tempBuffer));
            // Make sure the string is null terminated
            tempBuffer[sizeof(tempBuffer)-1] = 0;
            pImage->pAttributeList->add("TIFFImageDescription", "TIFFImageDescription", NDAttrString, tempBuffer);
        }
        
        break;
        
        retry:
        if (tiff != NULL) TIFFClose(tiff);
        tiff = NULL;
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        lock();
        if (status == epicsEventWaitOK) {
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    if (tiff != NULL) TIFFClose(tiff);

    return(asynSuccess);
}   

asynStatus mmpadDetector::setAcquireParams()
{
    int numImages;
    double acquireTime;
    double acquirePeriod;
    asynStatus status;
  
    status = getIntegerParam(ADNumImages, &numImages);
    if ((status != asynSuccess) || (numImages < 1)) {
        numImages = 1;
        setIntegerParam(ADNumImages, numImages);
    }
  
    status = getDoubleParam(ADAcquireTime, &acquireTime);
    if ((status != asynSuccess) || (acquireTime < 0.)) {
        acquireTime = 1.;
        setDoubleParam(ADAcquireTime, acquireTime);
    }
    
    status = getDoubleParam(ADAcquirePeriod, &acquirePeriod);
    if ((status != asynSuccess) || (acquirePeriod < acquireTime + MMPAD_READOUT_TIME)) {
        acquirePeriod = acquireTime + MMPAD_READOUT_TIME;
        setDoubleParam(ADAcquirePeriod, acquirePeriod);
    }
    
    double waitTime = acquirePeriod - acquireTime - MMPAD_READOUT_TIME;
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "set_take_n %f %f %d", acquireTime, waitTime, numImages);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    
    setIntegerParam(ADStatus, ADStatusIdle);
    
    return(asynSuccess);
  
  }
  
asynStatus mmpadDetector::writeCamserver(double timeout)
{
    size_t nwrite;
    asynStatus status;
    const char *functionName="writeCamserver";
  
    /* Flush any stale input, since the next operation is likely to be a read */
    status = pasynOctetSyncIO->flush(this->pasynUserCamserver);
    status = pasynOctetSyncIO->write(this->pasynUserCamserver, this->toCamserver,
                                     strlen(this->toCamserver), timeout,
                                     &nwrite);
  
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s, status=%d, sent\n%s\n",
                            driverName, functionName, status, this->toCamserver);
  
    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringToServer, this->toCamserver);
  
    return(status);
}


asynStatus mmpadDetector::readCamserver(double timeout)
{
    size_t nread;
    asynStatus status=asynSuccess;
    int eventStatus;
    asynUser *pasynUser = this->pasynUserCamserver;
    int eomReason;
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    const char *functionName="readCamserver";

    /* We implement the timeout with a loop so that the port does not
     * block during the entire read.  If we don't do this then it is not possible
     * to abort a long exposure */
    deltaTime = 0;
    epicsTimeGetCurrent(&tStart);
    while (deltaTime <= timeout) {
        unlock();
        status = pasynOctetSyncIO->read(pasynUser, this->fromCamserver,
                                        sizeof(this->fromCamserver), ASYN_POLL_TIME,
                                        &nread, &eomReason);
        /* Check for an abort event sent during a read. Otherwise we can miss it and mess up the next acquisition.*/
        eventStatus = epicsEventWaitWithTimeout(this->stopEventId, 0.001);
        lock();
        if (eventStatus == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        if (status != asynTimeout) break;

        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        unlock();
        eventStatus = epicsEventWaitWithTimeout(this->stopEventId, ASYN_POLL_TIME);
        lock();
        if (eventStatus == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            setIntegerParam(ADStatus, ADStatusAborted);
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    // If we got asynTimeout, and timeout=0 then this is not an error, it is a poll checking for possible reply and we are done
   if ((status == asynTimeout) && (timeout == 0)) return(asynSuccess);
   if (status != asynSuccess)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, timeout=%f, status=%d received %lu bytes\n%s\n",
                    driverName, functionName, timeout, status, (unsigned long)nread, this->fromCamserver);
   else {
        /* Look for the string OK in the response */
        if (!strstr(this->fromCamserver, "OK")) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s unexpected response from camserver, no OK, response=%s\n",
                      driverName, functionName, this->fromCamserver);
            setStringParam(ADStatusMessage, "Error from camserver");
            status = asynError;
        } else
            setStringParam(ADStatusMessage, "Camserver returned OK");
    }

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringFromServer, this->fromCamserver);

    return(status);
}


asynStatus mmpadDetector::writeReadCamserver(double timeout)
{
    asynStatus status;
  
    status = writeCamserver(timeout);
    if (status) return status;
    status = readCamserver(timeout);
    return status;
}

static void mmpadTaskC(void *drvPvt)
{
    mmpadDetector *pPvt = (mmpadDetector *)drvPvt;
  
    pPvt->mmpadTask();
}

/** This thread controls acquisition, reads image files to get the image data, and
  * does the callbacks to send it to higher layers */
void mmpadDetector::mmpadTask()
{
    int status = asynSuccess;
    int stat2;
//  int imageCounter;
    int numImages;
    int multipleFileNextImage=0;  /* This is the next image number, starting at 0 */
    int acquire;
    ADStatus_t acquiring;
//  NDArray *pImage;
    double acquireTime, acquirePeriod;
    double readImageFileTimeout, timeout;
    int triggerMode;
    epicsTimeStamp startTime;
    const char *functionName = "mmpadTask";
    char fullFileName[MAX_FILENAME_LEN];
    int fileFormat;
//  char statusMessage[MAX_MESSAGE_SIZE];
//  int dims[2];
    int arrayCallbacks;
    int roisum,roiul,roiur,roill,roilr;
    string a;
    stringstream sstr;
    int testDone;
    int retryFlag;
    testDone=0;
  
    this->lock();
  
    /* Loop forever */
    while (1) {
        /* Is acquisition active? */
        stat2 = getIntegerParam(ADAcquire, &acquire);
  
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            /* Only set the status message if we didn't encounter any errors last time, so we don't overwrite the
             error message */
            if (!status)
                stat2 = setStringParam(ADStatusMessage, "Waiting for acquire command");
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: waiting for acquire to start\n", driverName, functionName);
            status = epicsEventWait(this->startEventId);
            this->lock();
            stat2 = getIntegerParam(ADAcquire, &acquire);
        }
  
        /* We are acquiring. */
        /* Get the current time */
        epicsTimeGetCurrent(&startTime);
  
        /* Get the exposure parameters */
        stat2 = getDoubleParam(ADAcquireTime, &acquireTime);
        stat2 = getDoubleParam(ADAcquirePeriod, &acquirePeriod);
        stat2 = getDoubleParam(mmpadImageFileTmot, &readImageFileTimeout);
  
        /* Get the acquisition parameters */
        stat2 = getIntegerParam(ADTriggerMode, &triggerMode);
        stat2 = getIntegerParam(ADNumImages, &numImages);
  
        acquiring = ADStatusAcquire;
        stat2 = setIntegerParam(ADStatus, acquiring);
  
        createFileName(sizeof(fullFileName), fullFileName);
        stat2 = callParamCallbacks();

        getIntegerParam(NDFileFormat, &fileFormat);
        if (fileFormat == mmpadFileFormatRaw) {
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"filestore 1 5");
        } else {
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"filestore 1 1");
        }
        status = writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"Exposure %s", fullFileName);
        stat2 = setStringParam(ADStatusMessage, "Starting exposure");
        /* Send the acquire command to camserver and wait for the 15OK response */
        writeReadCamserver(2.0);
  
        
        /* If the status wasn't asynSuccess or asynTimeout, report the error */
        if (status>1) {
            acquire = 0;
        } else {
            /* Set status back to asynSuccess as the timeout was expected */
            status = asynSuccess;
            /* Open the shutter */
            setShutter(1);
            /* Set the armed flag */
            stat2 = setIntegerParam(mmpadArmed, 1);
            /* Create the format string for constructing file names for multi-image collection */
            makeMultipleFileFormat(fullFileName);
            multipleFileNextImage = 0;
            /* Call the callbacks to update any changes */
            setStringParam(NDFullFileName, fullFileName);
            callParamCallbacks();
        }
  
        while (acquire) {
            if (1) {
                /* For single frame or alignment mode need to wait for 7OK response from camserver
                 * saying acquisition is complete before trying to read file, else we get a
                 * recent but stale file. */
                //setStringParam(ADStatusMessage, "Waiting for 7OK response");
                callParamCallbacks();
                /* We release the mutex when waiting for 7OK because this takes a long time and
                 * we need to allow abort operations to get through */
                //this->unlock();
                //status = readCamserver(numImages*(acquireTime+0.001) + readImageFileTimeout+5);
                //this->lock();
                retryFlag=1;
                
                //if (status != asynTimeout)
                
                while(retryFlag) {
                    status = readCamserver(2*numImages*(acquireTime+0.001)+10);
                    if (status != asynTimeout) {
                        this->camserveInfo = string(this->fromCamserver);
                        sstr.clear();
                        sstr.str("");
  
                        sstr<<this->fromCamserver;
  
                        while(!sstr.eof()) {
                            sstr>>a;
                            if (a.compare("ERR")==0) {
                                retryFlag=0;
                                while(!sstr.eof()) {
                                    sstr>>a;
                                    if(a.compare("retry")==0) {
                                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "...Got retry message...\n");
                                        retryFlag=1;
                                    }
                                }     
                            }
                            else if(a.compare("OK")==0) {
                                retryFlag=0;
                            }
                        }
                    } else {       
                        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "... Got TimeOut ...\n");
                        retryFlag=0;
                        
                    }
                }

                /* If there was an error jump to bottom of loop */
                if (status) {
                    acquire = 0;
                    if(status==asynTimeout)
                        stat2 = setStringParam(ADStatusMessage, "Timeout waiting for camserver response");
                    continue;
                  }
//              else if(!status) {
//              }
            } else {
                /* If this is a multi-file acquisition the file name is built differently */
                epicsSnprintf(fullFileName, sizeof(fullFileName), multipleFileFormat,
                              multipleFileNumber);
                stat2 = setStringParam(NDFullFileName, fullFileName);
            }
            stat2 = getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
  
            //if (arrayCallbacks && numImages==1)
              //{
                //stat2 = getIntegerParam(NDArrayCounter, &imageCounter);
                //imageCounter++;
                //stat2 = setIntegerParam(NDArrayCounter, imageCounter);
                // /* Call the callbacks to update any changes */
                //stat2 = callParamCallbacks();
  
                // /* Get an image buffer from the pool */
                //stat2 = getIntegerParam(ADMaxSizeX, &dims[0]);
                //stat2 = getIntegerParam(ADMaxSizeY, &dims[1]);
                //pImage = this->pNDArrayPool->alloc(2, dims, NDInt32, 0, NULL);
                //epicsSnprintf(statusMessage, sizeof(statusMessage), "Reading image file %s", fullFileName);
                //stat2 = setStringParam(ADStatusMessage, statusMessage);
                //stat2 = callParamCallbacks();
                // /* We release the mutex when calling readImageFile, because this takes a long time and
                 //* we need to allow abort operations to get through */
                //this->unlock();
                //status = readImageFile(fullFileName, &startTime, acquireTime + readImageFileTimeout, pImage);
                //this->lock();
                // /* If there was an error jump to bottom of loop */
                //if (status)
                  //{
                    //acquire = 0;
                    //pImage->release();
                    //continue;
                  //}
  
                //stat2 = getIntegerParam(mmpadFlatFieldValid, &flatFieldValid);
                //if (flatFieldValid)
                  //{
                    //epicsInt32 *pData, *pFlat, i;
                    //for (i=0, pData = (epicsInt32 *)pImage->pData, pFlat = (epicsInt32 *)this->pFlatField->pData;
                         //i<dims[0]*dims[1];
                         //i++, pData++, pFlat++)
                      //{
                        //*pData = (epicsInt32)((this->averageFlatField * *pData) / *pFlat);
                      //}
                  //}
                // /* Put the frame number and time stamp into the buffer */
                //pImage->uniqueId = imageCounter;
                //pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
  
                // /* Get any attributes that have been defined for this driver */
                //this->getAttributes(pImage->pAttributeList);
  
                // /* Call the NDArray callback */
                // /* Must release the lock here, or we can get into a deadlock, because we can
                 //* block on the plugin lock, and the plugin can be calling us */
                //this->unlock();
                //asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          //"%s:%s: calling NDArray callback\n", driverName, functionName);
                //doCallbacksGenericPointer(pImage, NDArrayData, 0);
                //this->lock();
                // /* Free the image buffer */
                //pImage->release();
            //}
            if (1) {
                acquire = 0;
            }
//          else if (numImages > 1) {
//              multipleFileNextImage++;
//              multipleFileNumber++;
//              if (multipleFileNextImage == numImages) acquire = 0;
//            }
  
        }

        //GETTING AVERAGE DATA FROM PAD TO SEND TO EPICS VARIABLES HERE.
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"GetComputation");
        status = writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
        //status=readCamserver(0.0);
  
        this->camserveInfo = string(this->fromCamserver);
        sstr.clear();
        sstr.str("");
  
        sstr<<this->fromCamserver;
  
        while(!sstr.eof()) {
            sstr>>a;
            if(a.compare("SUM")==0) {
                sstr>>roisum;
            } else if(a.compare("UL")==0) {
                sstr>>roiul;
            } else if(a.compare("UR")==0) {
                sstr>>roiur;
            } else if(a.compare("LR")==0) {
                sstr>>roilr;
            } else if(a.compare("LL")==0) {
                sstr>>roill;
            }
        }
  
        stat2=readCamserver(CAMSERVER_DEFAULT_TIMEOUT);
        status |= setIntegerParam(mmpadRoiSum,roisum);
        status |= setIntegerParam(mmpadRoiUL,roiul);
        status |= setIntegerParam(mmpadRoiUR,roiur);
        status |= setIntegerParam(mmpadRoiLL,roill);
        status |= setIntegerParam(mmpadRoiLR,roilr);
        stat2 = callParamCallbacks();
        /* We are done acquiring */
        /* Wait for the 7OK response from camserver in the case of multiple images */
        if ((numImages > 1) && (status == asynSuccess)) {
            /* If arrayCallbacks is 0we will have gone through the above loop without waiting
             * for each image file to be written.  Thus, we may need to wait a long time for
             * the 7OK response.
             * If arrayCallbacks is 1 then the response should arrive fairly soon. */
            if (arrayCallbacks)
                timeout = readImageFileTimeout;
            else
                timeout = numImages * acquireTime + readImageFileTimeout;
            stat2 = setStringParam(ADStatusMessage, "Waiting for 7OK response");
            stat2 = callParamCallbacks();
            /* We release the mutex because we may wait a long time and need to allow abort
             * operations to get through */
//            this->unlock();
//          readCamserver(timeout);
//            this->lock();
        }
        setShutter(0);
        stat2 = setIntegerParam(ADAcquire, 0);
        stat2 = setIntegerParam(mmpadArmed, 0);
  
        /* If everything was ok, set the status back to idle */
        if (!status)
            stat2 = setIntegerParam(ADStatus, ADStatusIdle);
        else
            stat2 = setIntegerParam(ADStatus, ADStatusError);
  
        /* Call the callbacks to update any changes */
        stat2 = callParamCallbacks();
    }
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADTriggerMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus mmpadDetector::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    asynStatus status = asynSuccess;
    double videomodeTime;
    char fullFileName[MAX_FILENAME_LEN];
    char filePath[MAX_FILENAME_LEN];
    char fileName[MAX_FILENAME_LEN];
    int imgcount;
    int milbitshift, milon,miloffset, millog, milscale;
    int testDone;
    int acquire;
    int imageMode;
    string a;
    stringstream sstr;
    const char *functionName = "writeInt32";

    getIntegerParam(ADAcquire, &acquire);
    status = setIntegerParam(function, value);
  
    if (function == ADAcquire) {
        getIntegerParam(ADStatus, &adstatus);
        getIntegerParam(ADImageMode, &imageMode);
        if (value && !acquire) {
            if (imageMode == mmpadImageModeNormal) {
                /* Send an event to wake up the mmpad task.  */
                epicsEventSignal(this->startEventId);
            } else {
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeon %f", videomodeTime);
                writeReadCamserver(0.2); 
            }
        }
        if (!value && (acquire)) {
            /* This was a command to stop acquisition */
            if (imageMode == mmpadImageModeNormal) {
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "K");
                writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand reset_frame");
                writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
                epicsEventSignal(this->stopEventId);
            } else {
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeoff");
                writeReadCamserver(0.2);
            }
        }
    } else if ((function == ADTriggerMode) ||
               (function == ADNumImages) ||
               (function == ADNumExposures)) {
        setAcquireParams();
    } else if (function == mmpadAVGAcquire) {
        testDone = 1;
        if (value) {
            // need to get file name and path -> make string
            getStringParam(NDFilePath, sizeof(filePath), filePath);
            getStringParam(mmpadAVGFile, sizeof(fileName), fileName);
            strcpy(fullFileName,strncat(filePath,fileName,MAX_FILENAME_LEN));
            // need to retrieve file count.
            getIntegerParam(mmpadAVGCount,&imgcount);
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "avgexp %s %d",fullFileName, imgcount);
            writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
            //So - everytime it is called and the value is not zero - it takes and average.  done?
            // or do I need to set it back to zero?
            while(!testDone) {
                readCamserver(CAMSERVER_DEFAULT_TIMEOUT);
                this->camserveInfo = string(this->fromCamserver);
                sstr.clear();
                sstr.str("");
                while(!sstr.eof()) {
                    sstr>>a;
                    if (a.compare("5")==0) {
                        testDone = 1;
                        sstr>>a;
///                     if(a.compare("OK")) {
////                    } else if((a.compare("ERR")) {
////                    }
                    }   
                }
            }
              
            setIntegerParam(mmpadAVGAcquire, 0);  
        }
    }
    
    else if (function == mmpadBackSubFlag) {
        if (!value) {
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbacksub 0");
            writeReadCamserver(0.2);
            //writeReadCamserver(0);
            //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
        } else {
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbacksub 1");
            writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
             //writeReadCamserver(0);
             //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
        }
    }
    
    else if ((function == mmpadMilDispBitShift) ||
             (function == mmpadMilDispOn) ||
             (function == mmpadMilDispOffset)) {
        getIntegerParam(mmpadMilDispBitShift,&milbitshift);
        getIntegerParam(mmpadMilDispOn,&milon);
        getIntegerParam(mmpadMilDispOffset,&miloffset);
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand mildisp %d %d %d",milbitshift,milon,miloffset);
        writeReadCamserver(0.2);
    }
    
    else if ((function == mmpadMilDispLog) ||
             (function == mmpadMilDispScale) ||
             (function == mmpadMilDispOffset)) {
        getIntegerParam(mmpadMilDispLog,&millog);
        getIntegerParam(mmpadMilDispScale,&milscale);
        getIntegerParam(mmpadMilDispOffset,&miloffset);
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand loglin %d %d %d",millog,milscale,miloffset);
        writeReadCamserver(0.2);
    }

    else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_mmpad_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
  
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
  
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%d\n",
                driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%d\n",
                driverName, functionName, function, value);
    return status;
}


/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus mmpadDetector::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeFloat64";
    int imageMode;
  
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);
  
    getIntegerParam(ADImageMode, &imageMode);
  
    if ((function == ADAcquireTime) && (imageMode == mmpadImageModeVideo)) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeon %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if ((function == ADAcquireTime) ||
        /* Changing any of the following parameters requires recomputing the base image */
        (function == ADAcquirePeriod) ||
        (function == mmpadDelayTime)) {
        setAcquireParams();
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_mmpad_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }
  
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s error, status=%d function=%d, value=%f\n",
                driverName, functionName, status, function, value);
    else
      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%f\n",
                driverName, functionName, function, value);
    return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including mmpadBadPixelFile, ADFilePath, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus mmpadDetector::writeOctet(asynUser *pasynUser, const char *value,
                                     size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";
    char fullFileName[MAX_FILENAME_LEN];
    char filePath[MAX_FILENAME_LEN];
    //  char fileName[MAX_FILENAME_LEN];
  
    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);
      
    if (function == NDFilePath) {
        this->checkPath();
    }
    else if(function == mmpadBGFile) {
        getStringParam(NDFilePath, sizeof(filePath), filePath);
          strcpy(fullFileName,strncat(filePath,value,MAX_FILENAME_LEN));
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbackimg %s 0", fullFileName);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else {
        if (function == NDFileName) {
            status = (asynStatus)setIntegerParam(NDFileNumber, 0);
            status = (asynStatus)callParamCallbacks();
        }
        if (function < FIRST_mmpad_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual); 
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();
  
    if (status)
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                    "%s:%s: status=%d, function=%d, value=%s",
                    driverName, functionName, status, function, value);
    else
      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%s\n",
                driverName, functionName, function, value);
    *nActual = nChars;
    return status;
}


/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void mmpadDetector::report(FILE *fp, int details)
{

    fprintf(fp, "mmpad detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
      }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

extern "C" int mmpadDetectorConfig(const char *portName, const char *camserverPort,
                                   int maxSizeX, int maxSizeY,
                                   int maxBuffers, size_t maxMemory,
                                   int priority, int stackSize)
{
  new mmpadDetector(portName, camserverPort, maxSizeX, maxSizeY, maxBuffers, maxMemory,
                    priority, stackSize);
  return(asynSuccess);
}

/** Constructor for mmpad driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data,
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] camserverPort The name of the asyn port previously created with drvAsynIPPortConfigure to
  *            communicate with camserver.
  * \param[in] maxSizeX The size of the mmpad detector in the X direction.
  * \param[in] maxSizeY The size of the mmpad detector in the Y direction.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
mmpadDetector::mmpadDetector(const char *portName, const char *camserverPort,
                             int maxSizeX, int maxSizeY,
                             int maxBuffers, size_t maxMemory,
                             int priority, int stackSize)

    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
    imagesRemaining(0)

{
    int status = asynSuccess;
    const char *functionName = "mmpadDetector";
    size_t dims[2];
    char versionString[20];
    int milbitshift, milon, miloffset;
  
    /* Create the epicsEvents for signaling to the mmpad task when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        printf("%s:%s epicsEventCreate failure for start event\n",
               driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        printf("%s:%s epicsEventCreate failure for stop event\n",
               driverName, functionName);
        return;
    }
  
    /* Allocate the raw buffer we use to read image files.  Only do this once */
    dims[0] = maxSizeX;
    dims[1] = maxSizeY;
    /* Allocate the raw buffer we use for flat fields. */
    this->pFlatField = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);
  
    /* Connect to camserver */
    status = pasynOctetSyncIO->connect(camserverPort, 0, &this->pasynUserCamserver, NULL);
  
    createParam(mmpadDelayTimeString,      asynParamFloat64, &mmpadDelayTime);
    createParam(mmpadArmedString,          asynParamInt32,   &mmpadArmed);
    createParam(mmpadImageFileTmotString,  asynParamFloat64, &mmpadImageFileTmot);
    
    createParam(mmpadAVGAcquireString,     asynParamInt32,   &mmpadAVGAcquire);
    createParam(mmpadAVGFileString,        asynParamOctet ,  &mmpadAVGFile);
    createParam(mmpadAVGCountString,       asynParamInt32,   &mmpadAVGCount);

    createParam(mmpadBGSubtractString,     asynParamInt32,   &mmpadBGSubtract);
    createParam(mmpadBGFileString,         asynParamOctet ,  &mmpadBGFile);
    createParam(mmpadBackSubFlagString,    asynParamInt32,   &mmpadBackSubFlag);
    
    createParam(mmpadMilDispBitShiftString, asynParamInt32,  &mmpadMilDispBitShift);
    createParam(mmpadMilDispOnString,      asynParamInt32,   &mmpadMilDispOn);
    createParam(mmpadMilDispLogString,     asynParamInt32,   &mmpadMilDispLog);
    createParam(mmpadMilDispOffsetString,  asynParamInt32,   &mmpadMilDispOffset);
    createParam(mmpadMilDispScaleString,   asynParamInt32,   &mmpadMilDispScale);

    createParam(mmpadRoiSumString,         asynParamInt32,   &mmpadRoiSum);
    createParam(mmpadRoiULString,          asynParamInt32,   &mmpadRoiUL);
    createParam(mmpadRoiURString,          asynParamInt32,   &mmpadRoiUR);
    createParam(mmpadRoiLLString,          asynParamInt32,   &mmpadRoiLL);
    createParam(mmpadRoiLRString,          asynParamInt32,   &mmpadRoiLR);
      
    /* Set some default values for parameters */
    status |= setIntegerParam(NDFileNumber, 1);
    status |= (asynStatus)callParamCallbacks();
    status |= setIntegerParam(mmpadAVGCount, 40);
    status |= (asynStatus)callParamCallbacks();

    status |= setStringParam (ADManufacturer, "Cornell");
    status |= setStringParam (ADModel, "mmpad");
    epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", 
                  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    status |= setStringParam(NDDriverVersion, versionString);
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(NDArraySizeX, maxSizeX);
    status |= setIntegerParam(NDArraySizeY, maxSizeY);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType,  NDUInt32);
    status |= setIntegerParam(ADTriggerMode, 2);

    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }
    
    this->lock();  
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"ldcmndfile /home/padme/tvx_64/tvx/camera/camserver/startup.cmd");
    setStringParam(ADStatusMessage, "Initializing Camera");
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT+8);
   
    // ************ getting mildisp to go *********************
    status |= setIntegerParam(mmpadMilDispBitShift, 0);
    status |= setIntegerParam(mmpadMilDispOn, 1);
    status |= setIntegerParam(mmpadMilDispOffset, 40);
    
    status |= (asynStatus)callParamCallbacks();
   
         
    getIntegerParam(mmpadMilDispBitShift,&milbitshift);
    getIntegerParam(mmpadMilDispOn,&milon);
    getIntegerParam(mmpadMilDispOffset,&miloffset);
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand mildisp %d %d %d",milbitshift,milon,miloffset);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    // **************** end getting mildisp to go ******* slight kluge     
         
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("mmpadDetTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)mmpadTaskC,
                                this) == NULL);
    this->unlock();
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n",
               driverName, functionName);
        return;
    }
}

/* Code for iocsh registration */
static const iocshArg mmpadDetectorConfigArg0 = {"Port name", iocshArgString};
static const iocshArg mmpadDetectorConfigArg1 = {"camserver port name", iocshArgString};
static const iocshArg mmpadDetectorConfigArg2 = {"maxSizeX", iocshArgInt};
static const iocshArg mmpadDetectorConfigArg3 = {"maxSizeY", iocshArgInt};
static const iocshArg mmpadDetectorConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg mmpadDetectorConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg mmpadDetectorConfigArg6 = {"priority", iocshArgInt};
static const iocshArg mmpadDetectorConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const mmpadDetectorConfigArgs[] =  {&mmpadDetectorConfigArg0,
                                                            &mmpadDetectorConfigArg1,
                                                            &mmpadDetectorConfigArg2,
                                                            &mmpadDetectorConfigArg3,
                                                            &mmpadDetectorConfigArg4,
                                                            &mmpadDetectorConfigArg5,
                                                            &mmpadDetectorConfigArg6,
                                                            &mmpadDetectorConfigArg7};
static const iocshFuncDef configmmpadDetector = {"mmpadDetectorConfig", 8, mmpadDetectorConfigArgs};
static void configmmpadDetectorCallFunc(const iocshArgBuf *args)
{
    mmpadDetectorConfig(args[0].sval, args[1].sval, args[2].ival,  args[3].ival,
                        args[4].ival, args[5].ival, args[6].ival,  args[7].ival);
}


static void mmpadDetectorRegister(void)
{
    iocshRegister(&configmmpadDetector, configmmpadDetectorCallFunc);
}

extern "C" {
    epicsExportRegistrar(mmpadDetectorRegister);
}
