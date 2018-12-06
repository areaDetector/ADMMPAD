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
#include <cbf_ad.h>
#include <tiffio.h>
#include <iostream>
#include <sstream>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

/** Messages to/from camserver */
#define MAX_MESSAGE_SIZE 1024
#define MAX_FILENAME_LEN 256
#define MAX_BAD_PIXELS 100
/** Time to poll when reading from camserver */
#define ASYN_POLL_TIME .01
#define CAMSERVER_DEFAULT_TIMEOUT 1.0
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

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
} mmpadTriggerMode;

static const char *driverName = "mmpadDetector";

#define mmpadDelayTimeString      "DELAY_TIME"
#define mmpadThresholdString      "THRESHOLD"
#define mmpadThresholdApplyString "THRESHOLD_APPLY"
#define mmpadThresholdAutoApplyString "THRESHOLD_AUTO_APPLY"
#define mmpadArmedString          "ARMED"
#define mmpadImageFileTmotString  "IMAGE_FILE_TMOT"
#define mmpadBadPixelFileString   "BAD_PIXEL_FILE"
#define mmpadNumBadPixelsString   "NUM_BAD_PIXELS"
#define mmpadFlatFieldFileString  "FLAT_FIELD_FILE"
#define mmpadMinFlatFieldString   "MIN_FLAT_FIELD"
#define mmpadFlatFieldValidString "FLAT_FIELD_VALID"
#define mmpadGapFillString        "GAP_FILL"
#define mmpadWavelengthString     "WAVELENGTH"
#define mmpadEnergyLowString      "ENERGY_LOW"
#define mmpadEnergyHighString     "ENERGY_HIGH"
#define mmpadDetDistString        "DET_DIST"
#define mmpadDetVOffsetString     "DET_VOFFSET"
#define mmpadBeamXString          "BEAM_X"
#define mmpadBeamYString          "BEAM_Y"
#define mmpadFluxString           "FLUX"
#define mmpadFilterTransmString   "FILTER_TRANSM"
#define mmpadStartAngleString     "START_ANGLE"
#define mmpadAngleIncrString      "ANGLE_INCR"
#define mmpadDet2thetaString      "DET_2THETA"
#define mmpadPolarizationString   "POLARIZATION"
#define mmpadAlphaString          "ALPHA"
#define mmpadKappaString          "KAPPA"
#define mmpadPhiString            "PHI"
#define mmpadChiString            "CHI"
#define mmpadOscillAxisString     "OSCILL_AXIS"
#define mmpadNumOscillString      "NUM_OSCILL"

#define mmpadChipScale0String	  "CHIP_SCALE0"
#define mmpadChipScale1String	  "CHIP_SCALE1"
#define mmpadChipScale2String	  "CHIP_SCALE2"
#define mmpadChipScale3String	  "CHIP_SCALE3"
#define mmpadChipScale4String	  "CHIP_SCALE4"
#define mmpadChipScale5String	  "CHIP_SCALE5"
#define mmpadVideoModeOnString		  		"VIDEO_MODE_ON"
#define mmpadVideoModeFTimeString		  	"VIDEO_MODE_FTIME"
#define mmpadAVGAcquireString			  	"AVGACQUIRE"
#define mmpadAVGFileString			  		"AVGFILE"
#define mmpadAVGCountString			  		"AVGCOUNT"
#define mmpadBGSubtractString		      	"BG_SUBTRACT"
#define mmpadBGFileString				  	"BGFILE"

#define mmpadFileNameAPSString				"FILENAMEAPS"
#define mmpadFileIncNumString				"FILEINCNUM"

#define mmpadBackSubFlagString				"BACKSUB_FLAG"
#define mmpadMilDispBitShiftString			"MILDISP_BITSHIFT"
#define mmpadMilDispOnString				"MILDISP_ON"
#define mmpadMilDispOffsetString			"MILDISP_OFFSET"

#define mmpadResetFrameString						"RESET_FRAME"

#define mmpadRoiSumString					"ROI_SUM"
#define mmpadRoiULString					"ROI_UL"
#define mmpadRoiURString					"ROI_UR"
#define mmpadRoiLLString					"ROI_LL"
#define mmpadRoiLRString					"ROI_LR"

#define mmpadCamCmdString							"CAM_CMD"

//adding motor variables
#define mmpadGetMotString							"GET_MOT"			
#define mmpadMMot1String							"MMOT1"
#define mmpadMMot2String							"MMOT2"
#define mmpadMMot3String							"MMOT3"

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
    int mmpadItime;
    int mmpadFtime;
    int mmpadArmed;
    int mmpadImageFileTmot;
    int mmpadBadPixelFile;
    int mmpadNumBadPixels;
    int mmpadFlatFieldFile;
    int mmpadMinFlatField;
    int mmpadChipScale0;
    int mmpadChipScale1;
    int mmpadChipScale2;
    int mmpadChipScale3;
    int mmpadChipScale4;
    int mmpadChipScale5;
    int mmpadVideoModeOn;
    int mmpadVideoModeFTime;
    int mmpadAVGAcquire;
    int mmpadBGSubtract;

    int	mmpadBackSubFlag;
    int	mmpadMilDispBitShift;
    int	mmpadMilDispOn;
    int	mmpadMilDispOffset;

    int mmpadAVGFile;
    int mmpadAVGCount;
    int mmpadResetFrame;
    int mmpadBGFile;
    int mmpadFileNameAPS;
    int mmpadFileIncNum;
    
    int mmpadRoiSum;
    int mmpadRoiUL;
    int mmpadRoiUR;
    int mmpadRoiLL;
    int mmpadRoiLR;

    int mmpadCamCmd;
    //adding get motor addresses
    int mmpadGetMot;
    int mmpadMMot1;
    int mmpadMMot2;
    int mmpadMMot3;

    int mmpadFlatFieldValid;
#define LAST_mmpad_PARAM mmpadFlatFieldValid

  private:
    /* These are the methods that are new to this class */
    void makeMultipleFileFormat(const char *baseFileName);
    asynStatus waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    // void correctBadPixels(NDArray *pImage);
    int stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase);
    asynStatus readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus readCbf(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
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
//    badPixel badPixelMap[MAX_BAD_PIXELS];
    double averageFlatField;
    int	fileIncNum;
    string camserveInfo;
    //stringstream *sstr;
};

#define NUM_mmpad_PARAMS (&LAST_mmpad_PARAM - &FIRST_mmpad_PARAM + 1)

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
  if ( (q=strrchr(p, '.')) )
    {
      strcpy(mfExtension, q);
      *q = '\0';
    }
  else
    {
      strcpy(mfExtension, ""); /* default is raw image */
    }
  multipleFileNumber=0;   /* start number */
  fmt=5;        /* format length */
  if ( !(p=strrchr(mfTempFormat, '/')) )
    {
      p=mfTempFormat;
    }
  if ( (q=strrchr(p, '_')) )
    {
      q++;
      if (isdigit(*q) && isdigit(*(q+1)) && isdigit(*(q+2)))
        {
          multipleFileNumber=atoi(q);
          fmt=0;
          p=q;
          while(isdigit(*q))
            {
              fmt++;
              q++;
            }
          *p='\0';
          if (((fmt<3)  || ((fmt==3) && (numImages>999))) ||
              ((fmt==4) && (numImages>9999)))
            {
              fmt=5;
            }
        }
      else if (*q)
        {
          strcat(p, "_"); /* force '_' ending */
        }
    }
  else
    {
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

  while (deltaTime <= timeout)
    {
      fd = open(fileName, O_RDONLY, 0);
      if ((fd >= 0) && (timeout != 0.))
        {
          fileExists = 1;
          /* The file exists.  Make sure it is a new file, not an old one.
           * We don't do this check if timeout==0, which is used for reading flat field files */
          status = fstat(fd, &statBuff);
          if (status)
            {
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
      status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
      if (status == epicsEventWaitOK)
        {
          setStringParam(ADStatusMessage, "Acquisition aborted");
          return(asynError);
        }
      epicsTimeGetCurrent(&tCheck);
      deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }
  if (fd < 0)
    {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s timeout waiting for file to be created %s\n",
                driverName, functionName, fileName);
      if (fileExists)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "  file exists but is more than 10 seconds old, possible clock synchronization problem\n");
          setStringParam(ADStatusMessage, "Image file is more than 10 seconds old");
        }
      else
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
  while (i >= 0 && j >= 0)
    {
      if (shouldIgnoreCase)
        {
          if (tolower(aString[i]) != tolower(aSubstring[j])) return 0;
        }
      else
        {
          if (aString[i] != aSubstring[j]) return 0;
        }
      i--;
      j--;
    }
  return j < 0;
}

/** This function reads TIFF or CBF image files.  It is not intended to be general, it
 * is intended to read the TIFF or CBF files that camserver creates.  It checks to make
 * sure that the creation time of the file is after a start time passed to it, to force
 * it to wait for a new file to be created.
 */
asynStatus mmpadDetector::readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
  const char *functionName = "readImageFile";
  char tifFileName[MAX_FILENAME_LEN];
//  if (stringEndsWith(fileName, ".tif", 1) || stringEndsWith(fileName, ".tiff", 1))
//    {
  
  if (stringEndsWith(fileName, ".tif", 1))
		  {
			  return readTiff(fileName, pStartTime, timeout, pImage);
		  }
  else
		  {
			  strcpy(tifFileName,fileName);
			  strncat(tifFileName,".tif",MAX_FILENAME_LEN);
			  return readTiff(tifFileName, pStartTime, timeout, pImage);
		  }
//    }
//  else if (stringEndsWith(fileName, ".cbf", 1))
//    {
//      return readCbf(fileName, pStartTime, timeout, pImage);
//    }
//  else
//    {
//      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
//                "%s::%s, unsupported image file name extension, expected .tif or .cbf, fileName=%s\n",
//                driverName, functionName, fileName);
//      setStringParam(ADStatusMessage, "Unsupported file extension, expected .tif or .cbf");
//      return(asynError);
//    }
}

/** This function reads CBF files using CBFlib.  It is not intended to be general, it is
 * intended to read the CBF files that camserver creates.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus mmpadDetector::readCbf(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
  epicsTimeStamp tStart, tCheck;
  double deltaTime;
  int status=-1;
  const char *functionName = "readCbf";
  cbf_handle cbf;
  FILE *file=NULL;
  unsigned int cbfCompression;
  int cbfBinaryId;
  size_t cbfElSize;
  int cbfElSigned;
  int cbfElUnsigned;
  size_t cbfElements;
  int cbfMinElement;
  int cbfMaxElement;
  const char *cbfByteOrder;
  size_t cbfDimFast;
  size_t cbfDimMid;
  size_t cbfDimSlow;
  size_t cbfPadding;
  size_t cbfElementsRead;

  deltaTime = 0.;
  epicsTimeGetCurrent(&tStart);

  status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
  if (status != asynSuccess)
    {
      return((asynStatus)status);
    }

  cbf_set_warning_messages_enabled(0);
  cbf_set_error_messages_enabled(0);

  while (deltaTime <= timeout)
    {
      /* At this point we know the file exists, but it may not be completely
       * written yet.  If we get errors then try again. */

      status = cbf_make_handle(&cbf);
      if (status != 0)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, failed to make CBF handle, error code %#x\n",
                    driverName, functionName, status);
          return(asynError);
        }

      status = cbf_set_cbf_logfile(cbf, NULL);
      if (status != 0)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, failed to disable CBF logging, error code %#x\n",
                    driverName, functionName, status);
          return(asynError);
        }

      file = fopen(fileName, "rb");
      if (file == NULL)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, failed to open CBF file \"%s\" for reading: %s\n",
                    driverName, functionName, fileName, strerror(errno));
          cbf_free_handle(cbf);
          return(asynError);
        }

      status = cbf_read_widefile(cbf, file, MSG_DIGESTNOW);
      if (status != 0) goto retry;

      status = cbf_find_tag(cbf, "_array_data.data");
      if (status != 0) goto retry;

      /* Do some basic checking that the image size is what we expect */

      status = cbf_get_integerarrayparameters_wdims_fs(cbf, &cbfCompression,
               &cbfBinaryId, &cbfElSize, &cbfElSigned, &cbfElUnsigned,
               &cbfElements, &cbfMinElement, &cbfMaxElement, &cbfByteOrder,
               &cbfDimFast, &cbfDimMid, &cbfDimSlow, &cbfPadding);
      if (status != 0) goto retry;

      if (cbfDimFast != (size_t)pImage->dims[0].size)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, image width incorrect =%zd, should be %d\n",
                    driverName, functionName, cbfDimFast, pImage->dims[0].size);
          cbf_free_handle(cbf);
          return(asynError);
        }
      if (cbfDimMid != (size_t)pImage->dims[1].size)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, image height incorrect =%zd, should be %d\n",
                    driverName, functionName, cbfDimMid, pImage->dims[1].size);
          cbf_free_handle(cbf);
          return(asynError);
        }

      /* Read the image */

      status = cbf_get_integerarray(cbf, &cbfBinaryId, pImage->pData,
                                    sizeof(epicsInt32), 1, cbfElements, &cbfElementsRead);
      if (status != 0) goto retry;
      if (cbfElements != cbfElementsRead) goto retry;

      /* Sucesss! */
      status = cbf_free_handle(cbf);
      if (status != 0)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, failed to free CBF handle, error code %#x\n",
                    driverName, functionName, status);
          return(asynError);
        }
      break;

retry:
      status = cbf_free_handle(cbf);
      if (status != 0)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, failed to free CBF handle, error code %#x\n",
                    driverName, functionName, status);
          return(asynError);
        }
      /* Sleep, but check for stop event, which can be used to abort a long
       * acquisition */
      status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
      if (status == epicsEventWaitOK)
        {
          return(asynError);
        }
      epicsTimeGetCurrent(&tCheck);
      deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

  return(asynSuccess);
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
  int size, totalSize;
  int numStrips, strip;
  char *buffer;
  TIFF *tiff=NULL;
  epicsUInt32 uval;

  deltaTime = 0.;
  epicsTimeGetCurrent(&tStart);

  /* Suppress error messages from the TIFF library */
  TIFFSetErrorHandler(NULL);
  TIFFSetWarningHandler(NULL);

  status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
  if (status != asynSuccess)
    {
      return((asynStatus)status);
    }

  while (deltaTime <= timeout)
    {
      /* At this point we know the file exists, but it may not be completely written yet.
       * If we get errors then try again */
      tiff = TIFFOpen(fileName, "rc");
      if (tiff == NULL)
        {
          status = asynError;
          goto retry;
        }

      /* Do some basic checking that the image size is what we expect */
      status = TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &uval);
      if (uval != (epicsUInt32)pImage->dims[0].size)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, image width incorrect =%u, should be %d\n",
                    driverName, functionName, uval, pImage->dims[0].size);
          goto retry;
        }
      status = TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &uval);
      if (uval != (epicsUInt32)pImage->dims[1].size)
        {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, image length incorrect =%u, should be %d\n",
                    driverName, functionName, uval, pImage->dims[1].size);
          goto retry;
        }
      numStrips= TIFFNumberOfStrips(tiff);
      buffer = (char *)pImage->pData;
      totalSize = 0;
      for (strip=0; (strip < numStrips) && (totalSize < pImage->dataSize); strip++)
        {
          size = TIFFReadEncodedStrip(tiff, 0, buffer, pImage->dataSize-totalSize);
          if (size == -1)
            {
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
      if (totalSize != pImage->dataSize)
        {
          status = asynError;
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s, file size incorrect =%d, should be %d\n",
                    driverName, functionName, totalSize, pImage->dataSize);
          goto retry;
        }
      /* Sucesss! */
      break;

retry:
      if (tiff != NULL) TIFFClose(tiff);
      tiff = NULL;
      /* Sleep, but check for stop event, which can be used to abort a long acquisition */
      status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
      if (status == epicsEventWaitOK)
        {
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
  int ival;
  double dval;
  double dval2;
  int triggerMode;
  asynStatus status;

  //status = getIntegerParam(ADTriggerMode, &triggerMode);
  //if (status != asynSuccess) triggerMode = 2;

  triggerMode =2;
  /* When we change modes download all exposure parameters, since some modes
  * replace values with new parameters */
//    if (triggerMode == TMAlignment) {
//        setIntegerParam(ADNumImages, 1);
//    }
  /* nexpf > 1 is only supported in External Enable mode */
  //if (triggerMode!= TMbnc)
  //  {
  //    setIntegerParam(ADNumExposures, 1);
  //  }

status = getIntegerParam(ADNumImages, &ival);
  if ((status != asynSuccess) || (ival < 1))
    {
      ival = 1;
      setIntegerParam(ADNumImages, ival);
    }
//  status = getIntegerParam(ADNumExposures, &ival);
//  if ((status != asynSuccess) || (ival < 1))
//    {
//      ival = 1;
//      setIntegerParam(ADNumExposures, ival);
//    }
//  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "nexpframe %d", ival);
//  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

  status = getDoubleParam(ADAcquireTime, &dval);
  if ((status != asynSuccess) || (dval < 0.))
    {
      dval = 1.;
      setDoubleParam(ADAcquireTime, dval);
    }
  
  status = getDoubleParam(ADAcquirePeriod, &dval2);
    if ((status != asynSuccess) || (dval < 0.))
      {
        dval = 1.;
        setDoubleParam(ADAcquirePeriod, dval2);
      }
  
  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "set_take_n %f %f %d", dval, dval2, ival);
  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  
  setIntegerParam(ADStatus, ADStatusIdle);
  
  //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "FrameTriggerSource %d", triggerMode);
  //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

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
  while (deltaTime <= timeout)
    {
      status = pasynOctetSyncIO->read(pasynUser, this->fromCamserver,
                                      sizeof(this->fromCamserver), ASYN_POLL_TIME,
                                      &nread, &eomReason);
      if (status != asynTimeout) break;
      /* Sleep, but check for stop event, which can be used to abort a long acquisition */
      eventStatus = epicsEventWaitWithTimeout(this->stopEventId, ASYN_POLL_TIME);
      if (eventStatus == epicsEventWaitOK)
        {
          setStringParam(ADStatusMessage, "Acquisition aborted");
          return(asynError);
        }
      epicsTimeGetCurrent(&tCheck);
      deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

  // If we got asynTimeout, and timeout=0 then this is not an error, it is a poll checking for possible reply and we are done
  if ((status == asynTimeout) && (timeout == 0)) return(asynSuccess);
  if (status != asynSuccess)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s, timeout=%f, status=%d received %d bytes\n%s\n",
              driverName, functionName, timeout, status, nread, this->fromCamserver);
  else
    {
      /* Look for the string OK in the response */
      if (!strstr(this->fromCamserver, "OK"))
        {
          asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s unexpected response from camserver, no OK, response=%s\n",
                    driverName, functionName, this->fromCamserver);
          setStringParam(ADStatusMessage, "Error from camserver");
          status = asynError;
        }
      else
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
  int imageCounter;
  int numImages;
  int multipleFileNextImage=0;  /* This is the next image number, starting at 0 */
  int acquire;
  ADStatus_t acquiring;
  double startAngle;
  NDArray *pImage;
  double acquireTime, acquirePeriod;
  double readImageFileTimeout, timeout;
  int triggerMode;
  epicsTimeStamp startTime;
  const char *functionName = "mmpadTask";
  char fullFileName[MAX_FILENAME_LEN];
  char filePath[MAX_FILENAME_LEN];
  char fileName[MAX_FILENAME_LEN];
  char fileIncName[MAX_FILENAME_LEN];
  char statusMessage[MAX_MESSAGE_SIZE];
  int dims[2];
  int arrayCallbacks;
  int flatFieldValid;
  int len;
  int roisum,roiul,roiur,roill,roilr;
  string a;
  stringstream sstr;
  int testDone;
  int retryFlag;
  testDone=0;

  this->lock();

  /* Loop forever */
  while (1)
    {
      /* Is acquisition active? */
      stat2 = getIntegerParam(ADAcquire, &acquire);

      /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
      if (!acquire)
        {
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

      /* Reset the MX settings start angle */

//      epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Start_angle %f", startAngle);
//      writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

      /* Create the full filename */
      //createFileName(sizeof(fullFileName), fullFileName);

      status = asynSuccess;
      status |= getStringParam(NDFilePath, sizeof(filePath), filePath);
      status |= getStringParam(NDFileName, sizeof(fileName), fileName);

      sprintf(fileIncName,"%06d",fileIncNum);
      strcpy(fullFileName,strncat(filePath,fileName,MAX_FILENAME_LEN));
      strcat(fullFileName,fileIncName);
      //status = (asynStatus)setStringParam(function, (char *)value);
      status |= setStringParam(mmpadFileNameAPS,fullFileName);
      status |= setIntegerParam(mmpadFileIncNum,fileIncNum);
      //      strcpy(fullFileName,strncat(filePath,fileName,MAX_FILENAME_LEN));
      //modify for APS:

      status |= callParamCallbacks();

      fileIncNum++;
      status |= setIntegerParam(mmpadFileIncNum,fileIncNum);
      status |= callParamCallbacks();
      //status|=(asynStatus) callParamCallbacks();
      stat2 = setStringParam(NDFullFileName, fullFileName);
      stat2 = callParamCallbacks();

//      
//      switch (triggerMode)
//        {
//          case TMInternal:
//            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),
//                          "Exposure %s", fullFileName);
//            break;
//          case TMExternalEnable:
//            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),
//                          "ExtEnable %s", fullFileName);
//            break;
//          case TMExternalTrigger:
//            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),
//                          "ExtTrigger %s", fullFileName);
//            break;
//          case TMMultipleExternalTrigger:
//            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),
//                          "ExtMTrigger %s", fullFileName);
//            break;
////            case TMAlignment:
////                getStringParam(NDFilePath, sizeof(filePath), filePath);
////                epicsSnprintf(fullFileName, sizeof(fullFileName), "%salignment.tif",
////                              filePath);
////                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),
////                    "Exposure %s", fullFileName);
////                break;
//        }
      
      epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"Exposure %s", fullFileName);
      //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"Exposure test");
      stat2 = setStringParam(ADStatusMessage, "Starting exposure");
      /* Send the acquire command to camserver and wait for the 15OK response */
      writeReadCamserver(2.0);

//      this->camserveInfo = string(this->fromCamserver);
//      sstr.clear();
//      sstr.str("");
      //stringstream sstr(this->camserveInfo);
//      sstr<<this->fromCamserver;

//      while(!sstr.eof())
//      	{
//      		sstr>>a;
//      		if(a.compare("sum")==0)
//      		{
//      			sstr>>roisum;
//      		}
//      		else if(a.compare("ul")==0)
//      		{
//      			sstr>>roiul;
//      		}
//      		else if(a.compare("ur")==0)
//      		{
//      			sstr>>roiur;
//      		}
//      		else if(a.compare("lr")==0)
//      		{
//      		    sstr>>roilr;
//      		}
//      		else if(a.compare("ll")==0)
//      		{
//      		    sstr>>roill;
//      		}
//      	}
//
//      status |= setIntegerParam(mmpadRoiSum,roisum);
//      status |= setIntegerParam(mmpadRoiUL,roiul);
//      status |= setIntegerParam(mmpadRoiUR,roiur);
//      status |= setIntegerParam(mmpadRoiLL,roill);
//      status |= setIntegerParam(mmpadRoiLR,roilr);

      //    int mmpadRoiSum;
      //     int mmpadRoiUL;
      //     int mmpadRoiUR;
      //     int mmpadRoiLL;
      //     int mmpadRoiLR;


      /* OLD:Do another read in case there is an ERR string at the end of the input buffer */
      
      //NEW - looking for 5 enum response from camserver:
      
//      while(!testDone)
//      {
//			  status|=readCamserver(CAMSERVER_DEFAULT_TIMEOUT);
//			  this->camserveInfo = string(this->fromCamserver);
//			  sstr.clear();
//			  sstr.str("");
//			  while(!sstr.eof())
//			  {
//				  sstr>>a;
//				  if(a.compare("5")==0)
//				  {
//				        testDone = 1;
//						sstr>>a;
////				        if(a.compare("OK"))
////				        {
////				        	
////				        }
////				        else if((a.compare("ERR"))
////				        {
////				        	
////				        }
//				  }
//			  }
			  
      
    //  }
      
      /* If the status wasn't asynSuccess or asynTimeout, report the error */
      if (status>1)
        {
          acquire = 0;
        }
      else
        {
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

      while (acquire)
        {
          if (1)
            {
              /* For single frame or alignment mode need to wait for 7OK response from camserver
               * saying acquisition is complete before trying to read file, else we get a
               * recent but stale file. */
              //setStringParam(ADStatusMessage, "Waiting for 7OK response");
              callParamCallbacks();
              /* We release the mutex when waiting for 7OK because this takes a long time and
               * we need to allow abort operations to get through */
              this->unlock();
              //status = readCamserver(numImages*(acquireTime+0.001) + readImageFileTimeout+5);
              
              retryFlag=1;
              
              //if (status != asynTimeout)
              
              while(retryFlag)
              {
				  status = readCamserver(2*numImages*(acquireTime+0.001)+10);
				  if (status != asynTimeout)
				  {
				  this->camserveInfo = string(this->fromCamserver);
					sstr.clear();
					sstr.str("");

					sstr<<this->fromCamserver;

					while(!sstr.eof())
					{
						sstr>>a;
						if(a.compare("ERR")==0)
						{
							retryFlag=0;
							while(!sstr.eof())
							{
								sstr>>a;
								if(a.compare("retry")==0)
								{
									asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "...Got retry message...\n");
									retryFlag=1;
								}
							}
							
						}
						else if(a.compare("OK")==0)
						{
							retryFlag=0;
						}
      		
					}
				  }
				  else
				  {	   
					  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "... Got TimeOut ...\n");
					  retryFlag=0;
					  
				  }
			  }
              
              
              
              
              
              
              
              this->lock();
              /* If there was an error jump to bottom of loop */
              if (status)
                {
                  acquire = 0;
                  if(status==asynTimeout)
                	  stat2 = setStringParam(ADStatusMessage, "Timeout waiting for camserver response");
                  continue;
                }
//              else if(!status)
//              {
//					  
//            	  
//              }
            }
          else
            {
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
          if (1)
            {

              acquire = 0;
            }
//          else if (numImages > 1)
//            {
//              multipleFileNextImage++;
//              multipleFileNumber++;
//              if (multipleFileNextImage == numImages) acquire = 0;
//            }

        }
      
    
      
      //GETTING AVERAGE DATA FROM PAD TO SEND TO EPICS VARIABLES HERE.

      
      
      epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"GetComputation", fullFileName);
      status = writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
      //status=readCamserver(0.0);

      this->camserveInfo = string(this->fromCamserver);
      sstr.clear();
      sstr.str("");

      sstr<<this->fromCamserver;

      while(!sstr.eof())
      	{
      		sstr>>a;
      		if(a.compare("SUM")==0)
      		{
      			sstr>>roisum;
      		}
      		else if(a.compare("UL")==0)
      		{
      			sstr>>roiul;
      		}
      		else if(a.compare("UR")==0)
      		{
      			sstr>>roiur;
      		}
      		else if(a.compare("LR")==0)
      		{
      		    sstr>>roilr;
      		}
      		else if(a.compare("LL")==0)
      		{
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
      if ((numImages > 1) && (status == asynSuccess))
        {
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
          this->unlock();
  //        readCamserver(timeout);
          this->lock();
        }
      setShutter(0);
      stat2 = setIntegerParam(ADAcquire, 0);
      stat2 = setIntegerParam(mmpadArmed, 0);

//      status |= setStringParam(mmpadFileNameAPS,fullFileName);
//      status |= setIntegerParam(mmpadFileIncNum,fileIncNum);

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
  int	statusdum;
  const char *functionName = "writeInt32";
  double videomodeTime;
  
  char fullFileName[MAX_FILENAME_LEN];
  char filePath[MAX_FILENAME_LEN];
  char fileName[MAX_FILENAME_LEN];
  int	imgcount;
  int	milbitshift, milon,miloffset;
  int  testDone;
  string a;
  stringstream sstr;
  
  double	motpos1,motpos2,motpos3;
  int 		junkmot;
  int		stat2;
    
  
  
  status = setIntegerParam(function, value);

  if (function == ADAcquire)
    {
      getIntegerParam(ADStatus, &adstatus);
      if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError))
        {
          /* Send an event to wake up the mmpad task.  */
          epicsEventSignal(this->startEventId);
        }
      if (!value && (adstatus == ADStatusAcquire))
        {
          /* This was a command to stop acquisition */
          epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "Stop");
          writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
          epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "K");
          writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
          epicsEventSignal(this->stopEventId);
        }
    }
  else if ((function == ADTriggerMode) ||
           (function == ADNumImages) ||
           (function == ADNumExposures) )//||
//               (function == mmpadGapFill))
    {
      setAcquireParams();

    }
  else if(function == mmpadVideoModeOn)
  {
	 
	  if(value == 0)
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeoff");
		  writeReadCamserver(0.2);
		   //writeReadCamserver(0);
		   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
	  else
	  {
		  getDoubleParam(mmpadVideoModeFTime, &videomodeTime);
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeon %f", videomodeTime);
		  writeReadCamserver(0.2); 
		   //writeReadCamserver(0);
		   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
	  	  
  }
  else if(function == mmpadAVGAcquire)
  {
	  testDone = 1;
	  if(value)
	  {
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
			      while(!testDone)
			      {
						  readCamserver(CAMSERVER_DEFAULT_TIMEOUT);
						  this->camserveInfo = string(this->fromCamserver);
						  sstr.clear();
						  sstr.str("");
						  while(!sstr.eof())
						  {
							  sstr>>a;
							  if(a.compare("5")==0)
							  {
							        testDone = 1;
									sstr>>a;
			///				        if(a.compare("OK"))
			////				        {
			////				        	
			////				        }
			////				        else if((a.compare("ERR"))
			////				        {
			////				        	
			////				        }
							  }
							  
						  }
						  
			      
			     }
			
			setIntegerParam(mmpadAVGAcquire, 0);
			
	  }
  }
  else if(function == mmpadResetFrame)
  {
	  if(value)
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand reset_frame");
		  writeReadCamserver(0.2);
		   //writeReadCamserver(0);
		   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
		  setIntegerParam(mmpadResetFrame, 0);
	  }
  }
  
  else if(function == mmpadBackSubFlag)
  {
	  if(!value)
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbacksub 0");
		  writeReadCamserver(0.2);
		  //writeReadCamserver(0);
		  //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
	  else
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbacksub 1");
		  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
		   //writeReadCamserver(0);
		   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
  }
  
  else if ((function == mmpadMilDispBitShift)
		   || (function == mmpadMilDispOn)
		   || (function == mmpadMilDispOffset)
		  )
  {
	  getIntegerParam(mmpadMilDispBitShift,&milbitshift);
	  getIntegerParam(mmpadMilDispOn,&milon);
	  getIntegerParam(mmpadMilDispOffset,&miloffset);
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand mildisp %d %d %d",milbitshift,milon,miloffset);
	  //writeReadCamserver(0);
	  writeReadCamserver(0.2);
	  //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  }
  
  else if(function==mmpadGetMot)
  {
	  
	  // position 1?
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "getposition 1");
	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  
	  //read cam server - put into stringstream
	  this->camserveInfo = string(this->fromCamserver);
	  sstr.clear();
	  sstr.str("");

	  sstr<<this->fromCamserver;
	  
		while(!sstr.eof())
      	{
      		
      		sstr>>a;
      		if(a.compare("MOTOR")==0)
      		{
			
				sstr>>junkmot;
      			sstr>>motpos1;
      		}
		}
	  
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "getposition 2");
	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  
	  //read cam server - put into stringstream
	  this->camserveInfo = string(this->fromCamserver);
	  sstr.clear();
	  sstr.str("");

	  sstr<<this->fromCamserver;
		while(!sstr.eof())
      	{
		
      		sstr>>a;
      		if(a.compare("MOTOR")==0)
      		{
				sstr>>junkmot;
      			sstr>>motpos2;
      		}
		}
	  
	  
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "getposition 3");
	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  
	  //read cam server - put into stringstream
	  this->camserveInfo = string(this->fromCamserver);
	  sstr.clear();
	  sstr.str("");

	  sstr<<this->fromCamserver;
		while(!sstr.eof())
      	{
			
      		sstr>>a;
      		if(a.compare("MOTOR")==0)
      		{
				sstr>>junkmot;
      			sstr>>motpos3;
      		}
		}
	  
	  sstr.clear();
	  sstr.str("");
	  //sstr<<motpos1<<motpos2<<motpos3;
	  	
	  //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), sstr.str().c_str());
	  //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  //stat2=readCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  
	  //motpos1 = 1.0230;
	  
	  if(setDoubleParam(mmpadMMot1,motpos1))
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "EEEerror Mot 1");
	      writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
	  stat2 = callParamCallbacks();
	  
	  setDoubleParam(mmpadMMot2,motpos2);
	  stat2 = callParamCallbacks();
	  
	  setDoubleParam(mmpadMMot3,motpos3);
      stat2 = callParamCallbacks();
	  
	  
	  //resetting GetMotFlag
	  setIntegerParam(mmpadGetMot, 0);
  }
  else
    {
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
  int videomode;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setDoubleParam(function, value);


  /* Changing any of the following parameters requires recomputing the base image */
  if ((function == ADAcquireTime) ||
      (function == ADAcquirePeriod) ||
      (function == mmpadDelayTime))
    {
      setAcquireParams();
    }
  else if(function == mmpadVideoModeFTime)
  {
	  getIntegerParam(mmpadVideoModeOn, &videomode);
	  if(videomode)
	  {
		  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "videomodeon %f", value);
		  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
	  }
	  
  }
  
  else
    {
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

//  if (function == NDFilePath)
//    {
//     // epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "imgpath %s", value);
//     // writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
//      this->checkPath();
//    }
//  else
//    {
//      /* If this parameter belongs to a base class call its method */
//      if (function < FIRST_mmpad_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
//    }
//  *************************get error using case statement in compile:
//
//  switch(function)
//  {
//  case NDFilePath:
//	  this->checkPath();
//	  break;
//  case AVGFile:
//	  //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "imgpath %s", value);
//	  //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
//	  break;
//  case BGFile:
//	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "BackgroundFile %s", value);
//	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
//	  break;
//  default:
//	  if (function < FIRST_mmpad_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
//  }
  
  if(function ==NDFilePath)
  {
	  this->checkPath();
  }
//  else if(function == mmpadFileName)
//  {
//	  this->fileIncNum = 0;
//  }
//  else if(function == AVGFile)
//  {
//	   
//  }
  else if(function == mmpadBGFile)
  {
      getStringParam(NDFilePath, sizeof(filePath), filePath);
		strcpy(fullFileName,strncat(filePath,value,MAX_FILENAME_LEN));
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand milbackimg %s 0", fullFileName);
	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  }
  else if(function == mmpadCamCmd)
  {
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),value);
	  writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  }
  else
  {
	  if(function == NDFileName)
	  {
		  fileIncNum=0;
		  status = (asynStatus)setIntegerParam(mmpadFileIncNum,fileIncNum);
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
  if (details > 0)
    {
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

  : ADDriver(portName, 1, NUM_mmpad_PARAMS, maxBuffers, maxMemory,
             0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
             ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
             priority, stackSize),
  imagesRemaining(0)

{
  int status = asynSuccess;
  const char *functionName = "mmpadDetector";
  int dims[2];
  int	milbitshift, milon,miloffset;


  fileIncNum = 0;
  /* Create the epicsEvents for signaling to the mmpad task when acquisition starts and stops */
  this->startEventId = epicsEventCreate(epicsEventEmpty);
  if (!this->startEventId)
    {
      printf("%s:%s epicsEventCreate failure for start event\n",
             driverName, functionName);
      return;
    }
  this->stopEventId = epicsEventCreate(epicsEventEmpty);
  if (!this->stopEventId)
    {
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
    createParam(mmpadBadPixelFileString,   asynParamOctet,   &mmpadBadPixelFile);
    createParam(mmpadNumBadPixelsString,   asynParamInt32,   &mmpadNumBadPixels);
    createParam(mmpadFlatFieldFileString,  asynParamOctet,   &mmpadFlatFieldFile);
    createParam(mmpadMinFlatFieldString,   asynParamInt32,   &mmpadMinFlatField);
    createParam(mmpadFlatFieldValidString, asynParamInt32,   &mmpadFlatFieldValid);
    createParam(mmpadChipScale0String,  	asynParamInt32,   &mmpadChipScale0);
    createParam(mmpadChipScale1String,  	asynParamInt32,   &mmpadChipScale1);
    createParam(mmpadChipScale2String,  	asynParamInt32,   &mmpadChipScale2);
    createParam(mmpadChipScale3String,  	asynParamInt32,   &mmpadChipScale3);
    createParam(mmpadChipScale4String,  	asynParamInt32,   &mmpadChipScale4);
    createParam(mmpadChipScale5String,  	asynParamInt32,   &mmpadChipScale5);
    
    createParam(mmpadVideoModeOnString,  		asynParamInt32,   &mmpadVideoModeOn);
    createParam(mmpadVideoModeFTimeString,  	asynParamFloat64, &mmpadVideoModeFTime);
    createParam(mmpadAVGAcquireString,  		asynParamInt32,   &mmpadAVGAcquire);
    createParam(mmpadBGSubtractString,  		asynParamInt32,   &mmpadBGSubtract);
    createParam(mmpadAVGCountString,  			asynParamInt32,   &mmpadAVGCount);
    
    createParam(mmpadAVGFileString, 	asynParamOctet ,   &mmpadAVGFile);
    createParam(mmpadBGFileString, 		asynParamOctet ,   &mmpadBGFile);
   // createParam(mmpadThresholdString,      asynParamFloat64, &mmpadThreshold);

    createParam(mmpadFileIncNumString,				asynParamInt32,		&mmpadFileIncNum);
    createParam(mmpadFileNameAPSString,				asynParamOctet,		&mmpadFileNameAPS);

    createParam(mmpadBackSubFlagString,  			asynParamInt32,   &mmpadBackSubFlag);
    createParam(mmpadMilDispBitShiftString,  		asynParamInt32,   &mmpadMilDispBitShift);
    createParam(mmpadMilDispOnString,  				asynParamInt32,   &mmpadMilDispOn);
    createParam(mmpadMilDispOffsetString,  			asynParamInt32,   &mmpadMilDispOffset);
    createParam(mmpadResetFrameString,  			asynParamInt32,   &mmpadResetFrame);
    createParam(mmpadRoiSumString,  			asynParamInt32,   &mmpadRoiSum);
    createParam(mmpadRoiULString,  				asynParamInt32,   &mmpadRoiUL);
    createParam(mmpadRoiURString,  				asynParamInt32,   &mmpadRoiUR);
    createParam(mmpadRoiLLString,  				asynParamInt32,   &mmpadRoiLL);
    createParam(mmpadRoiLRString,  				asynParamInt32,   &mmpadRoiLR);
//attempting to add motor position command.    
    createParam(mmpadMMot1String,  				asynParamFloat64,   &mmpadMMot1);
    createParam(mmpadMMot2String,  				asynParamFloat64,   &mmpadMMot2);
    createParam(mmpadMMot3String,  				asynParamFloat64,   &mmpadMMot3);
    createParam(mmpadGetMotString,  		asynParamInt32,   &mmpadGetMot);

    createParam(mmpadCamCmdString,  				asynParamOctet,   &mmpadCamCmd);
//    int mmpadRoiSum;
//     int mmpadRoiUL;
//     int mmpadRoiUR;
//     int mmpadRoiLL;
//     int mmpadRoiLR;


    
  /* Set some default values for parameters */
 // status |=  setStringParam (NDFilePath, "/home/hugh/tmp/");
 // status |=  setStringParam (NDFileName, "junk");
 // status |=  setStringParam (NDFileTemplate, "00");
  status |= setIntegerParam(NDFileNumber, 1);
  status |= (asynStatus)callParamCallbacks();
  status |= setIntegerParam(mmpadAVGCount, 40);
  status |= (asynStatus)callParamCallbacks();
  //status |= setIntegerParam(NDAutoIncrement, 1);
  
  
  
  status |=  setStringParam (ADManufacturer, "Cornell");
  status |= setStringParam (ADModel, "mmpad");
  status |= setIntegerParam(ADMaxSizeX, maxSizeX);
  status |= setIntegerParam(ADMaxSizeY, maxSizeY);
  status |= setIntegerParam(ADSizeX, maxSizeX);
  status |= setIntegerParam(ADSizeX, maxSizeX);
  status |= setIntegerParam(ADSizeY, maxSizeY);
  status |= setIntegerParam(NDArraySizeX, maxSizeX);
  status |= setIntegerParam(NDArraySizeY, maxSizeY);
  status |= setIntegerParam(NDArraySize, 0);
  status |= setIntegerParam(NDDataType,  NDUInt32);
//    status |= setIntegerParam(ADImageMode, ADImageContinuous);
   status |= setIntegerParam(ADTriggerMode, 2);

//    status |= setIntegerParam(mmpadArmed, 0);
//    status |= setStringParam (mmpadBadPixelFile, "");
//    status |= setIntegerParam(mmpadNumBadPixels, 0);
//    status |= setStringParam (mmpadFlatFieldFile, "");
//    status |= setIntegerParam(mmpadFlatFieldValid, 0);
//
  if (status) 
    {
      printf("%s: unable to set camera parameters\n", functionName);
      return;
    }
    
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"ldcmndfile startup.cmd");
    setStringParam(ADStatusMessage, "Initializing Camera");
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT+8);
    
  //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"GrabInit");
       //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"Exposure test");
  //setStringParam(ADStatusMessage, "Initializing Camera");
  //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT+4);     
   //writeReadCamserver(0);
  //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  //sleep(1);
  
  //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"grab_display_on");
  //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT+4);
   //writeReadCamserver(0);  
   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
   //sleep(1);
   
   //epicsSnprintf(this->toCamserver, sizeof(this->toCamserver),"mmpadcommand compute 184 74 10 10");
   //writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
   //writeReadCamserver(0);  
   //writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
   //sleep(1);
   
  // ************ getting mildisp to go *********************
    status |= setIntegerParam(mmpadMilDispBitShift, 0);
    status |= setIntegerParam(mmpadMilDispOn, 1);
    status |= setIntegerParam(mmpadMilDispOffset, 40);
    
    status |= (asynStatus)callParamCallbacks();
   
         
	  getIntegerParam(mmpadMilDispBitShift,&milbitshift);
	  getIntegerParam(mmpadMilDispOn,&milon);
	  getIntegerParam(mmpadMilDispOffset,&miloffset);
	  epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mmpadcommand mildisp %d %d %d",milbitshift,milon,miloffset);
	  //writeReadCamserver(0);
       writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
  // **************** end getting mildisp to go ******* slight kluge     
       
       //sleep(1);
  /* Create the thread that updates the images */
  status = (epicsThreadCreate("mmpadDetTask",
                              epicsThreadPriorityMedium,
                              epicsThreadGetStackSize(epicsThreadStackMedium),
                              (EPICSTHREADFUNC)mmpadTaskC,
                              this) == NULL);
  if (status)
    {
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
    &mmpadDetectorConfigArg7
                                                           };
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

