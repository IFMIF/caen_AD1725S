/* ADCSimDetector.cpp
 *
 * This is a driver for a simulated ADC detector.
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  February 28, 2016
 *
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "ADCAEN1725S.h"
#include "asynNDArrayDriver.h"
#include <epicsExport.h>

static const char *driverName = "ADCAEN1725S";

// Some systems don't define M_PI in math.h
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void dataGrabTaskC(void *drvPvt) {
  ADCAEN1725S *pPvt = (ADCAEN1725S *)drvPvt;
  pPvt->dataGrabTask();
}

static void statusTaskC(void *drvPvt) {
  ADCAEN1725S *pPvt = (ADCAEN1725S *)drvPvt;
  pPvt->statusTask();
}

/** Constructor for ADCSimDetector; most parameters are simply passed to
 * ADDriver::ADDriver. After calling the base class constructor this method
 * creates a thread to compute the simulated detector data, and sets reasonable
 * default values for parameters defined in this class, asynNDArrayDriver and
 * ADDriver.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] numTimePoints The initial number of time points.
 * \param[in] dataType The initial data type (NDDataType_t) of the arrays that
 * this driver will create.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the
 * NDArrayPool for this driver is allowed to allocate. Set this to -1 to allow
 * an unlimited number of buffers. \param[in] maxMemory Then maximum amount of
 * memory that the NDArrayPool for this driver is allowed to allocate. Set this
 * to -1 to allow an unlimited amount of memory. \param[in] priority The thread
 * priority for the asyn port driver thread if ASYN_CANBLOCK is set in
 * asynFlags. \param[in] stackSize The stack size for the asyn port driver
 * thread if ASYN_CANBLOCK is set in asynFlags.
 */
ADCAEN1725S::ADCAEN1725S(const char *portName, int LinkType, int LinkNum,
                         int ConetNode, int VMEBaseAddress, int traceMask,
                         NDDataType_t dataType, int maxBuffers,
                         int memoryChannel, size_t maxMemory, int priority,
                         int stackSize)

    : asynNDArrayDriver(portName, MAX_SIGNALS, maxBuffers, maxMemory, 0,
                        0, /* No interfaces beyond those set in ADDriver.cpp */
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asyn flags*/
                        1,                                /* autoConnect=1 */
                        priority, stackSize),
      linkType_(static_cast<CAEN_DGTZ_ConnectionType>(LinkType)),
      linkNum_(LinkNum), conetNode_(ConetNode),
      VMEBaseAddress_(VMEBaseAddress) {
  int status = asynSuccess;
  char versionString[20];
  const char *functionName = "ADCAEN1725S";

  /* Create the epicsEvents for signaling to the simulate task when acquisition
   * starts and stops */
  this->startEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->startEventId_) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName,
           functionName);
    return;
  }
  this->stopEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->stopEventId_) {
    printf("%s:%s epicsEventCreate failure for stop event\n", driverName,
           functionName);
    return;
  }

  createParam(CaenAcquireString, asynParamInt32, &CN_Acquire);

  // Board information
  createParam(CaenModelNameString, asynParamOctet, &CN_ModelName);
  createParam(CaenModelString, asynParamInt32, &CN_Model);
  createParam(CaenChannelsString, asynParamInt32, &CN_Channels);
  createParam(CaenFormFactorString, asynParamInt32, &CN_FormFactor);
  createParam(CaenFamilyCodeString, asynParamInt32, &CN_FamilyCode);
  createParam(CaenAMCVersionString, asynParamOctet, &CN_AMCVersion);
  createParam(CaenPCBRevisionString, asynParamInt32, &CN_PCB_Revision);
  createParam(CaenADCNBitsString, asynParamInt32, &CN_ADC_NBits);

  // IRQ
  createParam(IRQEnableString, asynParamInt32, &CN_IRQEnable);
  createParam(IRQLevelString, asynParamInt32, &CN_IRQLevel);
  createParam(IRQStatusIDString, asynParamInt32, &CN_IRQStatusID);
  createParam(IRQEventNumberString, asynParamInt32, &CN_IRQEventNumber);
  createParam(IRQModeString, asynParamInt32, &CN_IRQMode);

  createParam(ClearDataString, asynParamInt32, &CN_ClearData);
  createParam(DisableEventAlignedReadoutString, asynParamInt32,
              &CN_DisableEventAlignedReadout);

  // Status
  createParam(ReadTemperatureString, asynParamInt32, &CN_ReadTemperature);

  createParam(AcquisitionModeString, asynParamInt32, &CN_AcquisitionMode);
  createParam(SendSWTriggerString, asynParamInt32, &CN_SendSWTrigger);
  createParam(SWTriggerModeString, asynParamInt32, &CN_SWTriggerMode);
  createParam(ExtTriggerModeString, asynParamInt32, &CN_ExtTriggerMode);
  createParam(ChannelSelfTriggerString, asynParamInt32, &CN_ChannelSelfTrigger);
  createParam(GroupSelfTriggerString, asynParamInt32, &CN_GroupSelfTrigger);
  createParam(RunSynchronizationModeString, asynParamInt32,
              &CN_RunSynchronizationMode);
  createParam(IOLevelString, asynParamInt32, &CN_IOLevel);
  createParam(DPPEventAggregationThresString, asynParamInt32,
              &CN_DPPEventAggregationThres);
  createParam(DPPEventAggregationMaxString, asynParamInt32,
              &CN_DPPEventAggregationMax);
  createParam(DPPNumEventAggregateString, asynParamInt32,
              &CN_DPPNumEventAggregate);

  createParam(DPPNumAggregateBLTString, asynParamInt32, &CN_DPPNumAggregateBLT);
  createParam(DPPAcquisitionModeString, asynParamInt32, &CN_DPPAcquisitionMode);
  createParam(DPPAcquisitionParamString, asynParamInt32,
              &CN_DPPAcquisitionParam);

  createParam(ChannelGroupMaskString, asynParamInt32, &CN_ChannelGroupMask);
  createParam(ChannelTriggerThresholdString, asynParamInt32,
              &CN_ChannelTriggerThreshold);
  createParam(GroupTriggerThresholdString, asynParamInt32,
              &CN_GroupTriggerThreshold);
  createParam(DPPTriggerModeString, asynParamInt32, &CN_DPPTriggerMode);

  createParam(ChannelEnableMaskString, asynParamInt32, &CN_ChannelEnableMask);

  // Input
  createParam(RecordLengthString, asynParamInt32, &CN_RecordLength);
  createParam(PreTriggerString, asynParamInt32, &CN_PreTrigger);
  createParam(ChannelPulsePolarityString, asynParamInt32,
              &CN_ChannelPulsePolarity);
  createParam(BaselineSamplesString, asynParamInt32, &CN_BaselineSamples);
  createParam(BaselineFixedValueString, asynParamInt32, &CN_BaselineFixedValue);
  createParam(DCOffsetString, asynParamInt32, &CN_DCOffset);
  createParam(InputDynamicString, asynParamInt32, &CN_InputDynamic);

  // Discriminator
  createParam(PSDDiscriminatorString, asynParamInt32, &CN_PSDDiscriminator);
  createParam(PSDSelfThresholdString, asynParamInt32, &CN_PSDSelfThreshold);
  createParam(PSDThresholdString, asynParamInt32, &CN_PSDThreshold);
  createParam(PSDThresholdHoldString, asynParamInt32, &CN_PSDThresholdHold);
  createParam(PSDCFDDelayString, asynParamInt32, &CN_PSDCFDDelay);
  createParam(PSDCFDFractionString, asynParamInt32, &CN_PSDCFDFraction);

  // QDC
  createParam(PSDChargeSensString, asynParamInt32, &CN_PSDChargeSens);
  createParam(PSDShortGateString, asynParamInt32, &CN_PSDShortGate);
  createParam(PSDLongGateString, asynParamInt32, &CN_PSDLongGate);
  createParam(PSDGateOffsetString, asynParamInt32, &CN_PSDGateOffset);

  // Veto/Coincidence
  createParam(PSDTriggerValidationWindowString, asynParamInt32,
              &CN_PSDTriggerValidationWindow);
  createParam(PSDPileUpString, asynParamInt32, &CN_PSDPileUp);
  createParam(PSDPileUpGapString, asynParamInt32, &CN_PSDPileUpGap);

  // Results
  createParam(PSDChargeLongString, asynParamInt32, &CN_PSDChargeLong);
  createParam(PSDChargeShortString, asynParamInt32, &CN_PSDChargeShort);
  createParam(PSDRateEvString, asynParamFloat64, &CN_PSDRateEv);
  createParam(PSDRateEv2String, asynParamFloat64, &CN_PSDRateEv2);

  epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d",
                DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
  setStringParam(NDDriverVersion, versionString);

  status |= setIntegerParam(NDDataType, dataType);

  status = connectADC();

  uint32_t temperature;
  for (auto i = 0; i < MAX_SIGNALS; i++) {
    auto ret = CAEN_DGTZ_ReadTemperature(handle_, i, &temperature);
    if (ret) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: CAEN_DGTZ_ReadTemperature failed (%d) \n", driverName,
                functionName, ret);
      status = asynError;
      // return (asynStatus)status;
    }
    setIntegerParam(i, CN_ReadTemperature, temperature);
  }

  status |= setIntegerParam(CN_AcquisitionMode, 0);

  status |= setIntegerParam(CN_IRQLevel, 1);
  status |= setIntegerParam(CN_IRQStatusID, 0);
  status |= setIntegerParam(CN_IRQEventNumber, 32);
  status |= setIntegerParam(CN_IRQMode, 0);
  status |= setIntegerParam(CN_IRQEnable, 1);

  status |= setIntegerParam(CN_SWTriggerMode, 0);
  status |= setIntegerParam(CN_ExtTriggerMode, 0);
  status |= setIntegerParam(CN_RunSynchronizationMode, 0);
  status |= setIntegerParam(CN_IOLevel, 0);

  status |= setIntegerParam(CN_DPPEventAggregationThres, 1);
  status |= setIntegerParam(CN_DPPEventAggregationMax, 0);
  status |= setIntegerParam(CN_DPPNumAggregateBLT, 1);
  status |= setIntegerParam(CN_DPPAcquisitionMode, 2);
  status |= setIntegerParam(CN_DPPAcquisitionParam, 2);
  status |= setIntegerParam(CN_DPPTriggerMode, 0);

  status |= setIntegerParam(CN_RecordLength, 256);

  for (auto i = 0; i < MAX_SIGNALS; i++) {

    status |= setIntegerParam(i, CN_PreTrigger, 24);
    status |= setIntegerParam(i, CN_ChannelPulsePolarity, 0);

    status |= setIntegerParam(i, CN_DPPNumEventAggregate, 1);

    status |= setIntegerParam(i, CN_PSDThresholdHold, 256);
    status |= setIntegerParam(i, CN_PSDThreshold, 650);
    status |= setIntegerParam(i, CN_PSDSelfThreshold, 1);
    status |= setIntegerParam(i, CN_PSDChargeSens, 1);
    status |= setIntegerParam(i, CN_PSDShortGate, 20);
    status |= setIntegerParam(i, CN_PSDLongGate, 75);
    status |= setIntegerParam(i, CN_PSDGateOffset, 0);
    status |= setIntegerParam(i, CN_PSDTriggerValidationWindow, 1);
    status |= setIntegerParam(i, CN_BaselineSamples, 1);
    status |= setIntegerParam(i, CN_BaselineFixedValue, 1);

    status |= setIntegerParam(i, CN_PSDDiscriminator, 0);
    status |= setIntegerParam(i, CN_PSDCFDFraction, 1);
    status |= setIntegerParam(i, CN_PSDCFDDelay, 1);
    status |= setIntegerParam(i, CN_PSDPileUp, 1);
    status |= setIntegerParam(i, CN_PSDPileUpGap, 1000);

    // Result
    status |= setIntegerParam(i, CN_PSDChargeLong, 0);
    status |= setIntegerParam(i, CN_PSDChargeShort, 0);
    status |= setDoubleParam(i, CN_PSDRateEv, 0.0);
    status |= setDoubleParam(i, CN_PSDRateEv2, 0.0);
  }

  if (status) {
    report(stdout, 1);
    return;
  }

  // createStaticEnums();
  status = (epicsThreadCreate("dataGrabTask", epicsThreadPriorityMedium,
                              epicsThreadGetStackSize(epicsThreadStackMedium),
                              (EPICSTHREADFUNC)dataGrabTaskC, this) == NULL);

  status |= (epicsThreadCreate("statusTask", epicsThreadPriorityLow,
                               epicsThreadGetStackSize(epicsThreadStackMedium),
                               (EPICSTHREADFUNC)statusTaskC, this) == NULL);

  startEventId_ = epicsEventCreate(epicsEventEmpty);

  if (status) {
    printf("%s:%s epicsThreadCreate failure for dataGrab task\n", driverName,
           functionName);
    return;
  }
}

asynStatus ADCAEN1725S::connectADC() {
  const char *functionName = "connectADC";
  int status = asynSuccess;
  CAEN_DGTZ_ErrorCode ret;
  // uint32_t temperature;

  ret = CAEN_DGTZ_OpenDigitizer(linkType_, linkNum_, conetNode_,
                                VMEBaseAddress_, &handle_);

  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_OpenDigitizer failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  ret = CAEN_DGTZ_GetInfo(handle_, &boardInfo_);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s:  CAEN_DGTZ_GetInfo failed (%d) \n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  setStringParam(ADSerialNumber, std::to_string(boardInfo_.SerialNumber));
  setStringParam(ADManufacturer, "CAEN");
  setStringParam(CN_ModelName, boardInfo_.ModelName);
  setIntegerParam(CN_Model, boardInfo_.Model);
  setIntegerParam(CN_Channels, boardInfo_.Channels);
  setIntegerParam(CN_FormFactor, boardInfo_.FormFactor);
  setIntegerParam(CN_FamilyCode, boardInfo_.FamilyCode);
  setStringParam(ADFirmwareVersion, boardInfo_.ROC_FirmwareRel);
  setStringParam(CN_AMCVersion, boardInfo_.AMC_FirmwareRel);
  setIntegerParam(CN_PCB_Revision, boardInfo_.PCB_Revision);
  setIntegerParam(CN_ADC_NBits, boardInfo_.ADC_NBits);

  ret = CAEN_DGTZ_Reset(handle_);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_Reset failed (%d) \n", driverName, functionName,
              ret);
    status = asynError;
    return (asynStatus)status;
  }

  ret = CAEN_DGTZ_SetRecordLength(handle_, TEMP_LENGTH_RECORD);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetRecordLength failed (%d) \n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  if (linkType_ != CAEN_DGTZ_USB_A4818) {
    ret = CAEN_DGTZ_SetInterruptConfig(handle_, CAEN_DGTZ_ENABLE, 1, 0xAAAA, 16,
                                       CAEN_DGTZ_IRQ_MODE_ROAK);
    if (ret) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: CAEN_DGTZ_SetInterruptConfig failed (%d) \n",
                driverName, functionName, ret);
      status = asynError;
      return (asynStatus)status;
    }
  }

  uint32_t mask;
  ret = CAEN_DGTZ_GetChannelEnableMask(handle_, &mask);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_GetChannelEnableMask failed (%d) \n",
              driverName, functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }
  status |= setIntegerParam(CN_ChannelEnableMask, mask); //, 0X0);

  for (auto i = 0; i < MAX_SIGNALS; i++) {
  }

  return (asynStatus)status;
}

asynStatus ADCAEN1725S::disconnectADC() {
  const char *functionName = "disconnectADC";
  int status = asynSuccess;
  CAEN_DGTZ_ErrorCode ret;

  ret = CAEN_DGTZ_CloseDigitizer(handle_);

  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s:  ADC disconnection failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  return (asynStatus)status;
}

void ADCAEN1725S::dataGrabTask() {
  int status = asynSuccess;
  // CAEN_DGTZ_ErrorCode ret;
  NDArray *pImage;
  epicsTimeStamp startTime;
  epicsTimeStamp prevTime;
  epicsTimeGetCurrent(&prevTime);
  epicsTimeStamp elapsedTime;
  int numTimePoints;
  int arrayCounter;
  double timeStep;
  int i;
  int acq_mode;

  /* Arrays for data analysis */
  uint64_t PrevTime[MAX_SIGNALS];
  uint64_t ExtendedTT[MAX_SIGNALS];
  uint32_t *EHisto[MAX_SIGNALS]; // Energy Histograms
  int ECnt[MAX_SIGNALS];
  int TrgCnt[MAX_SIGNALS];
  int PurCnt[MAX_SIGNALS];

  int event_counter[MAX_SIGNALS];

  const char *functionName = "dataGrabTask";

  int acquire;

  printf("%s:%s Running\n", driverName, functionName);

  lock();

  int Nb = 0;
  while (1) {
    getIntegerParam(CN_Acquire, &acquire);

    /* If we are not acquiring then wait for a semaphore that is given when
     * acquisition is started */
    if (!acquire) {
      unlock();
      printf("%s:%s Wait\n", driverName, functionName);
      epicsEventWait(startEventId_);
      lock();
      Nb = 0;
    }

    CAEN_DGTZ_ErrorCode ret;
    unlock();
    // epicsThreadSleep(1);
    ret = CAEN_DGTZ_ReadData(handle_, CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
                             buffer, &bufferSize);

    lock();
    if (ret) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s:  CAEN_DGTZ_ReadData failed (%d)\n", driverName,
                functionName, ret);
      status = asynError;
    }

    if (bufferSize == 0)
      continue;

    getIntegerParam(CN_AcquisitionMode, &acq_mode);

    Nb += bufferSize;
    unlock();
    ret = CAEN_DGTZ_GetDPPEvents(handle_, buffer, bufferSize, (void **)events,
                                 numEvents);
    lock();
    if (ret) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s:  CAEN_DGTZ_GetDPPEvents failed (%d)\n", driverName,
                functionName, ret);
      status = asynError;
    }

    epicsUInt16 *pData;
    auto numTimePoints = TEMP_LENGTH_RECORD;
    NDDataType_t dataType = NDDataType_t::NDUInt16;

    size_t dims[3];
    dims[0] = MAX_SIGNALS;
    dims[1] = numTimePoints;
    // dims[2] = 3;
    if (this->pArrays[0])
      this->pArrays[0]->release();
    this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, 0);
    pData = (epicsUInt16 *)this->pArrays[0]->pData;
    memset(pData, 0, MAX_SIGNALS * numTimePoints * sizeof(epicsUInt16));

    for (auto ch = 0; ch < MAX_SIGNALS; ch++) {
      event_counter[ch] = event_counter[ch] + numEvents[ch];
      for (auto ev = 0; ev < numEvents[ch]; ev++) {
        if ((events[ch][ev].ChargeLong > 0) &&
            (events[ch][ev].ChargeShort > 0)) {

          status |=
              setIntegerParam(ch, CN_PSDChargeLong, events[ch][ev].ChargeLong);
          status |= setIntegerParam(ch, CN_PSDChargeShort,
                                    events[ch][ev].ChargeShort);
          callParamCallbacks(ch);
        }

        if ((acq_mode != CAEN_DGTZ_DPP_ACQ_MODE_List) && (ev == 0)) {
          int size;
          uint16_t *WaveLine;
          uint8_t *DigitalWaveLine;
          CAEN_DGTZ_DecodeDPPWaveforms(handle_, (void *)&events[ch][ev],
                                       waveform);

          // Use waveform data here...
          size = (int)(waveform->Ns);  // Number of samples
          WaveLine = waveform->Trace1; // First trace (for DPP-PSD it is ALWAYS
                                       // the Input Signal)
          for (auto s = 0; s < size; s++) {
            pData[0 + MAX_SIGNALS * ch + s] = (epicsUInt16)(WaveLine[s]);
          }

          WaveLine = waveform->Trace2;
          DigitalWaveLine = waveform->DTrace1;
          DigitalWaveLine = waveform->DTrace2;
        }
      }
    }

    pImage = this->pArrays[0];

    pImage->uniqueId = uniqueId_++;
    getIntegerParam(NDArrayCounter, &arrayCounter);
    arrayCounter++;
    setIntegerParam(NDArrayCounter, arrayCounter);
    epicsTimeGetCurrent(&startTime);
    pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
    updateTimeStamp(&pImage->epicsTS);

    elapsedTime_ = epicsTimeDiffInSeconds(&startTime, &prevTime);

    if (elapsedTime_ > 0.5) {
      for (auto ch = 0; ch < MAX_SIGNALS; ch++) {
        status |= setDoubleParam(ch, CN_PSDRateEv,
                                 (float)event_counter[ch] / elapsedTime_);
        event_counter[ch] = 0;
        callParamCallbacks(ch);
      }
      prevTime = startTime;
    }

    /* Get any attributes that have been defined for this driver */
    this->getAttributes(pImage->pAttributeList);

    /* Call the NDArray callback */
    doCallbacksGenericPointer(pImage, NDArrayData, 0);

    // for (i = 0; i < MAX_SIGNALS; i++) {
    //   callParamCallbacks(i);
    // }

    getIntegerParam(CN_Acquire, &acquire);
    if ((acquire == 0)) {
      status = stopCapture();
    }
    callParamCallbacks();
  }
}

void ADCAEN1725S::statusTask() {
  // int status = asynSuccess;
  CAEN_DGTZ_ErrorCode ret;
  uint32_t temperature;

  const char *functionName = "statusTask";

  int acquire;

  while (1) {
    getIntegerParam(CN_Acquire, &acquire);
    lock();
    for (auto i = 0; i < MAX_SIGNALS; i++) {
      ret = CAEN_DGTZ_ReadTemperature(handle_, i, &temperature);
      if (ret) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: CAEN_DGTZ_ReadTemperature failed (%d) \n", driverName,
                  functionName, ret);
      }
      setIntegerParam(i, CN_ReadTemperature, temperature);
      callParamCallbacks(i);
    }
    unlock();
    epicsThreadSleep(1);
  }
}

asynStatus ADCAEN1725S::createStaticEnums() {
  // const char *functionName = "createStaticEnums";
  enumStruct_t *pEnum;
  int mode = 0;
  numValidSWTriggerModes_ = 0;
  for (mode = 0; mode < NUM_TRIGGER_MODES; mode++) {
    // Internal trigger mode is always supported
    pEnum = swtriggerModeEnums_ + mode;
    strcpy(pEnum->string, triggerModeStrings[mode]);
    pEnum->value = mode;
    numValidSWTriggerModes_++;
  }

  numValidExtTriggerModes_ = 0;
  for (mode = 0; mode < NUM_TRIGGER_MODES; mode++) {
    // Internal trigger mode is always supported
    pEnum = exttriggerModeEnums_ + mode;
    strcpy(pEnum->string, triggerModeStrings[mode]);
    pEnum->value = mode;
    numValidExtTriggerModes_++;
  }

  int status = asynSuccess;
  return (asynStatus)status;
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters, including ADAcquire,
 * ADColorMode, etc. For all parameters it sets the value in the parameter
 * library and calls any registered callbacks.. \param[in] pasynUser pasynUser
 * structure that encodes the reason and address. \param[in] value Value to
 * write. */
asynStatus ADCAEN1725S::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  asynStatus status = asynSuccess;
  int addr;

  const char *functionName = "writeInt32";

  int function = pasynUser->reason;
  // printf("%s:%s Function %i\n", driverName, functionName, function);

  pasynManager->getAddr(pasynUser, &addr);
  if (addr < 0)
    addr = 0;

  /* Set the value in the parameter library.  This may change later but that's
   * OK */
  status = setIntegerParam(addr, function, value);

  if (function == CN_Acquire) {
    if (value) {
      /* start acquisition */
      status = startCapture();
    } else {
      // setIntegerParam(CN_Acquire, 0);
      status = stopCapture();
    }
  } else if ((function == CN_ExtTriggerMode) ||
             (function == CN_ChannelSelfTrigger) ||
             (function == CN_GroupSelfTrigger) ||
             (function == CN_ChannelGroupMask) ||
             (function == CN_ChannelTriggerThreshold) ||
             (function == CN_GroupTriggerThreshold) ||
             (function == CN_RunSynchronizationMode) ||
             (function == CN_IOLevel) ||
             (function == CN_ChannelPulsePolarity)) {

  } else if ((function == CN_SWTriggerMode)) {
  } else if ((function == CN_IRQEnable) /** || (function == CN_IRQLevel) ||
             (function == CN_IRQStatusID) || (function == CN_IRQEventNumber) ||
             (function == CN_IRQMode)*/) {
    setIRQ();
  } else if ((function == CN_ChannelEnableMask)) {
    setChannelEnableMask();
  } else if ((function == CN_AcquisitionMode)) {
    setDigitizerParameter();
  }

  else {
    /* If this parameter belongs to a base class call its method */
    if (function < FIRST_CN_DETECTOR_PARAM)
      status = asynNDArrayDriver::writeInt32(pasynUser, value);
  }

  asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s::%s function=%d, addr=%d, value=%d, status=%d\n", driverName,
            functionName, function, addr, value, status);
  callParamCallbacks(addr);
  return status;
}

asynStatus ADCAEN1725S::startCapture() {
  asynStatus status = asynSuccess;
  const char *functionName = "startCapture";
  CAEN_DGTZ_ErrorCode ret;

  // setIntegerParam(ADNumImagesCounter, 0);

  ret = CAEN_DGTZ_SWStartAcquisition(handle_);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SWStartAcquisition failed (%d)\n", driverName,
              functionName, ret);
  }

  epicsEventSignal(startEventId_);

  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Done\n", driverName,
            functionName);

  return status;
}

asynStatus ADCAEN1725S::stopCapture() {
  asynStatus status = asynSuccess;
  const char *functionName = "stopCapture";
  CAEN_DGTZ_ErrorCode ret;
  setIntegerParam(CN_Acquire, 0);

  ret = CAEN_DGTZ_SWStopAcquisition(handle_);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SWStopAcquisition failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s Stop capture\n",
            driverName, functionName);

  return status;
}

asynStatus ADCAEN1725S::setIRQ() {
  asynStatus status = asynSuccess;
  const char *functionName = "setIRQ";
  if (linkType_ == CAEN_DGTZ_USB_A4818) {
    printf("%s:%s CAEN_DGTZ_SetInterruptConfig skipped\n", driverName,
           functionName);
    return status;
  }
  CAEN_DGTZ_ErrorCode ret;
  CAEN_DGTZ_EnaDis_t state;
  uint8_t level;
  uint32_t status_id;
  uint16_t event_number;
  CAEN_DGTZ_IRQMode_t irq_mode;

  int ret_int;
  getIntegerParam(CN_IRQEnable, &ret_int);
  state = static_cast<CAEN_DGTZ_EnaDis_t>(ret_int);
  getIntegerParam(CN_IRQLevel, &ret_int);
  level = static_cast<int>(ret_int);
  getIntegerParam(CN_IRQStatusID, &ret_int);
  status_id = static_cast<int>(ret_int);
  getIntegerParam(CN_IRQEventNumber, &ret_int);
  event_number = static_cast<uint16_t>(ret_int);
  getIntegerParam(CN_IRQMode, &ret_int);
  irq_mode = static_cast<CAEN_DGTZ_IRQMode_t>(ret_int);

  ret = CAEN_DGTZ_SetInterruptConfig(handle_, state, level, status_id,
                                     event_number, irq_mode);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetInterruptConfig failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }
  return status;
}

asynStatus ADCAEN1725S::setChannelEnableMask() {
  asynStatus status = asynSuccess;
  const char *functionName = "setChannelEnableMask";

  CAEN_DGTZ_ErrorCode ret;
  int ret_int;
  getIntegerParam(CN_ChannelEnableMask, &ret_int);
  auto mask = static_cast<uint32_t>(ret_int);

  ret = CAEN_DGTZ_SetChannelEnableMask(handle_, mask);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetChannelEnableMask failed (%d) -> %d\n",
              driverName, functionName, ret, mask);
    status = asynError;
    return (asynStatus)status;
  }

  printf("%s:%s CAEN_DGTZ_SetChannelEnableMask Done\n", driverName,
         functionName);
  return status;
}

asynStatus ADCAEN1725S::setDigitizerParameter() {
  asynStatus status = asynSuccess;
  const char *functionName = "setDigitizerParameter";

  int ret_int;
  CAEN_DGTZ_ErrorCode ret;
  CAEN_DGTZ_IOLevel_t io_level;
  CAEN_DGTZ_DPP_AcqMode_t acq_dpp_mode;
  CAEN_DGTZ_DPP_SaveParam_t save_parm;
  CAEN_DGTZ_AcqMode_t acq_mode;
  CAEN_DGTZ_TriggerMode_t trig_mode;
  CAEN_DGTZ_RunSyncMode_t sync_mode;
  int aggregate_th, aggregate_max;

  getIntegerParam(CN_DPPAcquisitionMode, &ret_int);
  acq_dpp_mode = static_cast<CAEN_DGTZ_DPP_AcqMode_t>(ret_int);
  getIntegerParam(CN_DPPAcquisitionParam, &ret_int);
  save_parm = static_cast<CAEN_DGTZ_DPP_SaveParam_t>(ret_int);
  ret = CAEN_DGTZ_SetDPPAcquisitionMode(handle_, acq_dpp_mode, save_parm);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetDPPAcquisitionMode failed (%d)\n",
              driverName, functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  getIntegerParam(CN_AcquisitionMode, &ret_int);
  acq_mode = static_cast<CAEN_DGTZ_AcqMode_t>(ret_int);
  ret = CAEN_DGTZ_SetAcquisitionMode(handle_, acq_mode);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetAcquisitionMode failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  getIntegerParam(CN_IOLevel, &ret_int);
  io_level = static_cast<CAEN_DGTZ_IOLevel_t>(ret_int);
  ret = CAEN_DGTZ_SetIOLevel(handle_, io_level);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetIOLevel failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  getIntegerParam(CN_ExtTriggerMode, &ret_int);
  trig_mode = static_cast<CAEN_DGTZ_TriggerMode_t>(ret_int);
  ret = CAEN_DGTZ_SetExtTriggerInputMode(handle_, trig_mode);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetExtTriggerInputMode failed (%d)\n",
              driverName, functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  // // Set how many events to accumulate in the board memory before being
  // // available for readout

  getIntegerParam(CN_DPPEventAggregationThres, &aggregate_th);
  getIntegerParam(CN_DPPEventAggregationMax, &aggregate_max);
  ret = CAEN_DGTZ_SetDPPEventAggregation(handle_, aggregate_th, aggregate_max);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetDPPEventAggregation failed (%d)\n",
              driverName, functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  getIntegerParam(CN_RunSynchronizationMode, &ret_int);
  sync_mode = static_cast<CAEN_DGTZ_RunSyncMode_t>(ret_int);
  ret = CAEN_DGTZ_SetRunSynchronizationMode(handle_, sync_mode);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_RunSyncMode_t failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  setChannelParameter();

  // TODO: Implement virtual probe.
  CAEN_DGTZ_SetDPP_VirtualProbe(handle_, ANALOG_TRACE_1,
                                CAEN_DGTZ_DPP_VIRTUALPROBE_Input);
  CAEN_DGTZ_SetDPP_VirtualProbe(handle_, ANALOG_TRACE_2,
                                CAEN_DGTZ_DPP_VIRTUALPROBE_Baseline);
  CAEN_DGTZ_SetDPP_VirtualProbe(handle_, DIGITAL_TRACE_1,
                                CAEN_DGTZ_DPP_DIGITALPROBE_Gate);
  CAEN_DGTZ_SetDPP_VirtualProbe(handle_, DIGITAL_TRACE_2,
                                CAEN_DGTZ_DPP_DIGITALPROBE_GateShort);

  printf("%s:%s Digitizer Parameter Done\n", driverName, functionName);
  return status;
}

asynStatus ADCAEN1725S::setChannelParameter() {
  asynStatus status = asynSuccess;
  const char *functionName = "setChannelParameter";
  CAEN_DGTZ_DPP_PSD_Params_t params;
  CAEN_DGTZ_ErrorCode ret;

  int recordlength[MAX_SIGNALS];
  int pretrigger[MAX_SIGNALS];
  int polarity[MAX_SIGNALS];
  int dcoffset[MAX_SIGNALS];

  int ret_int;
  int thrh;
  int thr[MAX_SIGNALS];
  int selfthr[MAX_SIGNALS];
  int csens[MAX_SIGNALS];
  int sgate[MAX_SIGNALS];
  int lgate[MAX_SIGNALS];
  int gateof[MAX_SIGNALS];
  int trigw[MAX_SIGNALS];
  int nsbs[MAX_SIGNALS];
  int disc[MAX_SIGNALS];
  int cfdf[MAX_SIGNALS];
  int cfdd[MAX_SIGNALS];
  CAEN_DGTZ_DPP_PUR_t purh;
  int purg;

  getIntegerParam(CN_PSDThresholdHold, &thrh);
  params.trgho = thrh;

  getIntegerParam(CN_PSDPileUp, &ret_int);
  purh = static_cast<CAEN_DGTZ_DPP_PUR_t>(ret_int);
  params.purh = purh;

  getIntegerParam(CN_PSDShortGate, &purg);
  params.purgap = purg;

  for (auto i = 0; i < MAX_SIGNALS; i++) {

    getIntegerParam(i, CN_PSDThreshold, thr + i);
    params.thr[i] = thr[i];

    getIntegerParam(i, CN_PSDSelfThreshold, selfthr + i);
    params.selft[i] = selfthr[i];

    getIntegerParam(i, CN_PSDChargeSens, csens + i);
    params.csens[i] = csens[i];

    getIntegerParam(i, CN_PSDShortGate, sgate + i);
    params.sgate[i] = sgate[i];

    getIntegerParam(i, CN_PSDLongGate, lgate + i);
    params.lgate[i] = lgate[i];

    getIntegerParam(i, CN_PSDGateOffset, gateof + i);
    params.pgate[i] = gateof[i];

    getIntegerParam(i, CN_PSDTriggerValidationWindow, trigw + i);
    params.tvaw[i] = trigw[i];

    getIntegerParam(i, CN_BaselineSamples, nsbs + i);
    params.nsbl[i] = nsbs[i];

    getIntegerParam(i, CN_PSDDiscriminator, disc + i);
    params.discr[i] = disc[i];

    getIntegerParam(i, CN_PSDCFDFraction, cfdf + i);
    params.cfdf[i] = cfdf[i];

    getIntegerParam(i, CN_PSDCFDDelay, cfdd + i);
    params.cfdd[i] = cfdd[i];

    params.trgc[i] =
        CAEN_DGTZ_DPP_TriggerConfig_t::CAEN_DGTZ_DPP_TriggerConfig_Threshold;
  }

  ret = CAEN_DGTZ_SetDPPParameters(handle_, 0xFF, &params);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_SetDPPParameters failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  getIntegerParam(CN_InputDynamic, &ret_int);
  ret = CAEN_DGTZ_WriteRegister(handle_, 0x8028, ret_int);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_WriteRegister failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  for (auto i = 0; i < MAX_SIGNALS; i++) {
    if (i % 2 == 0) {
      getIntegerParam(i, CN_RecordLength, recordlength + i);
      ret = CAEN_DGTZ_SetRecordLength(handle_, recordlength[i], i);
    }

    // Set a DC offset to the input signal to adapt it to digitizer's dynamic
    // range
    getIntegerParam(i, CN_DCOffset, dcoffset + i);
    ret = CAEN_DGTZ_SetChannelDCOffset(handle_, i, dcoffset[i]);

    // Set the Pre-Trigger size (in samples)
    getIntegerParam(i, CN_PreTrigger, pretrigger + i);
    ret = CAEN_DGTZ_SetDPPPreTriggerSize(handle_, i, pretrigger[i]);

    // Set the polarity for the given channel (CAEN_DGTZ_PulsePolarityPositive
    // or CAEN_DGTZ_PulsePolarityNegative)
    getIntegerParam(i, CN_ChannelPulsePolarity, polarity + i);
    ret = CAEN_DGTZ_SetChannelPulsePolarity(
        handle_, i, static_cast<CAEN_DGTZ_PulsePolarity_t>(polarity[i]));
  }

  printf("%s:%s DPP Parameter Done\n", driverName, functionName);

  allocBuffer();
  return status;
}

asynStatus ADCAEN1725S::allocBuffer() {
  asynStatus status = asynSuccess;
  const char *functionName = "allocBuffer";
  CAEN_DGTZ_ErrorCode ret;

  ret = CAEN_DGTZ_MallocReadoutBuffer(handle_, &buffer, &allocatedSize);
  /* Allocate memory for the events */
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_MallocReadoutBuffer failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  ret = CAEN_DGTZ_MallocDPPEvents(handle_, (void **)&events, &allocatedSize);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_MallocDPPEvents failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  /* Allocate memory for the waveforms */
  ret =
      CAEN_DGTZ_MallocDPPWaveforms(handle_, (void **)&waveform, &allocatedSize);
  if (ret) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: CAEN_DGTZ_MallocDPPWaveforms failed (%d)\n", driverName,
              functionName, ret);
    status = asynError;
    return (asynStatus)status;
  }

  printf("%s:%s allocBuffer Done\n", driverName, functionName);
  return status;
}

asynStatus ADCAEN1725S::readEnum(asynUser *pasynUser, char *strings[],
                                 int values[], int severities[],
                                 size_t nElements, size_t *nIn) {
  asynStatus status = asynSuccess;
  const char *functionName = "readEnum";

  int function = pasynUser->reason;
  enumStruct_t *pEnum;
  int numEnums;
  int i;

  printf("%s:%s Function %i\n", driverName, functionName, function);

  if (function == CN_SWTriggerMode) {
    pEnum = swtriggerModeEnums_;
    numEnums = numValidSWTriggerModes_;
  } else {
    *nIn = 0;
    return asynError;
  }

  for (i = 0; ((i < numEnums) && (i < (int)nElements)); i++) {
    if (strings[i])
      free(strings[i]);
    strings[i] = epicsStrDup(pEnum->string);
    values[i] = pEnum->value;
    severities[i] = 0;
    pEnum++;
  }
  *nIn = i;
  return status;
}

/** Report status of the driver.
 * Prints details about the driver if details>0.
 * It then calls the ADDriver::report() method.
 * \param[in] fp File pointed passed by caller where the output is written to.
 * \param[in] details If >0 then driver details are printed.
 */
void ADCAEN1725S::report(FILE *fp, int details) {

  fprintf(fp, "ADC simulation detector %s\n", this->portName);
  if (details > 0) {
    int numTimePoints, dataType;
    fprintf(fp, "  # time points:   %d\n", numTimePoints);
    fprintf(fp, "      Data type:   %d\n", dataType);
  }
  /* Invoke the base class method */
  asynNDArrayDriver::report(fp, details);
}

/** Configuration command, called directly or from iocsh */
extern "C" int ADCAEN1725SConfig(const char *portName, int LinkType,
                                 int LinkNum, int ConetNode, int VMEBaseAddress,
                                 int traceMask, int dataType, int maxBuffers,
                                 int memoryChannel, int maxMemory, int priority,
                                 int stackSize) {
  new ADCAEN1725S(portName, LinkType, LinkNum, ConetNode, VMEBaseAddress,
                  traceMask, (NDDataType_t)dataType,
                  (maxBuffers < 0) ? 0 : maxBuffers, memoryChannel,
                  (maxMemory < 0) ? 0 : maxMemory, priority, stackSize);
  return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADCAEN1725SConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADCAEN1725SConfigArg1 = {"LinkType", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg2 = {"LinkNum", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg3 = {"ConetNode", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg4 = {"VMEBaseAddress", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg5 = {"traceMask", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg6 = {"dataType", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg7 = {"maxBuffers", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg8 = {"memoryChannel", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg9 = {"maxMemory", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg10 = {"priority", iocshArgInt};
static const iocshArg ADCAEN1725SConfigArg11 = {"stackSize", iocshArgInt};
static const iocshArg *const ADCAEN1725SConfigArgs[] = {
    &ADCAEN1725SConfigArg0, &ADCAEN1725SConfigArg1,  &ADCAEN1725SConfigArg2,
    &ADCAEN1725SConfigArg3, &ADCAEN1725SConfigArg4,  &ADCAEN1725SConfigArg5,
    &ADCAEN1725SConfigArg6, &ADCAEN1725SConfigArg7,  &ADCAEN1725SConfigArg8,
    &ADCAEN1725SConfigArg9, &ADCAEN1725SConfigArg10, &ADCAEN1725SConfigArg11};
static const iocshFuncDef configADCAEN1725S = {"ADCAEN1725SConfig", 11,
                                               ADCAEN1725SConfigArgs};
static void configADCAEN1725SCallFunc(const iocshArgBuf *args) {
  ADCAEN1725SConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                    args[4].ival, args[5].ival, args[6].ival, args[7].ival,
                    args[8].ival, args[9].ival, args[10].ival, args[11].ival);
}

static void ADCAEN1725SRegister(void) {
  iocshRegister(&configADCAEN1725S, configADCAEN1725SCallFunc);
}

extern "C" {
epicsExportRegistrar(ADCAEN1725SRegister);
}
