# AD1725S

An [EPICS](http://www.aps.anl.gov/epics) [areaDetector](https://github.com/areaDetector/areaDetector/blob/master/README.md)
driver for [CAEN 1725S](https://www.caen.it/products/v1725/) digitizer board using the
[CAENDigitizer](https://www.caen.it/products/caendigitizer-library/) library.

The AD1725S is a VME digitizer board that can record signal up to 250 MS/s with 14 bis resolutions
The digitizer can use different firmware that provides various features:
- Pulse Height Analysis (DPP-PHA)
- Pulse Shape Discrimination (DPP-PSD)
- Zero Length Encoding (DPP-ZLEplus) 
- Dynamic Acquisition Window (DPP-DAW)

This repository is an attempt to create an EPICS support for the DPP-PSD firmware using areaDetector framework. This work should be considered as work in progress (WIP).

## Repository structure

| Folder         | Description                                      |
| -------------- | ------------------------------------------------ |
| ADCAEN1725SApp | Application folder that contains the support.    |
| caenSupport    | Folder that contains the 3rd party lib from CAEN. |
| configure      | EPICS configure folder.                           |
| iocBoot        | iocBoot to be use with the Application.           |

## Compiling

The module itself is a standard areaDetector, please check the areaDetector module.

### Prerequisites

One should have the following dependencies installed:
- EPICS base
- areaDetector and relevant EPICS modules (asyn, calc etc)
- Relevant areaDetector system dependencies (HDF5, xml2 etc)

### Configure

The first step is to create a `RELEASE.local` or edit `RELEASE` and provide the information about the `EPICS_BASE` and `AREA_DETECTOR` paths.
Again this folder use the EPICS build system so one should adapt according to its site configuration.

### Compiling

Finally the compilation can be started with simply typing:
```
make
```

### Testing

Use the `st.cmd` provided in the iocBoot folder to test the application. One must adapt the `st.cmd` to fit with its CONET adaptor.