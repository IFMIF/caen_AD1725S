/*!
		-----------------------------------------------------------------------------

					   --- CAEN SpA - Computing Systems Division ---

		-----------------------------------------------------------------------------

		CAENVMElib.h

		-----------------------------------------------------------------------------

		Created: March 2004

		-----------------------------------------------------------------------------
*/
#ifndef __CAENVMELIB_H
#define __CAENVMELIB_H

#include <stdint.h>

#include "CAENVMEoslib.h"
#include "CAENVMEtypes.h"

/*!
* @defgroup VersionMacros Version macros
* @brief Macros to define the library version.
* @{ */
#define CAENVME_VERSION_MAJOR		4																													//!< Major version
#define CAENVME_VERSION_MINOR		0																													//!< Minor version
#define CAENVME_VERSION_PATCH		2																													//!< Patch version
#define CAENVME_VERSION				CAENVME_STR(CAENVME_VERSION_MAJOR) "." CAENVME_STR(CAENVME_VERSION_MINOR) "." CAENVME_STR(CAENVME_VERSION_PATCH)	//!< The version string as Major.Minor.Patch
#define CAENVME_VERSION_NUMBER		((CAENVME_VERSION_MAJOR) * 10000 + (CAENVME_VERSION_MINOR) * 100 + (CAENVME_VERSION_PATCH))							//!< The version number: for example version 1.2.3 gives 10203
/*! @} */

/**
* @defgroup Functions API
* @brief Application programming interface.
* @{ */

/**
* @defgroup Generic Generic
* @brief Generic functions.
* @defgroup VME VME communication
* @brief VME communication functions.
* @defgroup IRQ Interrupt control
* @brief Interrupt control functions.
* @defgroup Pulser Pulser control
* @brief Pulser control functions.
* @defgroup Scaler Scaler control
* @brief Scaler control functions.
* @defgroup IO I/O control
* @brief I/O control functions.
*/

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * This function allows decoding an error code.
 *
 * @ingroup					Generic
 * @since					v2.3
 * @param[in] Code			The error code to decode.
 * @return					A string describing the error condition.
 */
CAENVME_DLLAPI const char* CAENVME_API
CAENVME_DecodeError(CVErrorCodes Code);

/**
 * This function permits the reading of the software release of the library.
 *
 * @ingroup					Generic
 * @param[out] SwRel		Returns the software release of the library.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SWRelease(char *SwRel);

/**
 * This function permits to read of the release of the firmware loaded
 * into the device.
 * 
 * @ingroup					Generic
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] FWRel		Returns the firmware release of the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_BoardFWRelease(int32_t Handle, char *FWRel);

/**
 * This function allows the reading of the software release of the device
 * driver loaded in the PC.
 *
 * @ingroup					Generic
 * @since					v2.4
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Rel			Returns the software release of the device driver.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_DriverRelease(int32_t Handle, char *Rel);

/**
 * This function permits resetting of the device.
 *
 * @ingroup					Generic
 * @since					v2.5
 * @pre						Works only for A2818 and A3818 on Linux and for A3818 on Windows.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_DeviceReset(int32_t Handle);

/**
 * Open a connection to a device attached to the PC and generates an opaque handle to identify that module.
 * The returned handle must be eventually closed with CAENVME_End().
 *
 * @ingroup								Generic
 * @deprecated							Use CAENVME_Init2() instead.
 * @warning								Does not support A4818 with PID > `SHRT_MAX` (usually 32767) and V4718.
 * @param[in] BdType					The model of the bridge.
 * @param[in] LinkNum_or_PID			The link number or the PID for those boards that support it (A4818/V3718). Don't care for V1718/V3718.
 * @param[in] ConetNode_or_USBNumber	The CONET number in the daisy-chain loop or USB number for USB connection via V1718 or V3718.
 * @param[out] Handle					The handle that identifies the device.
 * @return								::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DEPRECATED(3.4.0, CAENVME_Init2,

CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_Init(CVBoardTypes BdType, short LinkNum_or_PID, short ConetNode_or_USBNumber, int32_t* Handle)

);

/**
 * Open a connection to a device attached to the PC and generates an opaque handle to identify that module.
 * In the case of CONET connection (by V2718, V3718 or V4718), it is required to specify also the @p ConetNode,
 * due to the possibility of an optical Daisy chain with a CONET master device. The returned handle must be
 * eventually closed with CAENVME_End().
 * The @p Arg arg is a pointer to a value representing the link. The requested pointed type depends
 * on the type of connection specified by @p BdType (more details can be found on the official documentation):
 * 	- Pointer to the USB link number (as `uint32_t`), in case of an USB connection via V1718 or V3718.
 * 	- Pointer to the optical link number (as `uint32_t`), in case of an optical link connection via A2818, A3818 or A5818.
 * 	- Pointer to the PID (as `uint32_t`), in case of an USB connection to the A4818 or V4718.
 * 	- Pointer to the host (as `char` null-terminated string), consisting of either an hostname or an IPv4 address, in case of an Ethernet connection to the V4718.
 *
 * @ingroup					Generic
 * @since					v3.3.0
 * @param[in] BdType		The model of the bridge.
 * @param[in] Arg			See description for details.
 * @param[in] ConetNode		The CONET number in the daisy-chain loop. Don't care for connections that don't support it.
 * @param[out] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_Init2(CVBoardTypes BdType, const void* Arg, short ConetNode, int32_t* Handle);

/**
 * This function notifies the library about the end of work and frees the
 * allocated resources.
 *
 * @ingroup					Generic
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_End(int32_t Handle);

/**
 * This function performs a single VME read cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Data			The data to read from the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ReadCycle(int32_t Handle, uint32_t Address, void *Data,
				  CVAddressModifier AM, CVDataWidth DW);

/**
 * This function performs a Read-Modify-Write cycle. The @p Data parameter is
 * bidirectional: it is used to write the value to the VME bus and to return
 * the value read.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in, out] Data		The data read and then written to the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_RMWCycle(int32_t Handle, uint32_t Address, void *Data,
				 CVAddressModifier AM, CVDataWidth DW);

/**
 * This function performs a single VME write cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] Data			The data to be written to the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_WriteCycle(int32_t Handle, uint32_t Address, const void *Data,
				   CVAddressModifier AM, CVDataWidth DW);

/**
 * This function performs a sequence of VME read cycles.
 *
 * @ingroup					VME
 * @since					v2.2
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Addrs			The VME bus addresses.
 * @param[out] Buffer		The data read from the VME bus. Pointed integer size does not depend on @p DWs.
 * @param[in] NCycles		The number of read cycles to perform.
 * @param[in] AMs			The address modifiers.
 * @param[in] DWs			The data widths.
 * @param[out] ECs			The error codes relative to each cycle.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_MultiRead(int32_t Handle, const uint32_t *Addrs, uint32_t *Buffer,
		int NCycles, const CVAddressModifier *AMs, const CVDataWidth *DWs, CVErrorCodes *ECs);

/**
 * This function performs a sequence of VME write cycles.
 *
 * @ingroup					VME
 * @since					v2.2
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Addrs			The VME bus addresses.
 * @param[in] Buffer		The data to be written to the VME bus. Pointed integer size does not depend on @p DWs.
 * @param[in] NCycles		The number of read cycles to perform.
 * @param[in] AMs			The address modifiers.
 * @param[in] DWs			The data widths.
 * @param[out] ECs			The error codes relative to each cycle.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_MultiWrite(int32_t Handle, const uint32_t *Addrs, const uint32_t *Buffer,
		int NCycles, const CVAddressModifier *AMs, const CVDataWidth *DWs, CVErrorCodes *ECs);

/**
 * This function performs a VME block transfer read cycle. It can be used to perform
 * MBLT transfers using 64-bit data width.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Buffer		The data read from the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_BLTReadCycle(int32_t Handle, uint32_t Address, void *Buffer,
					 int Size, CVAddressModifier AM, CVDataWidth DW, int *Count);

/**
 * This function performs a VME block transfer read cycle. It can be used to perform
 * MBLT transfers using 64-bit data width. The address is not incremented on the VMEBus
 * during the cycle.
 *
 * @ingroup					VME
 * @since					v2.3
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Buffer		The data read from the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_FIFOBLTReadCycle(int32_t Handle, uint32_t Address, void *Buffer,
						 int Size, CVAddressModifier AM, CVDataWidth DW, int *Count);

/**
 * This function performs a VME multiplexed block transfer read cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Buffer		The data read from the VME bus.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_MBLTReadCycle(int32_t Handle, uint32_t Address, void *Buffer,
					  int Size, CVAddressModifier AM, int *Count);

/**
 * This function performs a VME multiplexed block transfer read cycle.
 * The address is not incremented on the VMEBus during the cycle. 
 *
 * @ingroup					VME
 * @since					v2.3
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Buffer		The data read from the VME bus.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_FIFOMBLTReadCycle(int32_t Handle, uint32_t Address, void *Buffer,
						  int Size, CVAddressModifier AM, int *Count);

/**
 * This function performs a VME block transfer write cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] Buffer		The data to be written to the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_BLTWriteCycle(int32_t Handle, uint32_t Address, const void *Buffer,
					  int Size, CVAddressModifier AM, CVDataWidth DW, int *Count);

/**
 * This function performs a VME block transfer write cycle. The address
 * is not incremented on the VMEBus during the cycle. 
 *
 * @ingroup					VME
 * @since					v2.3
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] Buffer		The data to be written to the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_FIFOBLTWriteCycle(int32_t Handle, uint32_t Address, const void *Buffer,
					  int Size, CVAddressModifier AM, CVDataWidth DW, int *Count);

/**
 * This function performs a VME multiplexed block transfer write cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] Buffer		The data to be written to the VME bus.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_MBLTWriteCycle(int32_t Handle, uint32_t Address, const void *Buffer,
					   int Size, CVAddressModifier AM, int *Count);

/**
 * This function performs a VME multiplexed block transfer write cycle.
 * The address is not incremented on the VMEBus during the cycle. 
 *
 * @ingroup					VME
 * @since					v2.3
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] Buffer		The data to be written to the VME bus.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[out] Count		The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_FIFOMBLTWriteCycle(int32_t Handle, uint32_t Address, const void *Buffer,
						   int Size, CVAddressModifier AM, int *Count);

/**
 * This function performs a VME address only cycle. It can be used to perform
 * MBLT transfers using 64-bit data width.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] AM			The address modifier.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ADOCycle(int32_t Handle, uint32_t Address, CVAddressModifier AM);

/**
 * This function performs a VME address only with a handshake cycle.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[in] AM			The address modifier.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ADOHCycle(int32_t Handle, uint32_t Address, CVAddressModifier AM);

/**
 * When using the CONET connection, this function performs a VME interrupt acknowledge cycle.
 *
 * @ingroup					VME
 * @pre						Works only for A2818, A3818 and A5818.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Level			The IRQ level to acknowledge.
 * @param[out] IntStatusID	The interrupt Status/ID. Pointed integer size is specified by @p DW.
 * @param[in] DW			The data width.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_IACKCycle(int32_t Handle, CVIRQLevels Level, void* IntStatusID, CVDataWidth DW);

/**
 * When using the CONET connection, this function returns a bitmask indicating the active IRQ lines.
 *
 * @ingroup					IRQ
 * @pre						Works only for A2818, A3818 and A5818.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Mask			A bit-mask indicating the active IRQ lines.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_IRQCheck(int32_t Handle, CAEN_BYTE *Mask);

/**
 * When using the CONET connection, this function enables the IRQ lines specified by a mask.
 *
 * @ingroup					IRQ
 * @pre						Works only for A2818, A3818 and A5818.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			A bit-mask indicating the IRQ lines.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_IRQEnable(int32_t Handle, uint32_t Mask);

/**
 * When using the CONET connection, this function disables all the IRQ lines.
 * Lines specified by @p Mask are cleared from the internal registers. For
 * example, calling this function with @p Mask set to zero does not modify
 * the active lines mask: a subsequent call to CAENVME_IRQEnable() will
 * enable those lines that were still enabled just before this call even if
 * not specified by the @p Mask argument of CAENVME_IRQEnable().
 *
 * @ingroup					IRQ
 * @pre						Works only for A2818, A3818 and A5818.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			A bit-mask indicating the IRQ lines.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_IRQDisable(int32_t Handle, uint32_t Mask);

/**
 * When using the CONET connection, this function waits for the IRQ lines
 * specified by the mask until one of them raises, or the timeout expires.
 *
 * @ingroup					IRQ
 * @pre						Works only for A2818, A3818 and A5818.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			A bit-mask indicating the IRQ lines.
 * @param[in] Timeout		Timeout in milliseconds.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_IRQWait(int32_t Handle, uint32_t Mask, uint32_t Timeout);

/**
 * This function permits configuring the pulsers embedded on the Bridge (Pulser A and Pulser B).
 * All the timing parameters are expressed in the specified time units.
 *
 * @ingroup					Pulser
 * @pre						Works only for V1718/VX1718, V2718/VX2718, and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] PulSel		The pulser to configure.
 * @param[in] Period		The period of the pulse in time units.
 * @param[in] Width			The width of the pulse in time units.
 * @param[in] Unit			The time unit for the pulser configuration.
 * @param[in] PulseNo		The number of pulses to generate (0 = infinite).
 * @param[in] Start			The source signal to start the pulse burst. The start signal source can optionally
 *							be front panel button or software (::cvManualSW), input signal 0 (::cvInputSrc0),
 *							input signal 1 (::cvInputSrc1), or inputs coincidence (::cvCoincidence).
 * @param[in] Reset			The source signal to stop the pulse burst. The reset source signal can optionally
 *							be front panel button or software (::cvManualSW) or, for pulser A the input signal 0
 *							(::cvInputSrc0), for pulser B the input signal 1 (::cvInputSrc1).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetPulserConf(int32_t Handle, CVPulserSelect PulSel, unsigned char Period,
					  unsigned char Width, CVTimeUnits Unit, unsigned char PulseNo,
					  CVIOSources Start, CVIOSources Reset);

/**
 * This function permits configuring the scaler embedded on the Bridge.
 *
 * @ingroup					Scaler
 * @pre						Works only for V1718/VX1718 and V2718/VX2718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Limit			The counter limit for the scaler (0 - 1023 over 10 bits).
 * @param[in] AutoReset		Enable/disable the counter auto reset.
 * @param[in] Hit			The source signal for the signal to count.
 * @param[in] Gate			The source signal for the gate.
 * @param[in] Reset			The source signal to stop the counter.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScalerConf(int32_t Handle, short Limit, short AutoReset,
					  CVIOSources Hit, CVIOSources Gate, CVIOSources Reset);

/**
 * This function permits configuring the output lines of the Bridge.
 *
 * @ingroup					IO
 * @note					Refer to the official documentation for more details on the arguments of this function
 * @warning					Works differently for V1718/VX1718/V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] OutSel		The output line to configure.
 * @param[in] OutPol		The output line polarity.
 * @param[in] LEDPol		The output LED polarity.
 * @param[in] Source		The source signal to propagate to the output line.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetOutputConf(int32_t Handle, CVOutputSelect OutSel, CVIOPolarity OutPol,
					  CVLEDPolarity LEDPol, CVIOSources Source);

/**
 * This function permits the configuration of the input lines of the Bridge. It
 * is possible to specify the polarity for the line and the LED.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] InSel			The input line to configure.
 * @param[in] InPol			The input line polarity.
 * @param[in] LEDPol		The output LED polarity.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetInputConf(int32_t Handle, CVInputSelect InSel, CVIOPolarity InPol,
					 CVLEDPolarity LEDPol);

/**
 * This function permits the reading of the pulsers configuration.
 *
 * @ingroup					Pulser
 * @pre						Works only for V1718/VX1718, V2718/VX2718, and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] PulSel		The pulser to configure.
 * @param[out] Period		The period of the pulse in time units.
 * @param[out] Width		The width of the pulse in time units.
 * @param[out] Unit			The time unit for the pulser configuration.
 * @param[out] PulseNo		The number of pulses to generate (0 = infinite).
 * @param[out] Start		The source signal to start the pulse burst.
 * @param[out] Reset		The source signal to stop the pulse burst.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetPulserConf(int32_t Handle, CVPulserSelect PulSel, unsigned char *Period,
					  unsigned char *Width, CVTimeUnits *Unit, unsigned char *PulseNo,
					  CVIOSources *Start, CVIOSources *Reset);

/**
 * This function permits the reading of the scaler configuration.
 *
 * @ingroup					Scaler
 * @pre						Works only for V1718/VX1718 and V2718/VX2718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Limit		The counter limit for the scaler (0 - 1023 over 10 bits).
 * @param[out] AutoReset	The auto-reset configuration.
 * @param[out] Hit			The source signal for the signal to count.
 * @param[out] Gate			The source signal for the gate.
 * @param[out] Reset		The source signal to stop the counter.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScalerConf(int32_t Handle, short *Limit, short *AutoReset,
					  CVIOSources *Hit, CVIOSources *Gate, CVIOSources *Reset);

/**
 * This function permits the reading of the output lines configuration.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] OutSel		The output line to configure.
 * @param[out] OutPol		The output line polarity.
 * @param[out] LEDPol		The output LED polarity.
 * @param[out] Source		The source signal to propagate to the output line.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetOutputConf(int32_t Handle, CVOutputSelect OutSel, CVIOPolarity *OutPol,
					  CVLEDPolarity *LEDPol, CVIOSources *Source);

/**
 * This function permits the reading of the input lines configuration.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] InSel			The input line to configure.
 * @param[out] InPol		The input line polarity.
 * @param[out] LEDPol		The output LED polarity.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetInputConf(int32_t Handle, CVInputSelect InSel, CVIOPolarity *InPol,
					 CVLEDPolarity *LEDPol);

/**
 * This function permits to read the accessible internal registers of the Bridge or of the CONET master.
 *
 * @ingroup					Generic
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Reg			The internal register to read. Type is #CVRegisters for historical reasons,
 *							but values are not restricted to that enumeration. Since #CVRegisters is guaranteed
 *							to be compatible with either `int` or `unsigned int`, you can safely cast any value
							of these types to the enumerated type. Refer to the device User Manual for a detailed
							registers description.
 * @param[out] Data			The data read from the module.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ReadRegister(int32_t Handle, CVRegisters Reg, unsigned int *Data);

/**
 * This function permits to write the accessible internal registers of the Bridge or of the CONET master.
 *
 * @ingroup					Generic
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Reg			The internal register to read. Type is #CVRegisters for historical reasons,
 *							but values are not restricted to that enumeration. Since #CVRegisters is guaranteed
 *							to be compatible with either `int` or `unsigned int`, you can safely cast any value
							of these types to the enumerated type. Refer to the device User Manual for a detailed
							registers description.
 * @param[in] Data			The data to be written to the module.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_WriteRegister(int32_t Handle, CVRegisters Reg, unsigned int Data);

/**
 * This function sets the specified lines.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			The lines to be set (refer to the #CVOutputRegisterBits to compose and decode the bitmask).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetOutputRegister(int32_t Handle, unsigned short Mask);

/**
 * This function clears the specified lines.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			The lines to be cleared (refer to the #CVOutputRegisterBits to compose and decode the bitmask).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ClearOutputRegister(int32_t Handle, unsigned short Mask);

/**
 * The function produces a pulse with the lines specified by setting and then
 * clearing them.
 *
 * @ingroup					IO
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mask			The lines to be pulsed (refer to #CVOutputRegisterBits to compose and decode the bitmask).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_PulseOutputRegister(int32_t Handle, unsigned short Mask);

/**
 * This function reads the VME data display on the front panel of the module.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The values read from the module.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ReadDisplay(int32_t Handle, CVDisplay *Value);

/**
 * This function sets the behavior of the VME bus arbiter on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The type of VME bus arbitration to implement.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetArbiterType(int32_t Handle, CVArbiterTypes Value);

/**
 * This function sets the behavior of the VME bus requester on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The type of VME bus requester to implement.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetRequesterType(int32_t Handle, CVRequesterTypes Value);

/**
 * This function sets the release policy of the VME bus on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The type of VME bus release policy to implement.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetReleaseType(int32_t Handle, CVReleaseTypes Value);

/**
 * This function sets the specified VME bus requester priority level on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The type of VME bus requester priority level to set.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetBusReqLevel(int32_t Handle, CVBusReqLevels Value);

/**
 * This function sets the specified VME bus timeout on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The value of VME bus timeout to set.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetTimeout(int32_t Handle, CVVMETimeouts Value);

/**
 * This function sets the Location Monitor.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The address to be monitored.
 * @param[in] AM			The address modifier.
 * @param[in] Write			Flag to specify read or write cycle types.
 * @param[in] Lword			Flag to specify long-word cycle type.
 * @param[in] Iack			Flag to specify interrupt acknowledge cycle type.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetLocationMonitor(int32_t Handle, uint32_t Address, CVAddressModifier AM,
						   short Write, short Lword, short Iack);

/**
 * This function enables/disables the auto-increment of the VME addresses
 * during the block transfer cycles. With the FIFO mode enabled, the addresses
 * are not incremented.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Value			The FIFO mode read setting (0 = enabled, other values = disabled).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetFIFOMode(int32_t Handle, short Value);

/**
 * This function gets the type of VME bus arbiter implemented on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The type of VME bus arbitration implemented.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetArbiterType(int32_t Handle, CVArbiterTypes *Value);

/**
 * This function gets the type of VME bus requester implemented on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The type of VME bus requester implemented.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetRequesterType(int32_t Handle, CVRequesterTypes *Value);

/**
 * This function gets the type of VME bus release implemented on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The type of VME bus release policy implemented.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetReleaseType(int32_t Handle, CVReleaseTypes *Value);

/**
 * This function reads the type of VME bus requester priority level implemented on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The type of VME bus requester priority level.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetBusReqLevel(int32_t Handle, CVBusReqLevels *Value);

/**
 * This function reads the specified VME bus timeout setting of the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The value of VME bus timeout.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetTimeout(int32_t Handle, CVVMETimeouts *Value);

/**
 * This function reads the auto-increment configuration of the VME addresses during the block
 * transfer cycles.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The FIFO mode read setting (0 = enabled, other values = disabled).
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetFIFOMode(int32_t Handle, short *Value);

/**
 * This function performs a system reset on the Bridge.
 *
 * @ingroup					VME
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SystemReset(int32_t Handle);

/**
 * This function resets the counter of the scaler.
 *
 * @ingroup					Scaler
 * @pre						Works only for V1718/VX1718 and V2718/VX2718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ResetScalerCount(int32_t Handle);

/**
 * This function enables the gate of the scaler.
 *
 * @ingroup					Scaler
 * @pre						Works only for V1718/VX1718 and V2718/VX2718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_EnableScalerGate(int32_t Handle);

/**
 * This function disables the gate of the scaler.
 *
 * @ingroup					Scaler
 * @pre						Works only for V1718/VX1718 and V2718/VX2718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_DisableScalerGate(int32_t Handle);

/**
 * This function starts the generation of the pulse burst if the specified pulser is configured
 * for manual/software operation.
 *
 * @ingroup					Pulser
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] PulSel		The pulser to configure.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_StartPulser(int32_t Handle, CVPulserSelect PulSel);

/**
 * This function stops the generation of the pulse burst if the specified pulser is configured
 * for manual/software operation.
 *
 * @ingroup					Pulser
 * @pre						Works only for V1718/VX1718, V2718/VX2718 and V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] PulSel		The pulser to configure.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_StopPulser(int32_t Handle, CVPulserSelect PulSel);

/**
 * This function writes the data into the specified flash page.
 *
 * @ingroup					Generic
 * @warning					Experts only.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Data			The data to write.
 * @param[in] PageNum		The flash page number to write.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_WriteFlashPage(int32_t Handle, const unsigned char *Data, int PageNum);

/**
 * This function reads the data into the specified flash page.
 *
 * @ingroup					Generic
 * @warning					Experts only.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Data			The data read.
 * @param[in] PageNum		The flash page number to read.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_ReadFlashPage(int32_t Handle, unsigned char *Data, int PageNum);

/**
 * This function erases the specified flash page.
 *
 * @ingroup					Generic
 * @warning					Experts only.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] PageNum		The flash page number to erase.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_EraseFlashPage(int32_t Handle, int PageNum);

/**
 * This function allows setting the scaler input source configuration for the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Source		The scaler source.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_InputSource(int32_t Handle, CVScalerSource Source);

/**
 * This function allows getting the scaler input source configuration from the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Source		The scaler source.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_InputSource(int32_t Handle, CVScalerSource* Source);

/**
 * This function allows setting the scaler gate source on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Source		The scaler source.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_GateSource(int32_t Handle, CVScalerSource Source);

/**
 * This function allows getting the scaler gate source on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Source		The scaler source.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_GateSource(int32_t Handle, CVScalerSource* Source);

/**
 * This function allows setting the scaler mode on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Mode			The scaler mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_Mode(int32_t Handle, CVScalerMode Mode);

/**
 * This function allows getting the scaler mode on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Mode			The scaler mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_Mode(int32_t Handle, CVScalerMode* Mode);

/**
 * This function allows setting the scaler clear source on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Source		The clear source mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_ClearSource(int32_t Handle, CVScalerSource Source);

/**
 * This function allows setting the scaler start source on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Source		The start source mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_StartSource(int32_t Handle, CVScalerSource Source);

/**
 * This function allows getting the scaler start source on the Bridge.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Source		The start source mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_StartSource(int32_t Handle, CVScalerSource* Source);

/**
 * This function allows setting the scaler continuous run mode.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] OnOff			The continuous run mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_ContinuousRun(int32_t Handle, CVContinuosRun OnOff);

/**
 * This function allows getting the scaler continuous run mode.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] OnOff		The continuous run mode.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_ContinuousRun(int32_t Handle, CVContinuosRun* OnOff);

/**
 * This function allows setting the scaler MaxHits mode.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The MaxHits value.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_MaxHits(int32_t Handle, uint16_t Value);

/**
 * This function allows getting the scaler MaxHits mode.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The MaxHits value.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_MaxHits(int32_t Handle, uint16_t* Value);

/**
 * This function allows setting the scaler D-Well time value.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The D-Well time value.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_DWellTime(int32_t Handle, uint16_t Value);

/**
 * This function allows setting the scaler D-Well time value.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @param[out] Value		The D-Well time value.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_GetScaler_DWellTime(int32_t Handle, uint16_t* Value);

/**
 * This function allows setting the scaler software start.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_SWStart(int32_t Handle);

/**
 * This function allows setting the scaler software stop.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_SWStop(int32_t Handle);

/**
 * This function allows setting the scaler software reset.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_SWReset(int32_t Handle);

/**
 * This function allows setting the scaler software open gate.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_SWOpenGate(int32_t Handle);

/**
 * This function allows setting the scaler software close gate.
 *
 * @ingroup					Scaler
 * @since v3.3.0
 * @pre						Works only for V3718/VX3718.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_SetScaler_SWCloseGate(int32_t Handle);

#ifndef _WIN32
/**
 * This function starts a VME block transfer read cycle. It can be used to perform MBLT
 * transfers using 64-bit data width. Please, take care to call the CAENVME_BLTReadWait()
 * function before any other call to a function with the same handle.
 *
 * @ingroup					VME
 * @pre						Linux only. Does not work with the USB bridges.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Address		The VME bus address.
 * @param[out] Buffer		The data read from the VME bus. Pointed integer size is specified by @p DW.
 * @param[in] Size			The size of the transfer in bytes.
 * @param[in] AM			The address modifier.
 * @param[in] DW			The data width.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_BLTReadAsync(int32_t Handle, uint32_t Address, void *Buffer,
					 int Size, CVAddressModifier AM, CVDataWidth DW);

/**
 * This function waits for the completion of a VME block transfer read cycle started
 * with the CAENVME_BLTReadAsync() function call.
 *
 * @ingroup					VME
 * @pre						Linux only. Does not work with the USB bridges.
 * @param[in] Handle		The handle that identifies the device.
 * @param[in] Count			The number of bytes transferred.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DLLAPI CVErrorCodes CAENVME_API
CAENVME_BLTReadWait(int32_t Handle, int *Count);

/**
 * This function waits resets a CONET slave.
 *
 * @ingroup					Generic
 * @deprecated				To be removed because the underlying functionality has never been implemented.
 * @pre						Linux only. Works only for A3818.
 * @param[in] Handle		The handle that identifies the device.
 * @return					::cvSuccess (0) in case of success, or a negative error code specified in #CVErrorCodes.
 */
CAENVME_DEPRECATED(3.5.0, none,

CAENVME_DLLAPI CVErrorCodes CAENVME_API
_CAENVME_OpticalLinkSlaveReset(int32_t Handle)

);

#endif

#ifdef __cplusplus
}
#endif // __cplusplus

/*! @} */

#endif // __CAENVMELIB_H
