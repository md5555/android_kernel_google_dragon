/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * @brief <b>nVIDIA Driver Development Kit:
 *           Private functions for the uart Ddk driver</b>
 *
 * @b Description:  Defines the private interfacing functions for the uart
 * hw interface.
 *
 */

#ifndef INCLUDED_DDKUART_HW_PRIVATE_H
#define INCLUDED_DDKUART_HW_PRIVATE_H

/**
 * @defgroup nvddk_uart Universal Asynchronous Receiver Transmitter (UART)
 * Controller hw interface API
 *
 * This is the Universal Asynchronous Receiver Transmitter (UART) hw interface
 * controller api.
 *
 * @ingroup nvddk_modules
 * @{
 *
 */

#include "nvcommon.h"
#include "nvddk_uart.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Combines the definition of the uart register and modem signals.
 */
typedef struct UartHwRegistersRec
{
    // Channel instance Id.
    NvU32 ChannelId;
    
    // Virtual base address of the uart hw register.
    NvU32 *pRegsVirtBaseAdd;

    // Physical base address of the uart hw register.
    NvRmPhysAddr RegsPhyBaseAdd;

    // Register bank size.
    NvU32 BankSize;

    // Uart shadow registers to keep the s/w value of registers

    // Currently programmed Fcr register value.
    NvU32 FcrRegister;

    // Currently programmed Ier register value.
    NvU32 IerRegister;

    // Currently programmed Lcr register value.
    NvU32 LcrRegister;

    // Currently programmed Mcr register value.
    NvU32 McrRegister;

    // Currently programmed irda csr register value.
    NvU32 IrdaCsrRegister;

    // Currently programmed Dlm register value.
    NvU32 DlmRegister;

    // Currently programmed Dll register value.
    NvU32 DllRegister;

    // Tells whether the dma mode is selected or not.
    NvBool IsDmaMode;

    // Modem signal status, 1 on the particular location shows the state to high.
    NvU32 ModemSignalSatus;
    
    // Tells whether the Rts hw flow control is  supported or not
    NvBool IsRtsHwFlowSupported;

    // Tells whether the end of data interrupt is  supported or not
    NvBool IsEndOfDataIntSupport;

    // Store the RTS line status
    NvBool IsRtsActive;

    NvBool IsDtrActive;
    
    NvU32 TransmitTrigLevel;
} UartHwRegisters;

/**
 * Combines the uart interrupt sources.
 */
typedef enum
{
    // No Uart interrupt source.
    UartHwInterruptSource_None = 0x0,

    // Receive Uart interrupt source.
    UartHwInterruptSource_Receive,

    // Receive error Uart interrupt source.
    UartHwInterruptSource_ReceiveError,

    // Rx timeout interrupt source.
    UartHwInterruptSource_RxTimeout,

    // End of data interrupt source.
    UartHwInterruptSource_EndOfData,

    // Transmit interrupt source.
    UartHwInterruptSource_Transmit,

    // Modem control interrupt source.
    UartHwInterruptSource_ModemControl,

    UartHwInterruptSource_Force32 = 0x7FFFFFFF
} UartHwInterruptSource;

/**
 * Combines the uart hw data direction.
 */
typedef enum
{
    // Invalid fifo type.
    UartDataDirection_Invalid = 0x0,

    // Receive direction.
    UartDataDirection_Receive = 0x1,

    // Transmit direction.
    UartDataDirection_Transmit = 0x2,

    // Both, receive and transmit data direction.
    UartDataDirection_Both = 0x3,

    UartDataDirection_Force32 = 0x7FFFFFFF
} UartDataDirection;

/**
 * Initialize the uart hw registers parameters. It does not write anything on 
 * the controller register.
 */
void 
NvDdkPrivUartHwRegisterInitialize(
    NvU32 CommChannelId, 
    UartHwRegisters *pUartHwRegs);
/**
 * Initialize the uart fifos.
 */
void NvDdkPrivUartHwInitFifo(UartHwRegisters *pUartHwRegs);

/**
 * Get the clock source frequency required for the given baud rate.
 */
void
NvDdkPrivUartHwGetReqdClockSourceFreq(
    NvU32 BaudRate,
    NvU32 *pMinClockFreqReqd,
    NvU32 *pClockFreqReqd,
    NvU32 *pMaxClockFreqReqd);

/**
 * Set the baud rate of uart communication.
 */
NvError
NvDdkPrivUartHwSetBaudRate(
    UartHwRegisters *pUartHwRegs,
    NvU32 BaudRate,
    NvU32 ClockSourceFreq);
/**
 * Set the parity of uart communication.
 */
NvError
NvDdkPrivUartHwSetParityBit(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartParity ParityBit);

/**
 * Set the data length of uart communication.
 */
NvError
NvDdkPrivUartHwSetDataLength(
    UartHwRegisters *pUartHwRegs,
    NvU32 DataLength);

/**
 * Set the stop bit of uart communication.
 */
NvError NvDdkPrivUartHwSetStopBit(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartStopBit StopBit);

/**
 * Start/stop sending the break control signal.
 */
void NvDdkPrivUartHwSetBreakControlSignal(
    UartHwRegisters *pUartHwRegs,
    NvBool IsStart);

/**
 * Reset the uart fifos.
 */
void
NvDdkPrivUartHwResetFifo(
    UartHwRegisters *pUartHwRegs,
    UartDataDirection FifoType);

/**
 * Set the fifo trigger level of uart fifo.
 */
void
NvDdkPrivUartHwSetFifoTriggerLevel(
    UartHwRegisters *pUartHwRegs,
    UartDataDirection FifoType,
    NvU32 TriggerLevel);

/**
 * Set the dma mode operation of uart
 */
void 
NvDdkPrivUartHwSetDmaMode(
    UartHwRegisters *pUartHwRegs, 
    NvBool IsEnable);

/**
 * Write into the transmit fifo register with the trigger level.
 * This function should be call when tx trigger level is reached.
 */
NvU32
NvDdkPrivUartHwWriteInTransmitFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pTxBuff,
    NvU32 BytesRequested);

/**
 * Write into the transmit fifo register and check the fifo status before writing
 * into the tx fifo.
 * With every write, the fifo status will be checked.
 */
NvU32
NvDdkPrivUartHwPollingWriteInTransmitFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pTxBuff,
    NvU32 BytesRequested);
    
/**
 * Check whether the Transmit fiof is empty or not.
 * Returns TRUE if the Tx fifo is empty otherwise returns FALSE.
 */
NvBool NvDdkPrivUartHwIsTransmitFifoEmpty(UartHwRegisters *pUartHwRegs);

/**
 * Check whether break condition is received or not.
 */
NvBool NvDdkPrivUartHwIsBreakSignalDetected(UartHwRegisters *pUartHwRegs);

/**
 * Get the Rx fifo status whether data is available in the rx fifo or not.
 */
NvBool NvDdkPrivUartHwIsDataAvailableInFifo(UartHwRegisters *pUartHwRegs);

/**
 * Read the data from the receive fifo.
 */
NvError
NvDdkPrivUartHwReadFromReceiveFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pRxBuff,
    NvU32 MaxBytesRequested,
    NvU32 *bytesRead);

/**
 * Clear all the uart interrupt.
 */
void NvDdkPrivUartHwClearAllInt(UartHwRegisters *pUartHwRegs);

/**
 * Configure the receive interrupt.
 */
void NvDdkPrivUartHwSetReceiveInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable,
    NvBool IsRxUsingApbDma);

/**
 * Configure the transmit interrupt.
 */
void NvDdkPrivUartHwSetTransmitInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable);

/**
 * Configure the modem control interrupt.
 */
void NvDdkPrivUartHwSetModemControlInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable);

/**
 * Get the interrupt reason.
 */
UartHwInterruptSource
NvDdkPrivUartHwGetInterruptReason(
    UartHwRegisters *pUartHwRegs);

/**
 * Enable/disable the Irda coding.
 */
void
NvDdkPrivUartHwSetIrdaCoding(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable);

/**
 * Set the output modem signals to high or low.
 */
void
NvDdkPrivUartHwSetModemSignal(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartSignalName SignalName,
    NvBool IsActive);

/**
 * Get the CTS status from the line.
 */
NvBool NvDdkPrivUartHwIsCtsActive(UartHwRegisters *pUartHwRegs);

/**
 * Read the modem signal level and update the state of the signal in the uart hw
 * register structure.
 */
void NvDdkPrivUartHwUpdateModemSignal(UartHwRegisters *pUartHwRegs);

/**
 * Enable/disable the hw flow control for transmitting the signal. If it is
 * enabled then it wil only transmit the data if CTS line is high.
 */
void
NvDdkPrivUartHwSetHWFlowControl(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable);

/**
 * Reconfigure the uart controller register. The shadow register content is
 * written into the uart controller register.
 */
void NvDdkPrivUartHwReconfigureRegisters(UartHwRegisters *pUartHwRegs);

/** @}*/

#if defined(__cplusplus)
}
#endif

#endif  // INCLUDED_DDKUART_HW_PRIVATE_H
