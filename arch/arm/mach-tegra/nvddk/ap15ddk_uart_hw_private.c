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
 *           Private functions for the uart hw access</b>
 *
 * @b Description:  Implements  the private interfacing functions for the uart
 * hw interface.
 *
 */

#include "nvrm_drf.h"
#include "ddkuart_hw_private.h"
#include "nvrm_hardware_access.h"
#include "nvassert.h"

// hardware includes
#include "ap15/aruart.h"

// Allowed uart baud rate accuracy in percent. As per theory, it should be 5%
enum {UART_BAUDRATE_ACCURACY = 5};

#define UART_REG_READ32(pUartHwRegsVirtBaseAdd, reg) \
        NV_READ32((pUartHwRegsVirtBaseAdd) + ((UART_##reg##_0)/4))

#define UART_REG_WRITE32(pUartHwRegsVirtBaseAdd, reg, val) \
    do { \
        NV_WRITE32((((pUartHwRegsVirtBaseAdd) + ((UART_##reg##_0)/4))), (val)); \
    } while (0)

#define IS_POSSIBLE_RX_ERROR (NV_DRF_DEF(UART, LSR, BRK, BREAK) | \
                        NV_DRF_DEF(UART, LSR, FIFOE, ERR) | \
                            NV_DRF_DEF(UART, LSR, FERR, FRAME_ERR) | \
                                NV_DRF_DEF(UART, LSR, OVRF, OVERRUN_ERROR) | \
                                    NV_DRF_DEF(UART, LSR, PERR, PARITY_ERR))

#define IS_DATA_IN_FIFO NV_DRF_DEF(UART, LSR, RDR, DATA_IN_FIFO)

#define TX_EMPTY_STATUS (NV_DRF_DEF(UART, LSR, TMTY, EMPTY) | \
                                NV_DRF_DEF(UART, LSR, THRE, EMPTY))

/**
 * Initialize the uart register.
 */
void NvDdkPrivUartHwRegisterInitialize(NvU32 CommChannelId, UartHwRegisters *pUartHwRegs)
{
    // Initialize the member of Uart hw register structure.
    pUartHwRegs->ModemSignalSatus = 0;
    pUartHwRegs->IsRtsHwFlowSupported = NV_FALSE;
    pUartHwRegs->IsEndOfDataIntSupport = NV_FALSE;

    pUartHwRegs->FcrRegister = 0;
    pUartHwRegs->IerRegister = 0;
    pUartHwRegs->LcrRegister = 0;
    pUartHwRegs->McrRegister = 0;
    pUartHwRegs->DllRegister = 0;
    pUartHwRegs->DlmRegister = 0;

    pUartHwRegs->IsDmaMode = NV_TRUE;
    pUartHwRegs->pRegsVirtBaseAdd = NULL;
    pUartHwRegs->ChannelId = CommChannelId;
    pUartHwRegs->RegsPhyBaseAdd = 0;
    pUartHwRegs->BankSize = 0;
    pUartHwRegs->TransmitTrigLevel = 1;
    pUartHwRegs->IsRtsActive = NV_FALSE;
    pUartHwRegs->IsDtrActive = NV_FALSE;
}

/**
 * Initialize the uart fifos.
 */
void NvDdkPrivUartHwInitFifo(UartHwRegisters *pUartHwRegs)
{
    pUartHwRegs->FcrRegister = NV_DRF_DEF(UART, IIR_FCR, FCR_EN_FIFO, ENABLE);
    NvDdkPrivUartHwResetFifo(pUartHwRegs, UartDataDirection_Both);
    NvDdkPrivUartHwSetFifoTriggerLevel(pUartHwRegs, UartDataDirection_Both, 4);
    NvDdkPrivUartHwUpdateModemSignal(pUartHwRegs);
    NvDdkPrivUartHwSetModemSignal(pUartHwRegs, NvDdkUartSignalName_Rts, NV_FALSE);
 }

/**
 * Get the clock frequency required related to the baud rate.
 */
void
NvDdkPrivUartHwGetReqdClockSourceFreq(
    NvU32 BaudRate,
    NvU32 *pMinClockFreqReqd,
    NvU32 *pClockFreqReqd,
    NvU32 *pMaxClockFreqReqd)
{
    NvU32 MaxClockFreqReq = 0;
    NvU32 MinClockFreqReq = 0;
    NvU32 ClockFreqReqd = 0;

    *pMinClockFreqReqd = 1;
    *pClockFreqReqd = 1;
    *pMaxClockFreqReqd = 1;

    // Required clock frequency in KHz. 
    // Required clock freq is 16 times of the required baudrate.
    ClockFreqReqd = (BaudRate *16)/1000;
    if (!ClockFreqReqd)
        ClockFreqReqd = 1;

    // Minimum clock frequency required, -5%
    MinClockFreqReq = (NvU32)NvDiv64(((100 - UART_BAUDRATE_ACCURACY)* (NvU64)BaudRate *16), (1000*100));
    if (!MinClockFreqReq)
        MinClockFreqReq = 1;

    // Maximum clock frequency required, +5%
    MaxClockFreqReq = (NvU32)NvDiv64(((100 + UART_BAUDRATE_ACCURACY)* (NvU64)BaudRate *16),(1000*100));
    if (MaxClockFreqReq <= ClockFreqReqd)
        MaxClockFreqReq = ClockFreqReqd + 1;

    *pMinClockFreqReqd = MinClockFreqReq;
    *pClockFreqReqd = ClockFreqReqd;
    *pMaxClockFreqReqd = MaxClockFreqReq;
}

/**
 * Set the baud rate of uart communication.
 */
NvError
NvDdkPrivUartHwSetBaudRate(
    UartHwRegisters *pUartHwRegs,
    NvU32 BaudRate,
    NvU32 ClockSourceFreq)
{
    NvU32 BaudRateConstant;

    // Calculated baud rate constant for 5% boundary.
    NvU32 CalcBRL;
    NvU32 CalcBRH;

    // boundary of the baudrate.
    NvU32 BR_Dev_L;
    NvU32 BR_Dev_H;

    NvU32 ClockFreq;

    // Final baud rate constant.
    NvU32 FinalBaudRateConstant = 0;
    NvU32 BaudrateDll;
    NvU32 BaudrateDlm;
    NvU32 LcrValue;

    // Get the upper and lower baud rate constant calculation.
    BR_Dev_L = (BaudRate - (BaudRate * UART_BAUDRATE_ACCURACY)/100);
    BR_Dev_H = (BaudRate + (BaudRate * UART_BAUDRATE_ACCURACY)/100);

    // Search the clock source which gives the proper baud rate constant
    // for given baud rate.
    // Get the baud rate constant for given clock frequency.
    ClockFreq = ClockSourceFreq*1000;
    BaudRateConstant = (ClockFreq /(BaudRate*16));

    // If baud rate constant is becoming 0 then it will not be possible to set
    // the baud rate.
    if (!BaudRateConstant)
        return NvError_NotSupported;

    // Get the baud rate with calculated baud rate constant and +1.
    CalcBRH = (ClockFreq)/(BaudRateConstant *16);
    CalcBRL = (ClockFreq)/((BaudRateConstant+1) *16);

    // If calculated low baud rate is in up/down 5 percent baudrates then
    // we have got the correct clock source.
    if ((CalcBRL >= BR_Dev_L) && (CalcBRL <= BR_Dev_H))
        FinalBaudRateConstant = BaudRateConstant+1;
    else if ((CalcBRH >= BR_Dev_L) && (CalcBRH <= BR_Dev_H))
        FinalBaudRateConstant = BaudRateConstant;
    else
        return NvError_NotSupported;

    // Get the lower byte and upper byte of the baud rate constant.
    BaudrateDll = FinalBaudRateConstant & 0xFF;
    BaudrateDlm = (FinalBaudRateConstant >> 8) & 0xFF;

    pUartHwRegs->DllRegister = BaudrateDll;
    pUartHwRegs->DlmRegister = BaudrateDlm;

    // Enable the LCR-DLAB.
    LcrValue = pUartHwRegs->LcrRegister;
    LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, ENABLE, LcrValue);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);

    // Program the lower and upper bytes of the baud rate constant.
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, THR_DLAB_0, BaudrateDll);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, BaudrateDlm);

    // Disable the DLAB.
    LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, DISABLE, LcrValue);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);
    return NvSuccess;
}

/**
 * Set the parity of uart communication.
 */
NvError
NvDdkPrivUartHwSetParityBit(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartParity ParityBit)
{
    NvU32 LcrValue;

    // Get the Lcr register value and set the parity as per per requirement.
    LcrValue = pUartHwRegs->LcrRegister;
    switch (ParityBit)
    {
        case NvDdkUartParity_None:
            // No parity
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, NO_PARITY, LcrValue);
            break;

        case NvDdkUartParity_Odd:
            // Odd parity
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, PARITY, LcrValue);
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, EVEN, DISABLE, LcrValue);
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, SET_P, NO_PARITY, LcrValue);
            break;

        case NvDdkUartParity_Even:
            // Even parity.
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, PAR, PARITY, LcrValue);
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, EVEN, ENABLE, LcrValue);
            LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, SET_P, NO_PARITY, LcrValue);
            break;

        default:
            // Unknown type of parameter.
            return  NvError_NotSupported;
    }

    pUartHwRegs->LcrRegister = LcrValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);
    return NvSuccess;
}

/**
 * Set the data length of uart communication.
 */
NvError
NvDdkPrivUartHwSetDataLength(
    UartHwRegisters *pUartHwRegs,
    NvU32 DataLength)
{
    NvU32 LcrValue;

    // Get the lcr register value.
    LcrValue = pUartHwRegs->LcrRegister;
    if (DataLength == 5)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_5, LcrValue);
    else if (DataLength == 6)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_6, LcrValue);
    else if (DataLength == 7)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_7, LcrValue);
    else if (DataLength == 8)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, WD_SIZE, WORD_LENGTH_8, LcrValue);
    else
        return NvError_NotSupported;

    pUartHwRegs->LcrRegister = LcrValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);
    return NvSuccess;
}

/**
 * Set the stop bit of uart communication.
 */
NvError NvDdkPrivUartHwSetStopBit(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartStopBit StopBit)
{
    NvU32 LcrValue;

    // Get the lcr register value and configured for the stop bit.
    LcrValue = pUartHwRegs->LcrRegister;
    if (StopBit == NvDdkUartStopBit_1)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, STOP, DISABLE, LcrValue);
    else if ((StopBit == NvDdkUartStopBit_1_5) || (StopBit == NvDdkUartStopBit_2))
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, STOP, ENABLE, LcrValue);
    else
        return NvError_NotSupported;
    pUartHwRegs->LcrRegister = LcrValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);
    return NvSuccess;
}

/**
 * Start/stop sending the break control signal.
 */
void NvDdkPrivUartHwSetBreakControlSignal(
    UartHwRegisters *pUartHwRegs,
    NvBool IsStart)
{
    NvU32 LcrValue;
    // Get the lcr register value and configured the break send control bit.
    LcrValue = pUartHwRegs->LcrRegister;
    if (IsStart)
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, SET_B, BREAK, LcrValue);
    else
        LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, SET_B, NO_BREAK, LcrValue);
    pUartHwRegs->LcrRegister = LcrValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);
}

/**
 * Reset the uart fifo register.
 */
void
NvDdkPrivUartHwResetFifo(
    UartHwRegisters *pUartHwRegs,
    UartDataDirection FifoType)
{
    NvU32 FcrValue;

    // Get the Fcr register value.
    FcrValue = pUartHwRegs->FcrRegister;

    // Get the fifo type whether receive and transmit fifo.
    // Reset the rx or tx fifo.
    if (FifoType & UartDataDirection_Receive)
        FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_CLR, CLEAR, FcrValue);

    if (FifoType & UartDataDirection_Transmit)
        FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_CLR, CLEAR, FcrValue);

    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IIR_FCR, FcrValue);
}

/**
 * Set the fifo trigger level of uart fifo.
 */
void
NvDdkPrivUartHwSetFifoTriggerLevel(
    UartHwRegisters *pUartHwRegs,
    UartDataDirection FifoType,
    NvU32 TriggerLevel)
{
    NvU32 FcrValue;

    // Get the fcr register value and configured for the receive and transmit
    // fifo trigger level.
    FcrValue = pUartHwRegs->FcrRegister;
    FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, FCR_EN_FIFO, ENABLE, FcrValue);

    // If it is receive fifo then configure the rx trigger value.
    if (FifoType & UartDataDirection_Receive)
    {
        switch (TriggerLevel)
        {
            case 1:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_TRIG,
                                               FIFO_COUNT_GREATER_1, FcrValue);
                break;

            case 4:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_TRIG,
                                               FIFO_COUNT_GREATER_4, FcrValue);
                break;

            case 8:
                // Fix me ar issue
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_TRIG,
                                                FIFO_COUNT_GREATER_8, FcrValue);
                break;

            default:
                // Fix me ar issue
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, RX_TRIG,
                                                FIFO_COUNT_GREATER_12, FcrValue);
                break;
        }
    }
    if (FifoType & UartDataDirection_Transmit)
    {
        // It is for transmit fifo, configure for tx fifo trigger level.
        switch (TriggerLevel)
        {
            case 1:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
                                        FIFO_COUNT_GREATER_1, FcrValue);
                pUartHwRegs->TransmitTrigLevel = 1;
                break;

            case 4:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
                                        FIFO_COUNT_GREATER_4, FcrValue);
                pUartHwRegs->TransmitTrigLevel = 4;
                break;

            case 8:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
                                        FIFO_COUNT_GREATER_8, FcrValue);
                pUartHwRegs->TransmitTrigLevel = 8;
                break;

            default:
                FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, TX_TRIG,
                                        FIFO_COUNT_GREATER_12, FcrValue);

                pUartHwRegs->TransmitTrigLevel = 12;
                break;
        }
    }

    // If no error then write into the uart register. Also update the soft copy
    // of fcr register.
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IIR_FCR, FcrValue);
    pUartHwRegs->FcrRegister = FcrValue;
}

/**
 * Set the dma mode operation of uart
 */
void NvDdkPrivUartHwSetDmaMode(UartHwRegisters *pUartHwRegs, NvBool IsEnable)
{
    NvU32 FcrValue;

    // Get the fcr register value and enable/disable the dma bit and write into
    // the register.
    FcrValue = pUartHwRegs->FcrRegister;
    if (IsEnable)
        FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, DMA, CHANGE, FcrValue);
    else
        FcrValue = NV_FLD_SET_DRF_DEF(UART, IIR_FCR, DMA, NO_CHANGE, FcrValue);

    pUartHwRegs->IsDmaMode = IsEnable;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IIR_FCR, FcrValue);
    pUartHwRegs->FcrRegister = FcrValue;
}

/**
 * Write into the transmit fifo register.
 * Returns the number of bytes written.
 */
NvU32
NvDdkPrivUartHwWriteInTransmitFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pTxBuff,
    NvU32 BytesRequested)
{
    NvU32 BytesToWrite;
    NvU32 TransmitChar;
    NvU32 DataWritten = 0;

    // This function is called once the trigger level interrupt is reach.
    // Write the data same as the trigger level without checking the fifo status.
    BytesToWrite = NV_MIN(BytesRequested, pUartHwRegs->TransmitTrigLevel);

    while (BytesToWrite)
    {
        TransmitChar = (NvU32)(*pTxBuff);
        UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, THR_DLAB_0, TransmitChar);
        pTxBuff++;
        BytesToWrite--;
        DataWritten++;
    }
    return DataWritten;
}

/**
 * Write into the transmit fifo register.
 * Returns the number of bytes written.
 */
NvU32
NvDdkPrivUartHwPollingWriteInTransmitFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pTxBuff,
    NvU32 BytesRequested)
{
    NvU32 BytesToWrite = BytesRequested;
    NvU32 TransmitChar;
    NvU32 LsrValue;

    while (BytesToWrite) 
    {
        LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);
        if (!(LsrValue & (NV_DRF_DEF(UART, LSR, TMTY, EMPTY))))
            break;
        TransmitChar = (NvU32)(*pTxBuff);
        UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, THR_DLAB_0, TransmitChar);
        pTxBuff++;
        BytesToWrite--;
    } 
    return (BytesRequested - BytesToWrite);
}

/**
 * Check whether the Transmit fifo is empty or not.
 * Returns TRUE if the Tx fifo is empty otherwise returns FALSE.
 */
NvBool NvDdkPrivUartHwIsTransmitFifoEmpty(UartHwRegisters *pUartHwRegs)
{
    NvU32 LsrValue;
    LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);
    if ((LsrValue & TX_EMPTY_STATUS) == TX_EMPTY_STATUS)
        return NV_TRUE;
    else
        return NV_FALSE;
}

NvBool NvDdkPrivUartHwIsBreakSignalDetected(UartHwRegisters *pUartHwRegs)
{
    NvBool IsBreakDetected = NV_FALSE;
    NvU32 LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);
    if (LsrValue & NV_DRF_DEF(UART, LSR, BRK, BREAK))
    {
        IsBreakDetected =  NV_TRUE;

        // Again read the LSR and if there is fifo error with data then reset 
        // here
        LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);
        if (LsrValue & NV_DRF_DEF(UART, LSR, FIFOE, ERR))
        {
            if (LsrValue & NV_DRF_DEF(UART, LSR, RDR, DATA_IN_FIFO))
                return IsBreakDetected;

            // Fifo error without rx data
            NvDdkPrivUartHwResetFifo(pUartHwRegs, UartDataDirection_Receive);
        }
    }
    
    return IsBreakDetected;
}

/**
 * Read the data from the receive fifo.
 */
NvBool NvDdkPrivUartHwIsDataAvailableInFifo(UartHwRegisters *pUartHwRegs)
{
    NvU32 LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);
    if (LsrValue & NV_DRF_DEF(UART, LSR, RDR, DATA_IN_FIFO))
        return NV_TRUE;
    return NV_FALSE;
}

/**
 * Read the data from the receive fifo.
 */
NvError
NvDdkPrivUartHwReadFromReceiveFifo(
    UartHwRegisters *pUartHwRegs,
    NvU8 *pRxBuff,
    NvU32 MaxByteRequested,
    NvU32 *pBytesRead)
{
    NvError Error = NvSuccess;
    NvU32 BytesToRead;
    NvU32 LsrValue = 0;
    NvU8 ReadChar;
    NvU32 DataRead = 0;

    // Get how many bytes to be read.
    BytesToRead = MaxByteRequested;

    // Read till bytes required to be read is there.
    while (BytesToRead)
    {
        // Get the Line status register and check for the error.
        LsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, LSR);

        // Check if any type of error occurred.
        if (LsrValue & IS_POSSIBLE_RX_ERROR)
        {
            if (LsrValue & NV_DRF_DEF(UART, LSR, OVRF, OVERRUN_ERROR))
            {
                // Overrun error
                Error = NvError_UartOverrun;
            }
            else if (LsrValue & NV_DRF_DEF(UART, LSR, PERR, PARITY_ERR))
            {
                // Parity error
                Error = NvError_UartParity;
            }
            else if (LsrValue & NV_DRF_DEF(UART, LSR, FERR, FRAME_ERR))
            {
                // Framing error
                Error = NvError_UartFraming;
            }
            else if (LsrValue & NV_DRF_DEF(UART, LSR, FIFOE, ERR))
            {
                // Fifo error
                Error = NvSuccess;
                NvDdkPrivUartHwResetFifo(pUartHwRegs, UartDataDirection_Receive);
                *pBytesRead = 0;
            }
            else if (LsrValue & NV_DRF_DEF(UART, LSR, BRK, BREAK))
            {
                // Break signal received.
                Error = NvError_UartBreakReceived;
            }
            break;
        }

        // No error in fifo, check if data is available and if then read from
        // receive fifo.
        if (LsrValue & IS_DATA_IN_FIFO)
        {
            ReadChar = (NvU8)((UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, THR_DLAB_0)) & 0xFF);
            *pRxBuff++  = ReadChar;
            BytesToRead--;
            DataRead++;
        }
        else
        {
            // Fifo is empty so exit from loop.
            break;
        }
    }

    *pBytesRead = DataRead;
    return Error;
}

void NvDdkPrivUartHwClearAllInt(UartHwRegisters *pUartHwRegs)
{
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, 0);
    pUartHwRegs->IerRegister = 0;
}

void NvDdkPrivUartHwSetReceiveInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable,
    NvBool IsRxUsingApbDma)
{
    NvU32 IerValue;

    // Get the IER register value.
    IerValue = pUartHwRegs->IerRegister;

    // Enable/disable the interrupt bit for the receive fifo ready and
    // receive line status error.
    if (IsEnable)
    {
        if (IsRxUsingApbDma)
        {
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RXS, ENABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RX_TIMEOUT, ENABLE, IerValue);
            if (pUartHwRegs->IsEndOfDataIntSupport)
                IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_EORD, ENABLE, IerValue);
        }
        else
        {
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RHR, ENABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RXS, ENABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RX_TIMEOUT, ENABLE, IerValue);
        }
    }
    else
    {
        if (IsRxUsingApbDma)
        {
            // Sw workaround to disable the rx timeout interrupt.
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RHR, ENABLE, IerValue);
            UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, IerValue);

            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RHR, DISABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RXS, DISABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RX_TIMEOUT, DISABLE, IerValue);
            if (pUartHwRegs->IsEndOfDataIntSupport)
                IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_EORD, DISABLE, IerValue);
        }
        else
        {
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RHR, DISABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RXS, DISABLE, IerValue);
            IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_RX_TIMEOUT, DISABLE, IerValue);
        }
    }
    pUartHwRegs->IerRegister = IerValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, IerValue);
}

void NvDdkPrivUartHwSetTransmitInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable)
{
    NvU32 IerValue;
    // Get the IER register value.
    IerValue = pUartHwRegs->IerRegister;

    // Enable/disable the transmit fifo empty
    if (IsEnable)
        IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_THR, ENABLE, IerValue);
    else
        IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_THR, DISABLE, IerValue);
    pUartHwRegs->IerRegister = IerValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, IerValue);
}

void NvDdkPrivUartHwSetModemControlInterrupt(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable)
{
    NvU32 IerValue;

    // Get the IER register value.
    IerValue = pUartHwRegs->IerRegister;

    // Enable/disable the modem control signal.
    if (IsEnable)
        IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_MSI, ENABLE, IerValue);
    else
        IerValue = NV_FLD_SET_DRF_DEF(UART, IER_DLAB_0, IE_MSI, DISABLE, IerValue);
    pUartHwRegs->IerRegister = IerValue;
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, IerValue);
}

/**
 * Get the interrupt reason.
 */
UartHwInterruptSource
NvDdkPrivUartHwGetInterruptReason(
    UartHwRegisters *pUartHwRegs)
{
    NvU32 IirValue;
    NvU32 P0;
    NvU32 P1;
    NvU32 P2;
    NvU32 InterruptValue = 0;
    NvU32 InterruptSource[8] =
            {
                UartHwInterruptSource_ModemControl, // Modem signal change interrupt
                UartHwInterruptSource_Transmit,     // Transmit fifo trigger level interrupt
                UartHwInterruptSource_Receive,      // Rx fifo data ready interrupt
                UartHwInterruptSource_ReceiveError, // Line status interrupt
                UartHwInterruptSource_EndOfData,    // End of data interrupt
                UartHwInterruptSource_None,         // Unused interrupt
                UartHwInterruptSource_RxTimeout,    // Rx timeout interrupt
                UartHwInterruptSource_None          // Unused interrupt
            };
    
    // Read the Iir register value.
    IirValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, IIR_FCR);

    // Check whether all the interrupt is served or not. If there is no
    // pending interrupt then return none.
    if (IirValue & NV_DRF_DEF(UART, IIR_FCR, IS_STA, NO_INTR_PEND))
        return UartHwInterruptSource_None;

    // Valid interrupt is there, identify the interrupt.
    P0 = NV_DRF_VAL(UART, IIR_FCR, IS_PRI0, IirValue);
    P1 = NV_DRF_VAL(UART, IIR_FCR, IS_PRI1, IirValue);
    P2 = NV_DRF_VAL(UART, IIR_FCR, IS_PRI2, IirValue);
    InterruptValue = ((P2 << 2) | (P1 <<1) | (P0)) & 0x7;
    return InterruptSource[InterruptValue];
}

/**
 * Enable/disable the Irda coding.
 */
void
NvDdkPrivUartHwSetIrdaCoding(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable)
{
    NvU32 IrdaCsrValue;

    // Read data from the irda csr register.
    IrdaCsrValue = pUartHwRegs->IrdaCsrRegister;

    // Enable/disable the sir modulation bit.
    if (IsEnable)
        IrdaCsrValue = NV_FLD_SET_DRF_DEF(UART, IRDA_CSR, SIR_A, ENABLE, IrdaCsrValue);
    else
        IrdaCsrValue = NV_FLD_SET_DRF_DEF(UART, IRDA_CSR, SIR_A, DISABLE, IrdaCsrValue);

    pUartHwRegs->IrdaCsrRegister = IrdaCsrValue;
    // Write into the irda csr register.
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IRDA_CSR, IrdaCsrValue);
}

/**
 * Set the output modem signal to active or inactive state.
 */
void
NvDdkPrivUartHwSetModemSignal(
    UartHwRegisters *pUartHwRegs,
    NvDdkUartSignalName SignalName,
    NvBool IsActive)
{
    NvU32 McrValue= pUartHwRegs->McrRegister;

    if (SignalName & NvDdkUartSignalName_Rts)
    {
        if (IsActive != pUartHwRegs->IsRtsActive)
        {
            if (IsActive) 
                McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_LOW, McrValue);
            else
                McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, RTS, FORCE_RTS_HI, McrValue);
            pUartHwRegs->IsRtsActive = IsActive;
        }
    }

    if (SignalName & NvDdkUartSignalName_Dtr)
    {
        if (IsActive != pUartHwRegs->IsDtrActive)
        {
            if (IsActive)
                McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, DTR, FORCE_DTR_LOW, McrValue);
            else
                McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, DTR, FORCE_DTR_HI, McrValue);
            pUartHwRegs->IsDtrActive = IsActive;
        }
    }

    // Write the updated value into the register.
    if (pUartHwRegs->McrRegister != McrValue)
    {
        UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, MCR, McrValue);
        pUartHwRegs->McrRegister = McrValue;
    }    
    return;
}

/**
 * Read the modem signal level and update the state of the signal in the uart hw
 * register structure.
 */
void NvDdkPrivUartHwUpdateModemSignal(UartHwRegisters *pUartHwRegs)
{
    NvU32 MsrValue;

    // Read the modem status register.
    MsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, MSR);

    // If change in CTS then update the CTS value.
    if (MsrValue & (NV_DRF_DEF(UART, MSR, CTS, ENABLE)))
        pUartHwRegs->ModemSignalSatus |= NvDdkUartSignalName_Cts;
    else
        pUartHwRegs->ModemSignalSatus &= ~NvDdkUartSignalName_Cts;
    
    // If change in DSR then update the DSR value.
    if (MsrValue & (NV_DRF_DEF(UART, MSR, DSR, ENABLE)))
        pUartHwRegs->ModemSignalSatus |= NvDdkUartSignalName_Dsr;
    else
        pUartHwRegs->ModemSignalSatus &= ~NvDdkUartSignalName_Dsr;

    // If change in RI then update the RI value.
    if (MsrValue & (NV_DRF_DEF(UART, MSR, RI, ENABLE)))
        pUartHwRegs->ModemSignalSatus |= NvDdkUartSignalName_Ri;
    else
        pUartHwRegs->ModemSignalSatus &= ~NvDdkUartSignalName_Ri;

    // If change in CD then update the CD value.
    if (MsrValue & (NV_DRF_DEF(UART, MSR, CD, ENABLE)))
        pUartHwRegs->ModemSignalSatus |= NvDdkUartSignalName_Cd;
    else
        pUartHwRegs->ModemSignalSatus &= ~NvDdkUartSignalName_Cd;
}

// NV_TRUE Active: Uart can send the data
// NV_FALSE InActive: Uart can not send the data as other terminal have not 
// activated its RTS line
NvBool NvDdkPrivUartHwIsCtsActive(UartHwRegisters *pUartHwRegs)
{
    NvU32 MsrValue = UART_REG_READ32(pUartHwRegs->pRegsVirtBaseAdd, MSR);
    if (MsrValue & (NV_DRF_DEF(UART, MSR, CTS, ENABLE)))
        return NV_TRUE;
    else
        return NV_FALSE;
}
/** 
* Enable/disable the hw flow control for transmitting the signal. If it is 
* enabled then it will only transmit the data if CTS line is Low. If the CTS
* line is high then it will not transmit the data. The controller reads the 
* inverted state of the signal state from pin.
*/ 
void
NvDdkPrivUartHwSetHWFlowControl(
    UartHwRegisters *pUartHwRegs,
    NvBool IsEnable)
{
    NvU32 McrValue;

    // Read the modem control register and configured for the CTS hw flow
    // bit for the enabling/disabling the hw flow for transmit.
    McrValue = pUartHwRegs->McrRegister;

    // Configired the CTS Hw flow bit.
    if (IsEnable)
    {
        McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, CTS_EN, ENABLE, McrValue);

        if (pUartHwRegs->IsRtsHwFlowSupported)
            McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, RTS_EN, ENABLE, McrValue);
    }
    else
    {
        McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, CTS_EN, DISABLE, McrValue);
        if (pUartHwRegs->IsRtsHwFlowSupported)
            McrValue = NV_FLD_SET_DRF_DEF(UART, MCR, RTS_EN, DISABLE, McrValue);
    }

    pUartHwRegs->McrRegister = McrValue;

    // Write into the register.
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, MCR, McrValue);
}

void NvDdkPrivUartHwReconfigureRegisters(UartHwRegisters *pUartHwRegs)
{
    NvU32 LcrValue;

    // Fcr Register
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IIR_FCR, pUartHwRegs->FcrRegister);

    // Lcr, DLL and DLAB registers
    // Enable the LCR-DLAB.
    LcrValue = pUartHwRegs->LcrRegister;
    LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, ENABLE, LcrValue);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);

    // Program the lower and upper bytes of the baud rate constant.
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, THR_DLAB_0, pUartHwRegs->DllRegister);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, pUartHwRegs->DlmRegister);

    // Disable the DLAB.
    LcrValue = NV_FLD_SET_DRF_DEF(UART, LCR, DLAB, DISABLE, LcrValue);
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, LCR, LcrValue);

    // Mcr register
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, MCR, pUartHwRegs->McrRegister);

    // Irda CSR
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IRDA_CSR, pUartHwRegs->IrdaCsrRegister);

    // IER register
    UART_REG_WRITE32(pUartHwRegs->pRegsVirtBaseAdd, IER_DLAB_0, pUartHwRegs->IerRegister);
}

