/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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

#include "nvcommon.h"
#include "nvrm_pinmux.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "ap20/arapb_misc.h"
#include "ap20/arclk_rst.h"
#include "ap15/ap15rm_pinmux_utils.h"
#include "nvrm_clocks.h"
#include "nvodm_query_pinmux.h"

//  FIXME:  None of the modules have reset configurations, yet.  This should
//  be fixed.
static const NvU32 g_Ap20Mux_Uart1[] = {
    UNCONFIG(C,IRRX,UARTA,UARTB),UNCONFIG(C,IRTX,UARTA,UARTB),
    UNCONFIG(A,UAA,UARTA,MIPI_HS),UNCONFIG(A,UAB,UARTA,MIPI_HS),
    UNCONFIG(D,SDB,UARTA,PWM),UNCONFIG(D,SDD,UARTA,PWM),
    CONFIGEND(),
    CONFIG(B,A,UAA,UARTA),CONFIG(B,A,UAB,UARTA),CONFIGEND(),
    CONFIG(A,D,GPU,UARTA),CONFIGEND(),
    CONFIG(A,C,IRRX,UARTA),CONFIG(A,C,IRTX,UARTA),CONFIG(B,A,UAD,UARTA),CONFIGEND(),
    CONFIG(A,C,IRRX,UARTA),CONFIG(A,C,IRTX,UARTA),CONFIGEND(),
    CONFIG(B,D,SDD,UARTA),CONFIG(D,D,SDB,UARTA),CONFIGEND(),
    // added as additonal config for 4b mode.
    CONFIG(B,A,UAA,UARTA),CONFIGEND(),
    CONFIG(A,A,SDIO1,UARTA),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Uart2[] = {
    /* disown IRDA and SPDIF chosen .*/
    UNCONFIG(A,UAD,IRDA,SPDIF),CONFIGEND(),
    CONFIG(A,C,IRRX,UARTB),CONFIG(A,C,IRTX,UARTB),CONFIG(B,A,UAD,IRDA),CONFIGEND(),
    CONFIG(B,A,UAD,IRDA),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Uart3[] = {
    UNCONFIG(B,UCA,UARTC,RSVD2),UNCONFIG(B,UCB,UARTC,RSVD4),CONFIGEND(),
    CONFIG(B,B,UCA,UARTC),CONFIG(B,B,UCB,UARTC),CONFIGEND(),
    CONFIG(B,B,UCA,UARTC),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Uart4[] = {
    UNCONFIG(B,GMC,UARTD,SPI4),CONFIGEND(),
    CONFIG(D,A,UDA,UARTD),CONFIGEND(),
    CONFIG(A,B,GMC,UARTD),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Uart5[] = {
    UNCONFIG(B,GMA,UARTE,SPI3),CONFIGEND(),
    CONFIG(A,A,SDIO1,UARTE),CONFIGEND(),
    CONFIG(A,B,GMA,UARTE),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32* g_Ap20MuxUart[] = {
    &g_Ap20Mux_Uart1[0],
    &g_Ap20Mux_Uart2[0],
    &g_Ap20Mux_Uart3[0],
    &g_Ap20Mux_Uart4[0],
    &g_Ap20Mux_Uart5[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Spi1[] = {
    /*  Disown UDA,SPIA,SPIB and SPIC  RSVD and GMI chosen*/
    UNCONFIG(A, UDA, SPI1, RSVD),UNCONFIG(D, SPIA, SPI1, GMI),
    UNCONFIG(D, SPIB, SPI1, GMI),UNCONFIG(D, SPIC, SPI1, GMI),
    CONFIGEND(),
    CONFIG(D,A,UDA,SPI1),CONFIGEND(),
    CONFIG(A,B,DTE,SPI1),CONFIG(A,B,DTB,SPI1),CONFIGEND(),
    CONFIG(B,D,SPIC,SPI1),CONFIG(B,D,SPIB,SPI1),CONFIG(B,D,SPIA,SPI1),CONFIGEND(),
    CONFIG(B,D,SPIE,SPI1),CONFIG(B,D,SPIF,SPI1),CONFIG(B,D,SPID,SPI1),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Spi2[] = {
    //  Reset config - abandon UAB, pads.  MIPI_HS and GMI chosen
    UNCONFIG(A,UAB,SPI2,MIPI_HS), UNCONFIG(D,SPID,SPI2,GMI),
    UNCONFIG(D,SPIE,SPI2,GMI),
    CONFIGEND(),
    // config 1
    CONFIG(B,A,UAB,SPI2),CONFIGEND(),
    // config 2
    CONFIG(B,D,SPIC,SPI2),CONFIG(B,D,SPIB,SPI2),CONFIG(B,D,SPIA,SPI2),
    CONFIG(B,D,SPIG,SPI2),CONFIG(B,D,SPIH,SPI2),CONFIGEND(),
    // config 3
    CONFIG(B,D,SPIE,SPI2_ALT),CONFIG(B,D,SPIF,SPI2),CONFIG(B,D,SPID,SPI2_ALT),
    CONFIG(B,D,SPIG,SPI2_ALT),CONFIG(B,D,SPIH,SPI2_ALT),CONFIGEND(),
    // config 4
    CONFIG(B,D,SPIC,SPI2),CONFIG(B,D,SPIB,SPI2),CONFIG(B,D,SPIA,SPI2),
    CONFIGEND(),
    // config 5
    CONFIG(B,D,SPIE,SPI2_ALT),CONFIG(B,D,SPIF,SPI2),CONFIG(B,D,SPID,SPI2_ALT),
    CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Spi3[] = {
    //  Reset config - abandon UAA,SPIF,SPIG,SPIH pads.  MIPI_HS and SPI2_ALT chosen
    UNCONFIG(A,UAA,SPI3,UARTA), UNCONFIG(D,SPIF,SPI3,RSVD),
    UNCONFIG(D,SPIG,SPI3,SPI2_ALT),UNCONFIG(D,SPIH,SPI3,SPI2_ALT),
    CONFIGEND(),
    // Config 1
    CONFIG(B,A,UAA,SPI3),CONFIGEND(),
    //  Config 2.
    CONFIG(C,E,LSC1,SPI3),CONFIG(D,E,LPW2,SPI3),CONFIG(D,E,LPW0,SPI3),
    CONFIG(C,E,LM0,SPI3),CONFIGEND(),
    //  config 3
    CONFIG(C,E,LSCK,SPI3),CONFIG(D,E,LSDI,SPI3),CONFIG(D,E,LSDA,SPI3),
    CONFIG(C,E,LCSN,SPI3),CONFIGEND(),
    // config 4
    CONFIG(A,B,GMA,SPI3),CONFIGEND(),
    // config 5
    CONFIG(B,D,SPIC,SPI3),CONFIG(B,D,SPIB,SPI3),CONFIG(B,D,SPIA,SPI3), CONFIGEND(),
    // config 6
    CONFIG(B,D,SDC,SPI3),CONFIG(B,D,SDD,SPI3), CONFIGEND(),

    // config 7
    /* -spif,spig,and spih are added as config 7 on mux: 0
     * -spia of SPI2_MOSI as spi3_dout on mux: 2 under config 7.
    */
    CONFIG(B,D,SPIA,SPI3),CONFIG(B,D,SPIF,SPI3),
    CONFIG(B,D,SPIG,SPI3),CONFIG(B,D,SPIG,SPI3), CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Spi4[] = {
    CONFIGEND(),
    //  config 1
    CONFIG(B,A,UAD,SPI4),CONFIG(A,C,IRRX,SPI4),CONFIG(A,C,IRTX,SPI4),CONFIGEND(),
    //  config 2
    CONFIG(A,B,GMC,SPI4),CONFIGEND(),
    //  config 3
    CONFIG(B,B,SLXC,SPI4),CONFIG(B,B,SLXK,SPI4),CONFIG(B,B,SLXA,SPI4),
    CONFIG(B,B,SLXD,SPI4),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxSpi[] = {
    &g_Ap20Mux_Spi1[0],
    &g_Ap20Mux_Spi2[0],
    &g_Ap20Mux_Spi3[0],
    &g_Ap20Mux_Spi4[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Sflash[] = {
    CONFIGEND(),
    CONFIG(B,C,GMD,SFLASH),CONFIG(A,B,GMC,SFLASH),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxSflash[] = {
    &g_Ap20Mux_Sflash[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Twc[] = {
    CONFIGEND(),
    CONFIG(A,C,DAP2,TWC),CONFIGEND(),
    CONFIG(B,D,SDC,TWC),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxTwc[] = {
    &g_Ap20Mux_Twc[0],
    NULL,
};

static const NvU32 g_Ap20Mux_I2c1[] = {
    UNCONFIG(A,RM,I2C,RSVD1),CONFIGEND(),
    // config 1
    CONFIG(A,A,RM,I2C),CONFIGEND(),
    // config 2
    CONFIG(B,D,SPDI,I2C),CONFIG(B,D,SPDO,I2C),CONFIGEND(),
    // config 3
    CONFIG(B,D,SPIG,I2C),CONFIG(B,D,SPIH,I2C),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_I2c2[] = {
    // reset & multiplex config
    UNCONFIG(G,PTA,I2C2,RSVD3),UNCONFIG(C,DDC,I2C2,RSVD1),CONFIGEND(),
    // config 1
    CONFIG(B,C,DDC,I2C2),CONFIGEND(),
    // config 2
    CONFIG(A,G,PTA,I2C2),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_I2c3[] = {
    UNCONFIG(G,DTF,I2C3,RSVD2),CONFIGEND(),
    CONFIG(D,G,DTF,I2C3),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxI2c[] = {
    &g_Ap20Mux_I2c1[0],
    &g_Ap20Mux_I2c2[0],
    &g_Ap20Mux_I2c3[0],
    NULL,
};

static const NvU32 g_Ap20Mux_I2cPmu[] = {
    UNCONFIG(C,I2CP,I2C,RSVD2),CONFIGEND(),
    CONFIG(A,C,I2CP,I2C),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxI2cPmu[] = {
    &g_Ap20Mux_I2cPmu[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Ulpi[] = {
    CONFIGEND(),
    CONFIG(B,A,UAA,ULPI),CONFIG(B,A,UAB,ULPI),CONFIG(D,A,UDA,ULPI),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxUlpi[] = {
    &g_Ap20Mux_Ulpi[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Sdio1[] = {
    UNCONFIG(A,SDIO1,SDIO1,RSVD1),CONFIGEND(),
    CONFIG(A,A,SDIO1,SDIO1),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Sdio2[] = {
    CONFIGEND(),
    // config 1
    CONFIG(D,G,KBCD,SDIO2),CONFIG(A,C,KBCB,SDIO2),CONFIGEND(),
    // config 2
    CONFIG(D,G,KBCD,SDIO2),CONFIG(A,C,KBCB,SDIO2),CONFIG(A,C,KBCA,SDIO2),CONFIGEND(),
    // config 3
    CONFIG(A,C,DAP1,SDIO2),CONFIG(B,D,SPDI,SDIO2),CONFIG(B,D,SPDO,SDIO2),CONFIGEND(),
    // config 4
    CONFIG(A,B,DTA,SDIO2),CONFIG(A,B,DTD,SDIO2),CONFIGEND(),
    // config 5
    CONFIG(A,B,DTA,SDIO2),CONFIG(A,B,DTD,SDIO2),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Sdio3[] = {
    CONFIGEND(),
    // config 1
    CONFIG(B,D,SDD,SDIO3), CONFIG(B,D,SDC,SDIO3), CONFIG(D,D,SDB,SDIO3), CONFIG(B,B,SLXA,SDIO3),
    CONFIG(B,B,SLXK,SDIO3),CONFIG(B,B,SLXC,SDIO3),CONFIG(B,B,SLXD,SDIO3),CONFIGEND(),
    // config 2
    CONFIG(B,D,SDD,SDIO3),CONFIG(B,D,SDC,SDIO3), CONFIG(D,D,SDB,SDIO3), CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Sdio4[] = {
    CONFIGEND(),
    // config 1
    CONFIG(A,A,ATC,SDIO4),CONFIG(A,A,ATD,SDIO4),CONFIGEND(),
    // config 2
    BRANCH(3),CONFIG(B,D,GME,SDIO4),CONFIGEND(),
    // config 3
    CONFIG(A,A,ATB,SDIO4),CONFIG(A,B,GMA,SDIO4),CONFIGEND(),
    MODULEDONE(),
    SUBROUTINESDONE(),
};

static const NvU32 *g_Ap20MuxSdio[] = {
    &g_Ap20Mux_Sdio1[0],
    &g_Ap20Mux_Sdio2[0],
    &g_Ap20Mux_Sdio3[0],
    &g_Ap20Mux_Sdio4[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Spdif[] = {
    UNCONFIG(D,SPDO,SPDIF,RSVD),UNCONFIG(D,SPDI,SPDIF,RSVD),
    UNCONFIG(B,SLXD,SPDIF,SPI4),UNCONFIG(B,SLXC,SPDIF,SPI4),
    CONFIGEND(),
    // config 1
    CONFIG(B,D,SPDO,SPDIF),CONFIG(B,D,SPDI,SPDIF),CONFIGEND(),
    // config 2
    CONFIG(B,B,SLXD,SPDIF),CONFIG(B,B,SLXC,SPDIF),CONFIGEND(),
    // config 3
    CONFIG(B,A,UAD,SPDIF),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxSpdif[] = {
    &g_Ap20Mux_Spdif[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Hsi[] = {
    CONFIGEND(),
    CONFIG(B,A,UAA,MIPI_HS),CONFIG(B,A,UAB,MIPI_HS),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxHsi[] = {
    &g_Ap20Mux_Hsi[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Hdmi[] = {
    // HDINT resets to HDMI, so move it to a reserved pin RSVD2
    UNCONFIG(B,HDINT,HDMI,RSVD2),CONFIGEND(),
    CONFIG(C,B,HDINT,HDMI),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxHdmi[] = {
    &g_Ap20Mux_Hdmi[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Pwm[] = {
    UNCONFIG(D,GPU,PWM,RSVD4),UNCONFIG(D,SDC,PWM,TWC),
    CONFIGEND(),
    // config 1
    CONFIG(A,D,GPU,PWM),CONFIGEND(),
    // config 2
    CONFIG(B,B,UCB,PWM),CONFIG(B,D,SDD,PWM),CONFIGEND(),
    // config 3
    CONFIG(B,B,UCB,PWM),CONFIGEND(),
    // config 4
    CONFIG(B,D,SDD,PWM),CONFIGEND(),
    // config 5
    CONFIG(B,D,SDC,PWM),CONFIG(B,D,SDD,PWM),CONFIGEND(),
    // config 6
    CONFIG(B,D,SDC,PWM),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxPwm[] = {
    &g_Ap20Mux_Pwm[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Ata[] = {
    CONFIGEND(),
    CONFIG(A,A,ATA,IDE),CONFIG(A,A,ATB,IDE),CONFIG(A,A,ATC,IDE),
    CONFIG(A,A,ATD,IDE),CONFIG(B,A,ATE,IDE),CONFIG(B,C,GMB,IDE),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxAta[] = {
    &g_Ap20Mux_Ata[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Nand[] = {

    UNCONFIG(A,ATA,NAND,IDE),  UNCONFIG(A,ATB,NAND,IDE), UNCONFIG(A,ATC,NAND,IDE),
    UNCONFIG(A,ATD,NAND,IDE), UNCONFIG(A,ATE,NAND,IDE),
    CONFIGEND(),
    // config 1
    CONFIG(A,A,ATA,NAND),CONFIG(A,A,ATB,NAND),CONFIG(A,A,ATC,NAND),
    CONFIG(A,A,ATD,NAND),CONFIG(B,A,ATE,NAND),CONFIG(B,C,GMB,IDE), CONFIGEND(),
    //.config 2
    CONFIG(A,A,ATA,NAND),CONFIG(A,A,ATB,NAND),CONFIG(A,A,ATC,NAND),CONFIGEND(),
    // config 3
    CONFIG(A,C,KBCA,NAND),CONFIG(A,C,KBCB,NAND),CONFIG(B,C,KBCC,NAND),
    CONFIG(D,G,KBCD,NAND),CONFIG(A,A,KBCE,NAND),CONFIG(A,A,KBCF,NAND), CONFIGEND(),
    // config 4
    CONFIG(A,A,ATC,NAND),CONFIGEND(),

    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxNand[] = {
    &g_Ap20Mux_Nand[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Dap1[] = {
    CONFIGEND(),
    CONFIG(A,C,DAP1,DAP1),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Dap2[] = {
    CONFIGEND(),
    CONFIG(A,C,DAP2,DAP2),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Dap3[] = {
    CONFIGEND(),
    CONFIG(A,C,DAP3,DAP3),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Dap4[] = {
    CONFIGEND(),
    CONFIG(A,C,DAP4,DAP4),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_Dap5[] = {
    CONFIGEND(),
    CONFIG(B,D,GME,DAP5),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxDap[] = {
    &g_Ap20Mux_Dap1[0],
    &g_Ap20Mux_Dap2[0],
    &g_Ap20Mux_Dap3[0],
    &g_Ap20Mux_Dap4[0],
    &g_Ap20Mux_Dap5[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Kbd[] = {
    CONFIGEND(),
    // config 1
    BRANCH(2),CONFIG(A,C,KBCB,KBC),CONFIGEND(),
    // config 2
    BRANCH(3),CONFIG(D,G,KBCD,KBC),CONFIGEND(),
    // config 3
    BRANCH(4),CONFIG(A,A,KBCF,KBC),CONFIGEND(),
    // config 4
    CONFIG(A,C,KBCA,KBC),CONFIG(B,C,KBCC,KBC),CONFIG(A,A,KBCE,KBC),CONFIGEND(),
    MODULEDONE(),
    SUBROUTINESDONE(),
};

static const NvU32 *g_Ap20MuxKbd[] = {
    &g_Ap20Mux_Kbd[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Hdcp[] = {
    CONFIGEND(),
    // config 1
    CONFIG(A,G,PTA,HDMI),CONFIGEND(),
    // config 2
    CONFIG(C,E,LSCK,HDMI),CONFIG(D,E,LSDA,HDMI),CONFIGEND(),
    // config 3
    CONFIG(D,E,LPW2,HDMI),CONFIG(D,E,LPW0,HDMI),CONFIGEND(),
    // config 4
    CONFIG(C,E,LSC1,HDMI),CONFIG(D,E,LPW0,HDMI),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxHdcp[] = {
    &g_Ap20Mux_Hdcp[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Snor[] = {
    CONFIGEND(),
    BRANCH(2),
    // config 1. separate 32b NOR
    CONFIG(A,C,IRRX,GMI),CONFIG(A,C,IRTX,GMI),CONFIG(B,B,UCA,GMI),
    CONFIG(B,B,UCB,GMI),CONFIG(A,D,GPU,GMI),CONFIGEND(),
    //  config 2.  muxed 32b NOR
    BRANCH(3),
    CONFIG(A,B,GMC,GMI),CONFIG(A,B,GMA,GMI), CONFIG(B,D,GME,GMI),
    CONFIG(A,C,DAP1,GMI), CONFIGEND(),
    // config 3.  muxed 16b NOR
    BRANCH(6),
    CONFIG(B,C,GMB,GMI),CONFIG(A,A,ATB,GMI),CONFIGEND(),
    // config 4.  separate 16b NOR
    BRANCH(3),
    CONFIG(A,C,IRRX,GMI),CONFIG(A,C,IRTX,GMI),CONFIG(B,B,UCA,GMI),
    CONFIG(B,B,UCB,GMI),CONFIG(A,D,GPU,GMI),CONFIGEND(),
    // config 5.  MuxOneNAND
    BRANCH(6),
    CONFIG(B,C,GMB,GMI_INT),CONFIG(A,B,GMC,SFLASH),CONFIGEND(),
    MODULEDONE(),
    // subroutine 1.  shared by 16b muxed NOR & muxOneNand
    CONFIG(A,C,DAP4,GMI),CONFIG(A,C,DAP2,GMI),CONFIG(B,D,SPIA,GMI),
    CONFIG(B,D,SPIB,GMI),CONFIG(B,D,SPIC,GMI),CONFIG(B,D,SPID,GMI),
    CONFIG(B,D,SPIE,GMI),
    CONFIG(A,A,ATA,GMI),CONFIG(A,A,ATC,GMI),CONFIG(A,A,ATD,GMI),
    CONFIG(B,A,ATE,GMI),CONFIG(B,C,GMD,GMI),CONFIGEND(),
    SUBROUTINESDONE(),
};

static const NvU32 *g_Ap20MuxSnor[] = {
    &g_Ap20Mux_Snor[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Mio[] = {
    CONFIGEND(),
    CONFIG(A,A,KBCF,MIO),CONFIG(D,G,KBCD,MIO),CONFIG(A,C,KBCB,MIO),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxMio[] = {
    &g_Ap20Mux_Mio[0],
    NULL,
};

static const NvU32 g_Ap20Mux_ExtClock1[] = {
    CONFIGEND(),
    CONFIG(A,C,CDEV1,PLLA_OUT),CONFIGEND(),
    CONFIG(A,C,CDEV1,OSC),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_ExtClock2[] = {
    CONFIGEND(),
    CONFIG(A,C,CDEV2,AHB_CLK),CONFIGEND(),
    CONFIG(A,C,CDEV2,OSC),CONFIGEND(),
    CONFIG(A,C,CDEV2,PLLP_OUT4),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 g_Ap20Mux_ExtClock3[] = {
    CONFIGEND(),
    CONFIG(A,C,CSUS,VI_SENSOR_CLK),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxExtClock[] = {
    &g_Ap20Mux_ExtClock1[0],
    &g_Ap20Mux_ExtClock2[0],
    &g_Ap20Mux_ExtClock3[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Vi[] = {
    CONFIGEND(),
    BRANCH(2),CONFIG(D,G,DTF,VI),CONFIGEND(),
    CONFIG(A,B,DTA,VI),CONFIG(A,B,DTB,VI),CONFIG(A,B,DTC,VI),
    CONFIG(A,B,DTD,VI),CONFIG(A,B,DTE,VI),CONFIGEND(),
    MODULEDONE(),
    SUBROUTINESDONE(),
};

static const NvU32 *g_Ap20MuxVi[] = {
    &g_Ap20Mux_Vi[0],
    NULL,
};

const NvU32 g_Ap20Mux_BacklightDisplay1Pwm0[] = {
    CONFIGEND(),
    // Config 1 LPW0 pad
    CONFIG(D,E,LPW0,DISPLAYA), CONFIGEND(),
    // Config 2 LPW2 pad
    CONFIG(D,E,LPW2,DISPLAYA), CONFIGEND(),
    // Config 3 LM0 pad
    CONFIG(C,E,LM0,DISPLAYA), CONFIGEND(),
    MODULEDONE(),
};

const NvU32 g_Ap20Mux_BacklightDisplay1Pwm1[] = {
    CONFIGEND(),
    // Config 1 LM1 pad
    CONFIG(C,E,LM1,DISPLAYA), CONFIGEND(),
    // Config 2 LDC pad
    CONFIG(C,E,LDC,DISPLAYA), CONFIGEND(),
    // Config 3 LPW1 pad
    CONFIG(D,E,LPW1,DISPLAYA), CONFIGEND(),
    MODULEDONE(),
};

const NvU32 g_Ap20Mux_BacklightDisplay2Pwm0[] = {
    CONFIGEND(),
    // Config 1 LPW0 pad
    CONFIG(D,E,LPW0,DISPLAYB), CONFIGEND(),
    // Config 2 LPW2 pad
    CONFIG(D,E,LPW2,DISPLAYB), CONFIGEND(),
    // Config 3 LM0 pad
    CONFIG(C,E,LM0,DISPLAYB), CONFIGEND(),
    MODULEDONE(),
};

const NvU32 g_Ap20Mux_BacklightDisplay2Pwm1[] = {
    CONFIGEND(),
    // Config 1 LM1 pad
    CONFIG(C,E,LM1,DISPLAYB), CONFIGEND(),
    // Config 2 LDC pad
    CONFIG(C,E,LDC,DISPLAYB), CONFIGEND(),
    // Config 3 LPW1 pad
    CONFIG(D,E,LPW1,DISPLAYB), CONFIGEND(),
    MODULEDONE(),
};

const NvU32* g_Ap20MuxBacklight[] = {
    &g_Ap20Mux_BacklightDisplay1Pwm0[0],
    &g_Ap20Mux_BacklightDisplay1Pwm1[0],
    &g_Ap20Mux_BacklightDisplay2Pwm0[0],
    &g_Ap20Mux_BacklightDisplay2Pwm1[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Display1[] = {
    CONFIGEND(),
    //  config 1, 24b RGB.  Pure superset of Config2 (18b RGB)
    BRANCH(2),
    CONFIG(C,G,LHP1,DISPLAYA),CONFIG(C,G,LHP2,DISPLAYA),CONFIG(C,G,LVP1,DISPLAYA),
    CONFIG(C,G,LHP0,DISPLAYA),CONFIG(D,G,LDI,DISPLAYA),CONFIG(D,G,LPP,DISPLAYA),
    CONFIGEND(),
    //  config 2, 18b RGB.
    BRANCH(7),
    CONFIG(C,E,LVS,DISPLAYA), CONFIG(D,E,LHS,DISPLAYA), CONFIG(D,E,LSPI,DISPLAYA),
    CONFIGEND(),
    // config 3, 8 & 9b CPU.
    CONFIG(C,G,LHP1,DISPLAYA), CONFIG(C,G,LHP2,DISPLAYA), CONFIG(C,G,LVP1,DISPLAYA),
    CONFIG(C,G,LHP0,DISPLAYA), CONFIG(D,G,LDI,DISPLAYA), CONFIG(D,G,LPP,DISPLAYA),
    CONFIG(D,E,LPW0,DISPLAYA), CONFIG(D,E,LPW1,DISPLAYA), CONFIG(D,E,LPW2,DISPLAYA),
    CONFIG(C,E,LSC1,DISPLAYA), CONFIG(C,E,LM1,DISPLAYA), CONFIG(C,E,LVP0,DISPLAYA),
    CONFIGEND(),
    // config 4.  SPI
    CONFIG(D,E,LPW0,DISPLAYA), CONFIG(D,E,LPW2,DISPLAYA), CONFIG(C,E,LSC1,DISPLAYA),
    CONFIG(C,E,LM0,DISPLAYA), CONFIG(C,E,LVP0,DISPLAYA), CONFIGEND(),
    // Config 5. Panel 86
    BRANCH(7),CONFIG(C,E,LSC1,DISPLAYA),CONFIG(C,E,LM1,DISPLAYA),CONFIGEND(),
    // config 6. 16/18b smart panels
    BRANCH(7),CONFIG(C,E,LDC,DISPLAYA),CONFIG(D,E,LSPI,DISPLAYA),CONFIGEND(),
    MODULEDONE(),
    //  subroutine 1. - 18b data + clock
    CONFIG(C,F,LD0,DISPLAYA), CONFIG(C,F,LD1,DISPLAYA), CONFIG(C,F,LD2,DISPLAYA),
    CONFIG(C,F,LD3,DISPLAYA), CONFIG(C,F,LD4,DISPLAYA), CONFIG(C,F,LD5,DISPLAYA),
    CONFIG(C,F,LD6,DISPLAYA), CONFIG(C,F,LD7,DISPLAYA), CONFIG(C,F,LD8,DISPLAYA),
    CONFIG(C,F,LD9,DISPLAYA), CONFIG(C,F,LD10,DISPLAYA), CONFIG(C,F,LD11,DISPLAYA),
    CONFIG(C,F,LD12,DISPLAYA), CONFIG(C,F,LD13,DISPLAYA), CONFIG(C,F,LD14,DISPLAYA),
    CONFIG(C,F,LD15,DISPLAYA), CONFIG(C,G,LD16,DISPLAYA), CONFIG(C,G,LD17,DISPLAYA),
    CONFIG(C,E,LSC0,DISPLAYA), CONFIGEND(),
    SUBROUTINESDONE(),  // This is required, since BRANCH is used.
/* For handy reference, here is the complete list of CONFIG macros for the display
   pad groups, in case any more configurations are defined in the future.
    CONFIG(C,F,LD0,DISPLAYA), CONFIG(C,F,LD1,DISPLAYA), CONFIG(C,F,LD2,DISPLAYA),
    CONFIG(C,F,LD3,DISPLAYA), CONFIG(C,F,LD4,DISPLAYA), CONFIG(C,F,LD5,DISPLAYA),
    CONFIG(C,F,LD6,DISPLAYA), CONFIG(C,F,LD7,DISPLAYA), CONFIG(C,F,LD8,DISPLAYA),
    CONFIG(C,F,LD9,DISPLAYA), CONFIG(C,F,LD10,DISPLAYA), CONFIG(C,F,LD11,DISPLAYA),
    CONFIG(C,F,LD12,DISPLAYA),
    CONFIG(C,F,LD13,DISPLAYA), CONFIG(C,F,LD14,DISPLAYA), CONFIG(C,F,LD15,DISPLAYA),
    CONFIG(C,G,LD16,DISPLAYA), CONFIG(C,G,LD17,DISPLAYA),CONFIG(C,E,LSC0,DISPLAYA),
    CONFIG(C,E,LVS,DISPLAYA), CONFIG(D,E,LHS,DISPLAYA), CONFIG(D,E,LSPI,DISPLAYA),
    CONFIG(C,G,LHP1,DISPLAYA), CONFIG(C,G,LHP2,DISPLAYA), CONFIG(C,G,LHP0,DISPLAYA),
    CONFIG(C,G,LVP1,DISPLAYA), CONFIG(D,G,LDI,DISPLAYA), CONFIG(D,G,LPP,DISPLAYA),
    CONFIG(C,E,LCSN,DISPLAYA), CONFIG(C,E,LM1,DISPLAYA),CONFIG(C,E,LM0,DISPLAYA),
    CONFIG(D,E,LPW0,DISPLAYA),CONFIG(D,E,LPW2,DISPLAYA), CONFIG(D,E,LPW1,DISPLAYA),
    CONFIG(C,E,LVP0,DISPLAYA), CONFIG(C,E,LDC,DISPLAYA), CONFIG(C,E,LSC1,DISPLAYA),
    CONFIG(D,E,LSDI,DISPLAYA),
    */
};

static const NvU32 g_Ap20Mux_Display2[] = {
    CONFIGEND(),
    //  config 1, 24b RGB.  Pure superset of Config2 (18b RGB)
    BRANCH(2),
    CONFIG(C,G,LHP1,DISPLAYB),CONFIG(C,G,LHP2,DISPLAYB),CONFIG(C,G,LVP1,DISPLAYB),
    CONFIG(C,G,LHP0,DISPLAYB),CONFIG(D,G,LDI,DISPLAYB),CONFIG(D,G,LPP,DISPLAYB),
    CONFIGEND(),
    //  config 2, 18b RGB.
    BRANCH(7),
    CONFIG(C,E,LVS,DISPLAYB), CONFIG(D,E,LHS,DISPLAYB), CONFIG(D,E,LSPI,DISPLAYB),
    CONFIGEND(),
    // config 3, 8 & 9b CPU.
    CONFIG(C,G,LHP1,DISPLAYB), CONFIG(C,G,LHP2,DISPLAYB), CONFIG(C,G,LVP1,DISPLAYB),
    CONFIG(C,G,LHP0,DISPLAYB), CONFIG(D,G,LDI,DISPLAYB), CONFIG(D,G,LPP,DISPLAYB),
    CONFIG(D,E,LPW0,DISPLAYB), CONFIG(D,E,LPW1,DISPLAYB), CONFIG(D,E,LPW2,DISPLAYB),
    CONFIG(C,E,LSC1,DISPLAYB), CONFIG(C,E,LM1,DISPLAYB), CONFIG(C,E,LVP0,DISPLAYB),
    CONFIGEND(),
    // config 4.  SPI
    CONFIG(D,E,LPW0,DISPLAYB), CONFIG(D,E,LPW2,DISPLAYB), CONFIG(C,E,LSC1,DISPLAYB),
    CONFIG(C,E,LM0,DISPLAYB), CONFIG(C,E,LVP0,DISPLAYB), CONFIGEND(),
    // Config 5. Panel 86
    BRANCH(7),CONFIG(C,E,LSC1,DISPLAYB),CONFIG(C,E,LM1,DISPLAYB),CONFIGEND(),
    // config 6. 16/18b smart panels
    BRANCH(7),CONFIG(C,E,LDC,DISPLAYB),CONFIG(D,E,LSPI,DISPLAYB),CONFIGEND(),
    MODULEDONE(),
    //  subroutine 1. (config 7)
    CONFIG(C,F,LD0,DISPLAYB), CONFIG(C,F,LD1,DISPLAYB), CONFIG(C,F,LD2,DISPLAYB),
    CONFIG(C,F,LD3,DISPLAYB), CONFIG(C,F,LD4,DISPLAYB), CONFIG(C,F,LD5,DISPLAYB),
    CONFIG(C,F,LD6,DISPLAYB), CONFIG(C,F,LD7,DISPLAYB), CONFIG(C,F,LD8,DISPLAYB),
    CONFIG(C,F,LD9,DISPLAYB), CONFIG(C,F,LD10,DISPLAYB), CONFIG(C,F,LD11,DISPLAYB),
    CONFIG(C,F,LD12,DISPLAYB), CONFIG(C,F,LD13,DISPLAYB), CONFIG(C,F,LD14,DISPLAYB),
    CONFIG(C,F,LD15,DISPLAYB), CONFIG(C,G,LD16,DISPLAYB), CONFIG(C,G,LD17,DISPLAYB),
    CONFIG(C,E,LSC0,DISPLAYB), CONFIGEND(),
    SUBROUTINESDONE(),
};

static const NvU32* g_Ap20MuxDisplay[] = {
    &g_Ap20Mux_Display1[0],
    &g_Ap20Mux_Display2[0],
    NULL
};

static const NvU32 g_Ap20Mux_Crt[] = {
    CONFIGEND(),
    CONFIG(D,G,CRTP,CRT),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxCrt[] = {
    &g_Ap20Mux_Crt[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Etm[] = {
    CONFIGEND(),
    CONFIG(A,A,KBCF,TRACE),CONFIG(A,C,KBCB,SDIO2),CONFIG(B,C,KBCC,TRACE),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxEtm[] = {
    &g_Ap20Mux_Etm[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Owr[] = {
    UNCONFIG(B,OWC,OWR,RSVD1),UNCONFIG(A,UAC,OWR,RSVD2),UNCONFIG(D,GPU,PWM,RSVD4),
    CONFIGEND(),
    // config 1
    CONFIG(A,B,OWC,OWR),CONFIG(B,A,UAC,OWR),CONFIGEND(),
    // config 2
    CONFIG(A,B,OWC,OWR),CONFIG(A,D,GPU,PWM),CONFIGEND(),
    // config 3
    CONFIG(A,B,OWC,OWR),CONFIG(A,A,KBCE,OWR),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxOwr[] = {
    &g_Ap20Mux_Owr[0],
    NULL,
};

static const NvU32 g_Ap20Mux_Pcie[] = {
    CONFIGEND(),
    CONFIG(A,D,GPV,PCIE),CONFIG(B,D,SDC,PWM),CONFIG(B,B,SLXK,PCIE),
    CONFIG(B,B,SLXA,PCIE),CONFIGEND(),
    MODULEDONE(),
};

static const NvU32 *g_Ap20MuxPcie[] = {
    &g_Ap20Mux_Pcie[0],
    NULL,
};

static const NvU32** g_Ap20MuxControllers[] = {
    &g_Ap20MuxAta[0],
    &g_Ap20MuxCrt[0],
    NULL, // no options for CSI
    &g_Ap20MuxDap[0],
    &g_Ap20MuxDisplay[0],
    NULL, // no options for DSI
    NULL, // no options for GPIO
    &g_Ap20MuxHdcp[0],
    &g_Ap20MuxHdmi[0],
    &g_Ap20MuxHsi[0],
    NULL, // No options for HSMMC
    NULL, // no options for I2S
    &g_Ap20MuxI2c[0],
    &g_Ap20MuxI2cPmu[0],
    &g_Ap20MuxKbd[0],
    &g_Ap20MuxMio[0],
    &g_Ap20MuxNand[0],
    &g_Ap20MuxPwm[0],
    &g_Ap20MuxSdio[0],
    &g_Ap20MuxSflash[0],
    NULL, // no options for Slink
    &g_Ap20MuxSpdif[0],
    &g_Ap20MuxSpi[0],
    &g_Ap20MuxTwc[0],
    NULL, //  no options for TVO
    &g_Ap20MuxUart[0],
    NULL, //  no options for USB
    NULL, //  no options for VDD
    &g_Ap20MuxVi[0],
    NULL, //  no options for XIO
    &g_Ap20MuxExtClock[0],
    &g_Ap20MuxUlpi[0],
    &g_Ap20MuxOwr[0],
    &g_Ap20MuxSnor[0], //  SyncNOR
    &g_Ap20MuxPcie[0],
    &g_Ap20MuxEtm[0],
    NULL, //  no options for TSENSor
    &g_Ap20MuxBacklight[0],
};

NV_CT_ASSERT(NV_ARRAY_SIZE(g_Ap20MuxControllers)==NvOdmIoModule_Num);

const NvU32***
NvRmAp20GetPinMuxConfigs(NvRmDeviceHandle hDevice)
{
    NV_ASSERT(hDevice);
    return (const NvU32***) g_Ap20MuxControllers;
}

/* Define the GPIO port/pin to tristate mappings */

const NvU16 g_Ap20GpioPadGroupMapping[] =
{
    //  Port A
    GPIO_TRISTATE(A,DTE), GPIO_TRISTATE(B,UCB), GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(A,DAP2),
    GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(A,DAP2), GPIO_TRISTATE(B,SDD), GPIO_TRISTATE(B,SDD),
    //  Port B
    GPIO_TRISTATE(A,GMC), GPIO_TRISTATE(A,GMC), GPIO_TRISTATE(D,LPW0), GPIO_TRISTATE(C,LSC0),
    GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC), GPIO_TRISTATE(B,SDC),
    //  Port C
    GPIO_TRISTATE(B,UCB), GPIO_TRISTATE(D,LPW1), GPIO_TRISTATE(B,UAD), GPIO_TRISTATE(B,UAD),
    GPIO_TRISTATE(A,RM), GPIO_TRISTATE(A,RM), GPIO_TRISTATE(D,LPW2), GPIO_TRISTATE(B,GMB),
    //  Port D
    GPIO_TRISTATE(B,SLXK), GPIO_TRISTATE(B,SLXA), GPIO_TRISTATE(A,DTE), GPIO_TRISTATE(B,SLXC),
    GPIO_TRISTATE(B,SLXD), GPIO_TRISTATE(A,DTA), GPIO_TRISTATE(A,DTC), GPIO_TRISTATE(A,DTC),
    //  Port E
    GPIO_TRISTATE(C,LD0), GPIO_TRISTATE(C,LD1), GPIO_TRISTATE(C,LD2), GPIO_TRISTATE(C,LD3),
    GPIO_TRISTATE(C,LD4), GPIO_TRISTATE(C,LD5), GPIO_TRISTATE(C,LD6), GPIO_TRISTATE(C,LD7),
    //  Port F
    GPIO_TRISTATE(C,LD8),GPIO_TRISTATE(C,LD9),GPIO_TRISTATE(C,LD10), GPIO_TRISTATE(C,LD11),
    GPIO_TRISTATE(C,LD12),GPIO_TRISTATE(C,LD13),GPIO_TRISTATE(C,LD14), GPIO_TRISTATE(C,LD15),
    //  Port G
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    //  Port H
    GPIO_TRISTATE(A,ATD), GPIO_TRISTATE(A,ATD),GPIO_TRISTATE(A,ATD), GPIO_TRISTATE(A,ATD),
    GPIO_TRISTATE(B,ATE), GPIO_TRISTATE(B,ATE),GPIO_TRISTATE(B,ATE), GPIO_TRISTATE(B,ATE),
    //  Port I
    GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATB), GPIO_TRISTATE(A,ATA),
    GPIO_TRISTATE(A,ATA),GPIO_TRISTATE(A,ATB), GPIO_TRISTATE(A,ATA), GPIO_TRISTATE(A,ATC),
    //  Port J
    GPIO_TRISTATE(B,GMD),GPIO_TRISTATE(D,LSPI), GPIO_TRISTATE(B,GMD), GPIO_TRISTATE(D,LHS),
    GPIO_TRISTATE(C,LVS),GPIO_TRISTATE(A,IRTX), GPIO_TRISTATE(A,IRRX), GPIO_TRISTATE(A,GMC),
    //  Port K
    GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC), GPIO_TRISTATE(A,ATC),
    GPIO_TRISTATE(A,ATC),GPIO_TRISTATE(B,SPDO), GPIO_TRISTATE(B,SPDI), GPIO_TRISTATE(A,GMC),
    //  Port L
    GPIO_TRISTATE(A,DTD),GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD),
    GPIO_TRISTATE(A,DTD),GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,DTD),
    //  Port M
    GPIO_TRISTATE(C,LD16),GPIO_TRISTATE(C,LD17), GPIO_TRISTATE(C,LHP1), GPIO_TRISTATE(C,LHP2),
    GPIO_TRISTATE(C,LVP1),GPIO_TRISTATE(C,LHP0), GPIO_TRISTATE(D,LDI), GPIO_TRISTATE(D,LPP),
    //  Port N
    GPIO_TRISTATE(A,DAP1),GPIO_TRISTATE(A,DAP1), GPIO_TRISTATE(A,DAP1), GPIO_TRISTATE(A,DAP1),
    GPIO_TRISTATE(C,LCSN),GPIO_TRISTATE(D,LSDA), GPIO_TRISTATE(C,LDC), GPIO_TRISTATE(C,HDINT),
    //  Port O
    GPIO_TRISTATE(B,UAB),GPIO_TRISTATE(B,UAA), GPIO_TRISTATE(B,UAA), GPIO_TRISTATE(B,UAA),
    GPIO_TRISTATE(B,UAA),GPIO_TRISTATE(B,UAB), GPIO_TRISTATE(B,UAB), GPIO_TRISTATE(B,UAB),
    //  Port P
    GPIO_TRISTATE(A,DAP3),GPIO_TRISTATE(A,DAP3), GPIO_TRISTATE(A,DAP3), GPIO_TRISTATE(A,DAP3),
    GPIO_TRISTATE(A,DAP4),GPIO_TRISTATE(A,DAP4), GPIO_TRISTATE(A,DAP4), GPIO_TRISTATE(A,DAP4),
    //  Port Q
    GPIO_TRISTATE(B,KBCC),GPIO_TRISTATE(B,KBCC), GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCF),
    GPIO_TRISTATE(A,KBCF),GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCF), GPIO_TRISTATE(A,KBCE),
    //  Port R
    GPIO_TRISTATE(A,KBCA),GPIO_TRISTATE(A,KBCA), GPIO_TRISTATE(A,KBCA), GPIO_TRISTATE(D,KBCD),
    GPIO_TRISTATE(D,KBCD),GPIO_TRISTATE(D,KBCD), GPIO_TRISTATE(D,KBCD), GPIO_TRISTATE(A,KBCB),
    //  Port S
    GPIO_TRISTATE(A,KBCB),GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB),
    GPIO_TRISTATE(A,KBCB),GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB), GPIO_TRISTATE(A,KBCB),
    //  Port T
    GPIO_TRISTATE(A,DTD), GPIO_TRISTATE(A,CSUS), GPIO_TRISTATE(A,DTB), GPIO_TRISTATE(A,DTB),
    GPIO_TRISTATE(A,DTA), GPIO_TRISTATE(A,PTA), GPIO_TRISTATE(A,PTA), GPIO_TRISTATE(A,ATB),
    //  Port U
    GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU),
    GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(A,GPU), GPIO_TRISTATE(D,GPU7),
    //  Port V
    GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC), GPIO_TRISTATE(B,UAC),
    GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(A,GPV), GPIO_TRISTATE(C,LVP0),
    //  Port W
    GPIO_TRISTATE(C,LM0), GPIO_TRISTATE(C,LM1), GPIO_TRISTATE(B,SPIG), GPIO_TRISTATE(B,SPIH),
    GPIO_TRISTATE(A,CDEV1), GPIO_TRISTATE(A,CDEV2),GPIO_TRISTATE(B,UCA),GPIO_TRISTATE(B,UCA),
    //  Port X
    GPIO_TRISTATE(B,SPIA),GPIO_TRISTATE(B,SPIB),GPIO_TRISTATE(B,SPIC),GPIO_TRISTATE(B,SPIC),
    GPIO_TRISTATE(B,SPID),GPIO_TRISTATE(B,SPIE),GPIO_TRISTATE(B,SPIE),GPIO_TRISTATE(B,SPIF),
    //  Port Y
    GPIO_TRISTATE(D,UDA),GPIO_TRISTATE(D,UDA),GPIO_TRISTATE(D,UDA),GPIO_TRISTATE(D,UDA),
    GPIO_TRISTATE(A,SDIO1),GPIO_TRISTATE(A,SDIO1),GPIO_TRISTATE(A,SDIO1),GPIO_TRISTATE(A,SDIO1),
    //  Port Z
    GPIO_TRISTATE(A,SDIO1),GPIO_TRISTATE(A,SDIO1),GPIO_TRISTATE(D,LSDI),GPIO_TRISTATE(C,LSC1),
    GPIO_TRISTATE(C,LSCK), GPIO_TRISTATE(A,PMC), GPIO_TRISTATE(A,I2CP), GPIO_TRISTATE(A,I2CP),
    //  Port AA
    GPIO_TRISTATE(A,GMA), GPIO_TRISTATE(A,GMA), GPIO_TRISTATE(A,GMA), GPIO_TRISTATE(A,GMA),
    GPIO_TRISTATE(B,GME), GPIO_TRISTATE(B,GME), GPIO_TRISTATE(B,GME), GPIO_TRISTATE(B,GME),
    //  Port BB
    GPIO_TRISTATE(A,PMC), GPIO_TRISTATE(A,DTE), GPIO_TRISTATE(D,DTF), GPIO_TRISTATE(D,DTF),
    GPIO_TRISTATE(A,DTE), GPIO_TRISTATE(A,DTE),
};

NvBool
NvRmAp20GetPinGroupForGpio(NvRmDeviceHandle hDevice,
                           NvU32 Port,
                           NvU32 Pin,
                           NvU32 *pMapping)
{
    const NvU32 GpiosPerPort = 8;
    NvU32 Index = Port*GpiosPerPort + Pin;

    if ((Pin >= GpiosPerPort) || (Index >= NV_ARRAY_SIZE(g_Ap20GpioPadGroupMapping)))
        return NV_FALSE;

    *pMapping = (NvU32)g_Ap20GpioPadGroupMapping[Index];
    return NV_TRUE;
}

NvU32
NvRmPrivAp20GetExternalClockSourceFreq(
    NvRmDeviceHandle hDevice,
    const NvU32* Instance,
    NvU32 Config)
{
    NvU32 MuxCtlShift, MuxCtlSet;
    NvU32 ClockFreqInKHz = 0;

    MuxCtlShift = NV_DRF_VAL(MUX,ENTRY,MUX_CTL_SHIFT,*Instance);
    MuxCtlSet = NV_DRF_VAL(MUX,ENTRY,MUX_CTL_SET,*Instance);

    switch (MuxCtlShift)
    {
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_SHIFT:
        if (MuxCtlSet==APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_PLLA_OUT)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllA0);
        else if (MuxCtlSet==APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_OSC)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
        break;
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_SHIFT:
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_AHB_CLK)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_Ahb);
        else if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_OSC)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_ClkM);
        else if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_PLLP_OUT4)
            ClockFreqInKHz = NvRmPrivGetClockSourceFreq(NvRmClockSource_PllP4);
        break;
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_SHIFT:
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_VI_SENSOR_CLK)
        {
            if (NvRmPowerModuleClockConfig(hDevice, NvRmModuleID_Vi, 0, 0, 0,
                NULL, 0, &ClockFreqInKHz, NvRmClockConfig_SubConfig) != NvSuccess)
            {
                ClockFreqInKHz = 0;
            }
        }
        break;
    default:
        ClockFreqInKHz = 0;
    }

    return ClockFreqInKHz;
}

void NvRmPrivAp20EnableExternalClockSource(
    NvRmDeviceHandle hDevice,
    const NvU32* Instance,
    NvU32 Config,
    NvBool ClockState)
{
    NvU32 MuxCtlShift = NV_DRF_VAL(MUX,ENTRY,MUX_CTL_SHIFT,*Instance);
    NvU32 MuxCtlSet = NV_DRF_VAL(MUX,ENTRY,MUX_CTL_SET,*Instance);
    NvU32 ClkEnbShift = ~0;

    switch (MuxCtlShift)
    {
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_SHIFT:
        ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_DEV1_OUT_SHIFT;
         if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV1_SEL_PLLA_OUT)
         {
             NvRmPrivExternalClockAttach(
                 hDevice, NvRmClockSource_PllA0, ClockState);
         }
        break;
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_SHIFT:
        ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_DEV2_OUT_SHIFT;
        if (MuxCtlSet == APB_MISC_PP_PIN_MUX_CTL_C_0_CDEV2_SEL_PLLP_OUT4)
        {
            NvRmPrivExternalClockAttach(
                hDevice, NvRmClockSource_PllP4, ClockState);
        }
        break;
    case APB_MISC_PP_PIN_MUX_CTL_C_0_CSUS_SEL_SHIFT:
        ClkEnbShift = CLK_RST_CONTROLLER_CLK_ENB_U_SET_0_SET_CLK_ENB_SUS_OUT_SHIFT;
        break;
    default:
        return;
    }

    if (ClockState)
    {
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_SET_0, (1UL<<ClkEnbShift));
    }
    else
    {
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_CLK_ENB_U_CLR_0, (1UL<<ClkEnbShift));
    }
}

NvBool NvRmPrivAp20RmModuleToOdmModule(
    NvRmModuleID RmModule,
    NvOdmIoModule *OdmModule,
    NvU32 *OdmInstance,
    NvU32 *pCnt)
{
    NvRmModuleID Module = NVRM_MODULE_ID_MODULE(RmModule);
    NvU32 Instance = NVRM_MODULE_ID_INSTANCE(RmModule);
    NvBool Success = NV_TRUE;
    *pCnt = 1;
    switch (Module)
    {
    case NvRmModuleID_Usb2Otg:
        switch (Instance)
        {
        case 0:
            *OdmModule = NvOdmIoModule_Usb;
            *OdmInstance = 0;
            break;
        case 1:
            *OdmModule = NvOdmIoModule_Ulpi;
            *OdmInstance = 0;
            break;
        case 2:
            *OdmModule = NvOdmIoModule_Usb;
            *OdmInstance = 1;
            break;
        default:
            NV_ASSERT(!"Invalid USB instance");
            break;
        }
        break;
    case NvRmModuleID_OneWire:
        *OdmModule = NvOdmIoModule_OneWire;
        *OdmInstance = Instance;
        break;
    case NvRmModuleID_SyncNor:
        *OdmModule = NvOdmIoModule_SyncNor;
        *OdmInstance = Instance;
        break;
    case NvRmPrivModuleID_Pcie:
        *OdmModule = NvOdmIoModule_PciExpress;
        *OdmInstance = Instance;
        break;
    default:
        Success = NV_FALSE;
        *pCnt = 0;
        break;
    }
    return  Success;
}

NvError
NvRmPrivAp20GetModuleInterfaceCaps(
    NvOdmIoModule Module,
    NvU32 Instance,
    NvU32 PinMap,
    void *pCaps)
{
    switch (Module)
    {
    case NvOdmIoModule_Sdio:
        if (Instance == 1)
        {
            if (PinMap == NvOdmSdioPinMap_Config2 || PinMap == NvOdmSdioPinMap_Config4)
                ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
            else if (PinMap == NvOdmSdioPinMap_Config1 ||
            PinMap == NvOdmSdioPinMap_Config3 || PinMap == NvOdmSdioPinMap_Config5)
                ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 4;
            else
            {
                NV_ASSERT(NV_FALSE);
                return NvError_NotSupported;
            }
        }
        else if (Instance==2 && PinMap==NvOdmSdioPinMap_Config1)
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
        else if (Instance==3 && (PinMap==NvOdmSdioPinMap_Config1 || PinMap==NvOdmSdioPinMap_Config2))
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 8;
        else
            ((NvRmModuleSdmmcInterfaceCaps *)pCaps)->MmcInterfaceWidth = 4;
        return NvError_Success;

    case NvOdmIoModule_Pwm:
        if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config1))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 15;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config2))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 13;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config3))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 1;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config4))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 12;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config5))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 15;
        else if (Instance == 0 && (PinMap == NvOdmPwmPinMap_Config6))
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 3;
        else
        {
            ((NvRmModulePwmInterfaceCaps *)pCaps)->PwmOutputIdSupported = 0;
            return NvError_NotSupported;
        }
        return NvError_Success;
    case NvOdmIoModule_Nand:
        if (Instance == 0 && (PinMap == NvOdmNandPinMap_Config1 || PinMap ==
        NvOdmNandPinMap_Config3))
        {
            ((NvRmModuleNandInterfaceCaps*)pCaps)->IsCombRbsyMode = NV_TRUE;
            ((NvRmModuleNandInterfaceCaps*)pCaps)->NandInterfaceWidth = 16;
        }
        else if (Instance == 0 && (PinMap == NvOdmNandPinMap_Config2 ||
            PinMap == NvOdmNandPinMap_Config4))
        {
            ((NvRmModuleNandInterfaceCaps*)pCaps)->IsCombRbsyMode = NV_TRUE;
            ((NvRmModuleNandInterfaceCaps*)pCaps)->NandInterfaceWidth = 8;
        }
        else
        {
            NV_ASSERT(NV_FALSE);
            return NvError_NotSupported;
        }
        return NvSuccess;
    case NvOdmIoModule_Uart:
        if (Instance == 0)
        {
            if (PinMap == NvOdmUartPinMap_Config1)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 8;
            else if (PinMap == NvOdmUartPinMap_Config2)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 7;
            else if ((PinMap == NvOdmUartPinMap_Config3) || (PinMap == NvOdmUartPinMap_Config6))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
            else if ((PinMap == NvOdmUartPinMap_Config4) || (PinMap == NvOdmUartPinMap_Config5))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
            else if (PinMap == NvOdmUartPinMap_Config7)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 6;
            else
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
        }
        else if ((Instance == 1) || (Instance == 2))
        {
            if (PinMap == NvOdmUartPinMap_Config1)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
            else if (PinMap == NvOdmUartPinMap_Config2)
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 2;
            else
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
        }
        else if ((Instance == 3) || (Instance == 4))
        {
            if ((PinMap == NvOdmUartPinMap_Config1) || (PinMap == NvOdmUartPinMap_Config2))
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 4;
            else
                ((NvRmModuleUartInterfaceCaps *)pCaps)->NumberOfInterfaceLines = 0;
        }
        else
        {
            NV_ASSERT(NV_FALSE);
            return NvError_NotSupported;
        }
        return NvSuccess;

    default:
        break;
    }

    return NvError_NotSupported;
}

NvError
NvRmAp20GetStraps(
    NvRmDeviceHandle hDevice,
    NvRmStrapGroup StrapGroup,
    NvU32* pStrapValue)
{
    NvU32 reg = NV_REGR(
        hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_STRAPPING_OPT_A_0);

    switch (StrapGroup)
    {
        case NvRmStrapGroup_RamCode:
            reg = NV_DRF_VAL(APB_MISC_PP, STRAPPING_OPT_A, RAM_CODE, reg);
            break;
        default:
            return NvError_NotSupported;
    }
    *pStrapValue = reg;
    return NvSuccess;
}

static const NvU32 g_VddioNand[] = {
    TRISTATE_UNUSED (ATA,A), TRISTATE_UNUSED (ATB,A), TRISTATE_UNUSED (ATC,A),
    TRISTATE_UNUSED (ATD,A), TRISTATE_UNUSED (PTA,A), TRISTATE_UNUSED (GMA,A),
    TRISTATE_UNUSED (GMC,A), TRISTATE_UNUSED (GMD,B), TRISTATE_UNUSED (GMB,B),
    TRISTATE_UNUSED (ATE,B), TRISTATE_UNUSED (GME,B),
};

void NvRmAp20SetDefaultTristate(NvRmDeviceHandle hDevice)
{
    NvU32 Size = 0;
    NvU32 TsOffs;
    NvU32 TsShift ;
    NvU32 i = 0;
    Size = NV_ARRAY_SIZE(g_VddioNand) / sizeof(NvU32);
    for ( i =0; i< Size; i++)
    {
        TsOffs = NV_DRF_VAL(MUX,ENTRY, TS_OFFSET, g_VddioNand[i]);
        TsShift = NV_DRF_VAL(MUX,ENTRY, TS_SHIFT,  g_VddioNand[i]);
        NvU32 Curr = NV_REGR(hDevice,
             NvRmModuleID_Misc, 0 ,
            APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs);
        Curr &= ~(1<<TsShift);
        Curr |= 1<<TsShift;

        NV_REGW(hDevice, NvRmModuleID_Misc, 0 ,
            APB_MISC_PP_TRISTATE_REG_A_0 + 4*TsOffs, Curr);
    }
}

