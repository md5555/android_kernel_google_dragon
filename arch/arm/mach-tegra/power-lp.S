/*
 * arch/arm/mach-tegra/power-lp.S
 *
 * CPU hotplug save and restore for Tegra SoCs based on Cortex A9MP
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/vfpmacros.h>
#include "ap20/arflow_ctlr.h"
#include "ap20/arclk_rst.h"
#include "ap20/arevp.h"
#include "ap20/arapbpm.h"
#include "ap20/aremc.h"
#include "nvrm_drf.h"

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define EMC_PA_BASE 0x7000f400
#define PMC_PA_BASE 0x7000e400
#define FLOW_PA_BASE 0x60007000
#define TIMERUS_PA_BASE 0x60005010
#define CLK_RST_PA_BASE 0x60006000
#define EVP_PA_BASE 0x6000f000
#define CSITE_PA_BASE 0x70040000
#define TEMP_RESET_VECTOR 8
#define TEMP_SCLK_BURST_POLICY 16
#define TEMP_CCLK_BURST_POLICY 20
#define TEMP_PLLX_BASE 12
#define CSITE_CPUDBG0_LAR_0 0x10fb0
#define CSITE_CPUDBG1_LAR_0 0x12fb0
#define TEMP_AREA_SIZE 16
#define DEBUG_FORCE_RTC_WAKEUP 5
#else
#error "Unrecognized Tegra SoC Family"
#endif

	//r0 - Power state
	//r1 - proc_id
ENTRY(enter_power_state)
	//We should be in SVC mode
	//with IRQs turned off
	mrs	r2, CPSR
	stmfd	sp!, {r0-r12, lr}
	stmfd	sp!, {r0}

	cmp	r1, #0
	bne	save_arm_state

	//Wait for everyone else to spin down
wait_for_other_cores:
	stmfd	sp!, {r1}
	bl	check_for_cpu1_reset
	ldmfd	sp!, {r1}

	cmp	r0, #0
	beq	finish_power_state

	//Save the local timers
	stmfd	sp!, {r1}
	bl	save_local_timers
	ldmfd	sp!, {r1}

	//Ok we can save state for core0 now	
save_arm_state:	
	//Get the context save pointer for the core
	ldr	r0, =g_contextSaveVA
	ldr	r0, [r0]
	mov	r2, #0x800
	//r0 = r0 + r1 * r2
	smlabb	r0, r1, r2, r0

	ldr	r1, =g_ArmPerif
	ldr	r1, [r1]

	//We need r0 = virtual context save
	//We need R1 = SCU VA
	b	ArmCortexA9Save

ArmCortexA9Saved:
	//All cores but core 0 must be reset
	mrc	p15, 0, r2, c0, c0, 5
	ands	r2, r2, #0x3	
	bne	reset_slave

	//Check which power state we want to enter
	ldmfd	sp!, {r0}
	//Is it LP2?
	cmp r0, #0
	ldreq r2, =g_enterLP2PA
	ldreq r2, [r2]
	beq transition_to_state

	ldr r4, =g_pIRAM
	ldr r4, [r4]

	//Is it LP1?
	cmp r0, #1
	ldreq r5, =exit_lp1_end
	ldreq r6, =enter_lp1
	beq	copy_to_iram

	//Is it LP0?
	cmp r0, #2
	ldr r5, =enter_lp0_end
	ldr r6, =enter_lp0

	//For LP0, the AVP stores it's continuation address at the first
	//location in IRAM. Before we overwrite IRAM with the LP0 entry
	//code, copy the AVP continuation and store it in the scratch
	//register dedicated for this purposed.

	//R1 = *g_pIRAM
	ldr r1, [r4]
	//R3 = &(g_pPMC)
	ldr r3, =g_pPMC
	//R3 = g_pPMC
	ldr r3, [r3]
	//Store in scratch39
	str r1, [r3, #APBDEV_PMC_SCRATCH39_0]

copy_to_iram:
	//Copy the enter_lp0 function to IRAM using 8x4 block moves.
	//It doesn't matter if we copy a few extra words.
	//IRAM has already been safely saved by the AVP at this point
	//R4 = destination address to copy code to
	//R5 = size of code to copy in bytes
	//R6 = source address to copy code from

	//r2 is the source address
	cpy r2, r6
	//r3 is the size to copy
	sub r3, r5, r6

copy_code:
	//Load source
	ldmia   r2!, {r5-r12}
	//Store at destination
	stmia   r4!, {r5-r12}
	//Decrement count
	subs    r3, r3, #32
	bgt     copy_code

	//Get the physical address of IRAM
	//This is where we will jump to start LP0
	ldr r2, =g_IramPA
	ldr r2, [r2]

	//We are the master. We should
	//turn of MMUs and caches.
	//Write a value to unblock the slaves

transition_to_state:
	//Turn off caches and MMU
	mrc	p15, 0, r3, c1, c0, 0
	bic	r3, r3, #(1<<12) //I-Cache
	bic	r3, r3, #(1<<11) //Branch-pred
	bic	r3, r3, #(1<<2)  //D-Cache
	bic	r3, r3, #(1<<0)  //MMU

	ldr	r0, =g_modifiedPlls
	ldr	r0, [r0]
	ldr	r1, =g_wakeupCcbp
	ldr	r1, [r1] 

	mov	r10, #0	
	mcr	p15, 0, r10, c8, c7, 0	// invalidate TLB
	dsb

	.align 5
	//Disable L1 caches and MMU
	mcr	p15, 0, r3, c1, c0, 0

	//Jump to the appropriate LPx function
	//bl enter_lp2
	bx	r2

	ldr	r2, =g_Sync
	mov	r3, #1
	str	r3, [r2]

	b	finish_power_state

reset_slave:
	//Reset the slave cores
	mov	r0, #1
	mov r1, #1
	bl	reset_cpu
	b .

 	ldr	r2, =g_Sync
wait_for_master: 	
 	ldr	r3, [r2]
 	cmp	r3, #1
 	bne	wait_for_master

	//Reset the sync variable
	//and increment g_ActiveCpus.
 	mov	r3, #0
 	str	r3, [r2]

finish_power_state:
	ldmfd	sp!, {r0}
	ldmfd	sp!, {r0-r12, lr}
	bx	lr
.ltorg
ENDPROC(EnterPowerState)

ENTRY(enter_lp2)
	ldr	r5, =PMC_PA_BASE	//R5 = PMC PA base address
	ldr	r6, =FLOW_PA_BASE	//R6 = FLOW PA base address
	ldr	r7, =TIMERUS_PA_BASE //R7 = TIMERUS PA base address
	ldr	r8, =CLK_RST_PA_BASE //R8 = CLK PA base address
	ldr	r9, =EVP_PA_BASE	//R9 = EVP PA base address

	//This funny little instruction obtains a piece of memory
	//that is relative to the PC. We can't use literals
	//as the MMU has been turned off.	
	add	r12, pc, #TempStoreArea-(.+8)

	//Save the input paramters in temp region
	stmia	r12, {r0-r1}

	//Save old reset vector
	ldr	r2, [r9, #EVP_CPU_RESET_VECTOR_0]
	str	r2, [r12, #TEMP_RESET_VECTOR]

	//Save pllx base
	ldr	r2, [r8, #CLK_RST_CONTROLLER_PLLX_BASE_0]
	str	r2, [r12, #TEMP_PLLX_BASE]

	//Store WB2 entry point in reset
	add	r2, pc, #exit_lp2-(.+8)
	str	r2, [r9, #EVP_CPU_RESET_VECTOR_0]

	//Make sure SIDE_EFFECT_LP0 is not set
	ldr	r2, [r5, #APBDEV_PMC_CNTRL_0]	
	//Unset the SIDE_EFFECT bit
	bic	r2, r2, #(1<<14)
	str	r2, [r5, #APBDEV_PMC_CNTRL_0]

	//Powergate the cpu by setting the ENABLE bit
	ldr	r2, [r6, #FLOW_CTLR_CPU_CSR_0]
	orr	r2, r2, #(1<<0)
	str	r2, [r6, #FLOW_CTLR_CPU_CSR_0]

	//Put the CPU on the desired clock source for wakeup.
	str	r1, [r8, #CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0]
	dmb

	//Finally, halt the CPU
	mov	r2, #0
	orr	r2, r2, #(4<<29)  //STOP_UNTIL_IRQ
	orr	r2, r2, #(1<<10)  //IRQ_0 event
	orr	r2, r2, #(1<<8)	//FIQ_0 event
	str	r2, [r6, #FLOW_CTLR_HALT_CPU_EVENTS_0]

DoWFI:
	dsb		
	wfi
	b	DoWFI	
ENDPROC(enter_lp2)

ENTRY(exit_lp2)
	ldr	r5, =PMC_PA_BASE	//R5 = PMC PA base address
	ldr	r6, =FLOW_PA_BASE	//R6 = FLOW PA base address
	ldr	r7, =TIMERUS_PA_BASE //R7 = TIMERUS PA base address
	ldr	r8, =CLK_RST_PA_BASE //R8 = CLK PA base address
	ldr	r9, =EVP_PA_BASE	//R9 = EVP PA base address
	ldr	r10, =CSITE_PA_BASE  //R10 = CSITE PA base address

	//Check which core we are by checking the MPIDR
	mrc	p15, 0, r2, c0, c0, 5
	ands	r2, r2, #0x3
	bne	skip_cpu0_restore

	//This funny little instruction obtains a piece of memory
	//that is relative to the PC. We can't use literals
	//as the MMU has been turned off.
	add	r12, pc, #TempStoreArea-(.+8)

	//Get the current microsecond count
	ldr	r11, [r7, #0]

	//NOTE: Any low-power state that cuts power to the CPU power island
	//but not to the island containing the APB bus on which
	//CoreSight resides will cause a JTAG debugger to lose its
	//connection to the target after the power is restored to
	//the CPU power island. This is because CoreSight will remain
	//active while the CPU is powered down and the debugger will
	//continue trying to talk to the CPU. This will cause the
	//CoreSight-to-CPU interface to stall. Normally, this will
	//not cause a problem because condition will be resolved when
	//the CoreSight state is restored as part of the CPU state
	//(DORMANT_RESTORE2). However, in order to be able to debug
	//the early stages of CPU restoration, it is necessary to
	//reset CoreSight here. Be aware than any on-chip breakpoint
	//set before the CPU island was powered down will not be
	//functional until after CPU state restoration.

	//Assert CoreSight reset.
	ldr	r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]
	orr	r0, r0, #(1<<9)
	str	r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]

	//Hold CoreSight reset for 2us.
	add	r1, r11, #2
reset_poll:
	ldr	r2, [r7, #0]
	cmp	r2, r1
	ble	reset_poll

	//De-assert CoreSight reset.
	bic	r0, r0, #(1<<9)
	str	r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]

	//Unlock debugger access by writing special "CSACCESS"
	ldr	r0, =0xC5ACCE55
	ldr	r1, =CSITE_CPUDBG0_LAR_0  //R1 = CPU0 lock offset
	ldr	r2, =CSITE_CPUDBG1_LAR_0  //R2 = CPU1 lock offset
	str	r0, [r10, r1]		//Unlock CPU0
	str	r0, [r10, r2]		//Unlock CPU1

	//Make sure we no longer powergate the CPU island when halting.
	ldr	r1, [r6, #FLOW_CTLR_CPU_CSR_0]
	bic	r1, r1, #(1<<0)
	str	r1, [r6, #FLOW_CTLR_CPU_CSR_0]

	//Restore the input parameters passed to enter_lp2
	ldm	r12, {r0-r1}

	//Put the CPU on the desired clock source for wakeup
	str	r1, [r8, #CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0]

	//Enable PLL-C if we disabled it.
	//tst r0, #PowerPllC
	tst	r0, #1
	beq	no_pllc
	ldr	r2, [r8, #CLK_RST_CONTROLLER_PLLC_BASE_0]
	mov	r3, #(1<<30) //PllC ENABLE
	tst	r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLC_BASE_0]
no_pllc:
	//Enable PLL-M if we disabled it.
	//tst r0, #PowerPllM
	tst	r0, #2
	beq	no_pllm
	ldr	r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]
	mov	r3, #(1<<30) //PllM ENABLE
	tst	r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]
no_pllm:
	//Enable PLL-P if we disabled it.
	//tst r0, #PowerPllP
	tst	r0, #3
	beq	no_pllp
	ldr	r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]
	mov	r3, #(1<<30) //PllP ENABLE
	tst	r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]
no_pllp:
	//Enable PLL-X if we disabled it.
	//NOTE: The PLLX_BASE have have been cleared so get it's value
	//from the temporary save area.
	//tst r0, #PowerPllX
	tst	r0, #4
	beq	no_pllx
	ldr	r2, [r12, #TEMP_PLLX_BASE]
	mov	r3, #(1<<30) //PllX ENABLE
	orr	r2, r2, r3
	str	r2, [r8, #CLK_RST_CONTROLLER_PLLX_BASE_0]
no_pllx:
	//Restore the reset vector target.
	//##
	//ldr r1, [r12, #TEMP_RESET_VECTOR]
	//str r1, [r9, #EVP_CPU_RESET_VECTOR_0]

skip_cpu0_restore:
	mrc	p15, 0, r2, c0, c0, 5
	ands	r2, r2, #0x3
	//Write to reset vector to allow platsmp to continue
	str	r2, [r9, #EVP_CPU_RESET_VECTOR_0]

	//Set lr to the resume function
	ldr	lr, [r5, #APBDEV_PMC_SCRATCH33_0]
	bx	lr

TempStoreArea:
	//Create some empty space. We can't use literals
	//after the MMU has been turned off, so we need
	//some PC relative scratch space
	.space TEMP_AREA_SIZE

ENDPROC(exit_lp2)

ENTRY(enter_lp1)
	add		r4, pc, #lp1_literals-(.+8)
	ldr		r4, [r4]
	add		r5, pc, #lp1_literals-(.+4)
	ldr		r5, [r5]
	add		r6, pc, #lp1_literals-(.+0)
	ldr		r6, [r6]
	add		r7, pc, #lp1_literals-(.-4)
	ldr		r7, [r7]
	add		r8, pc, #lp1_literals-(.-8)
	ldr		r8, [r8]
	add		r9, pc, #lp1_literals-(.-12)
	ldr		r9, [r9]

	add		r12, pc, #TemporaryStore-(.+8)
	orr		r0, r0, #5
	stmia	r12!, {r0, r1}

	ldr		r0, [r9, #EVP_CPU_RESET_VECTOR_0]
	ldr		r1, [r8, #CLK_RST_CONTROLLER_PLLX_BASE_0]
	ldr		r2, [r8, #CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0]
	ldr		r3, [r8, #CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0]
	stmia	r12, {r0 - r3}
	sub	r12, r12, #8

	add		r2, pc, #exit_lp1-(.+8)
	str		r2, [r9, #EVP_CPU_RESET_VECTOR_0]

	dmb

	//Stall incoming EMC read/write transactions
	mov		r2, #3
	str		r2, [r4, #0x2B0]

	//Poll till EMC is idle
is_idle1:
	ldr		r2, [r4, #0x2B4]
	tst		r2, #4
	beq		is_idle1

	//Put SDRAM into self refresh
	mov		r2, #1
	str		r2, [r4, #0xE0]
	ldr		r2, [r4, #0x10]
	ands	r2, r2, #3, 8
	moveq	r0, #1, 24
	movne	r0, #3, 24

	//Poll until all devices are in self refresh
is_self1:
	ldr		r2, [r4, #0x2B4]
	and		r2, r2, r0
	teq		r0, r2
	bne		is_self1

	//Make sure SIDE_EFFECT_LP0 is not set
	ldr		r2, [r5, #APBDEV_PMC_CNTRL_0]
	//Unset the SIDE_EFFECT bit
	bic		r2, r2, #(1<<14)
	str		r2, [r5, #APBDEV_PMC_CNTRL_0]

	//Powergate the cpu by setting the ENABLE bit
	ldr		r2, [r6, #FLOW_CTLR_CPU_CSR_0]
	orr		r2, r2, #(1<<0)
	str		r2, [r6, #FLOW_CTLR_CPU_CSR_0]

	mov		r2, #1, 4
	str		r2, [r8, #CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0]

	//Switch clocks to 32Khz
	mov		r2, #0x61, 4
	mov		r3, #0
	str		r2, [r8, #CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0]
	str		r3, [r8, #CLK_RST_CONTROLLER_SUPER_SCLK_DIVIDER_0]

	//Turn off pll-m
	ldr		r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]
	bic		r2, r2, #1, 2
	str		r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]

	//Turn off pll-p - Uncomment in real testing
	//ldr       r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]
	//bic       r2, r2, #1, 2
	//str       r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]

	//Get the microsecond count before LP1
	ldr		r2, [r7]
	str		r2, [r5, #0x134]

	//Finally, halt the CPU
	mov		r2, #0
	orr		r2, r2, #(4<<29)  //STOP_UNTIL_IRQ
	orr		r2, r2, #(1<<10)  //IRQ_0 event
	orr		r2, r2, #(1<<8)	//FIQ_0 event
	str		r2, [r6, #FLOW_CTLR_HALT_CPU_EVENTS_0]

do_wfi1:
	dsb
	wfi
	b		do_wfi1
enter_lp1_end:
.ltorg
ENDPROC(enter_lp1)

ENTRY(exit_lp1)
	add		r4, pc, #lp1_literals-(.+8)
	ldr		r4, [r4]
	add		r5, pc, #lp1_literals-(.+4)
	ldr		r5, [r5]
	add		r6, pc, #lp1_literals-(.+0)
	ldr		r6, [r6]
	add		r7, pc, #lp1_literals-(.-4)
	ldr		r7, [r7]
	add		r8, pc, #lp1_literals-(.-8)
	ldr		r8, [r8]
	add		r9, pc, #lp1_literals-(.-12)
	ldr		r9, [r9]
	add		r10, pc, #lp1_literals-(.-16)
	ldr		r10, [r10]

	//R12 = Temporary iram store
	add		r12, pc, #TemporaryStore-(.+8)

	//Read the microsecond counter
	ldr		r11, [r7]

	//Assert CoreSight reset.
	ldr		r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]
	orr		r0, r0, #(1<<9)
	str		r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]

	//Hold CoreSight reset for 2us.
	add		r1, r11, #2
reset_poll1:
	ldr		r2, [r7, #0]
	cmp		r2, r1
	ble		reset_poll1

	//De-assert CoreSight reset.
	bic		r0, r0, #(1<<9)
	str		r0, [r8, #CLK_RST_CONTROLLER_RST_DEVICES_U_0]

	//Unlock debugger access by writing special "CSACCESS"
	ldr		r0, =0xC5ACCE55
	ldr		r1, =CSITE_CPUDBG0_LAR_0  //R1 = CPU0 lock offset
	ldr		r2, =CSITE_CPUDBG1_LAR_0  //R2 = CPU1 lock offset
	str		r0, [r10, r1]		//Unlock CPU0
	str		r0, [r10, r2]		//Unlock CPU1

	//Switch the system to CLKM
	mov		r2, #1, 4
	str		r2, [r8, #CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0]

	//Check which plls to start
	ldr		r0, [r12]
	mov		r1, #0
	tst		r0, #2

	//Enable PLL-C if we disabled it.
	//tst r0, #PowerPllC
	tst		r0, #1
	beq		no_pllc1
	ldr		r2, [r8, #CLK_RST_CONTROLLER_PLLC_BASE_0]
	mov		r3, #(1<<30) //PllC ENABLE
	tst		r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLC_BASE_0]
no_pllc1:
	//Enable PLL-M if we disabled it.
	//tst r0, #PowerPllM
	tst		r0, #2
	beq		no_pllm1
	ldr		r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]
	mov		r3, #(1<<30) //PllM ENABLE
	tst		r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLM_BASE_0]
no_pllm1:
	//Enable PLL-P if we disabled it.
	//tst r0, #PowerPllP
	mov		r0, #
	tst		r0, #3
	beq		no_pllp1
	ldr		r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]
	mov		r3, #(1<<30) //PllP ENABLE
	tst		r2, r3
	orreq	r2, r2, r3
	streq	r2, [r8, #CLK_RST_CONTROLLER_PLLP_BASE_0]
no_pllp1:
	//Enable PLL-X if we disabled it.
	//NOTE: The PLLX_BASE have have been cleared so get it's value
	//from the temporary save area.
	//tst r0, #PowerPllX
	tst		r0, #4
	beq		no_pllx1
	ldr		r2, [r12, #TEMP_PLLX_BASE]
	mov		r3, #(1<<30) //PllX ENABLE
	orr		r2, r2, r3
	str		r2, [r8, #CLK_RST_CONTROLLER_PLLX_BASE_0]
no_pllx1:
	//Configure CPU island to not be power gated
	ldr		r2, [r6, #8]
	bic		r2, r2, #1
	str		r2, [r6, #8]

	//Restore the reset vector target
	ldr		r2, [r12, #TEMP_RESET_VECTOR]
	str		r2, [r9, #EVP_CPU_RESET_VECTOR_0]

	//Check if we started any plls
	cmp		r1, #0
	beq		restore_clocks

	//Explicit delay for pll stabilization
	mov		r0, #0xFF
	mov		r1, #0x42, 8
	mov		r2, #0x4B, 30
delay_for_pll:
	cmp		r2, r0
	orrle	r3, r1, r2
	orrgt	r3, r1, r0
	sub		r2, r2, r0
	//Do a flow control wait
	str		r3, [r6]
wait_for_event:
	wfe

	//Check if the event is ours
	ldr		r3, [r6, #FLOW_CTLR_CPU_CSR_0]
	tst		r3, #1, 18
	strne	r3, [r6, #FLOW_CTLR_CPU_CSR_0]
	bne		event_signalled

	//Check if we're still waiting for an event
	ldr		r3, [r6, #FLOW_CTLR_HALT_CPU_EVENTS_0]
	tst		r3, #0xE, 4
	bne		wait_for_event
event_signalled:
	//Check if we waited long enough
	cmp		r2, #0
	bgt		delay_for_pll
restore_clocks:
	//Restore the system and CPU burst, csite, clksrc registers
	ldr		r1, [r12, #TEMP_SCLK_BURST_POLICY]
	ldr		r2, [r12, #TEMP_CCLK_BURST_POLICY]
	str		r1, [r8, #CLK_RST_CONTROLLER_SCLK_BURST_POLICY_0]
	str		r2, [r8, #CLK_RST_CONTROLLER_CCLK_BURST_POLICY_0]

	//Take the SDRAM out of self-refresh

	//Make sure that the sdram is not gated
	ldr		r1, [r4, #EMC_CFG_0]
	bic		r1, r1, #2, 2
	str		r1, [r4, #EMC_CFG_0]

	//Take the sdram out of self refresh
	mov		r1, #0
	str		r1, [r4, #EMC_SELF_REF_0]

	//Issue a couple of nops
	mov		r1, #1
	str		r1, [r4, #EMC_NOP_0]
	str		r1, [r4, #EMC_NOP_0]

	//Issue a refresh command
	mov		r1, #1
	str		r1, [r4, #EMC_REF_0]

	//Confirm that all chips have exited self-refresh
	ldr		r1, [r4, #EMC_ADR_CFG_0]
	ands	r1, r1, #3, 8
	moveq	r0, #1, 24
	movne	r0, #3, 24

	//Poll until all chips have exited self-refresh
is_auto:
	ldr		r1, [r4, #EMC_EMC_STATUS_0]
	and		r2, r1, r0
	cmp		r2, #0
	bne		is_auto

	//Un-stall incoming reads/writes
	mov		r1, #0
	str		r1, [r4, #EMC_REQ_CTRL_0]

	//Store the LP1 exit time and restore return addr
	ldr		lr, [r5, #APBDEV_PMC_SCRATCH1_0]
	str		r11, [r5, #APBDEV_PMC_SCRATCH1_0]
	bx		lr
lp1_literals:
	.word	0x7000f400
	.word	0x7000e400
	.word	0x60007000
	.word	0x60005010
	.word	0x60006000
	.word	0x6000f000
	.word	0x70040000
TemporaryStore:
	//Create some empty space. We can't use literals
	//after the MMU has been turned off, so we need
	//some PC relative scratch space
	.space TEMP_AREA_SIZE
exit_lp1_end:
ENDPROC(exit_lp1)

ENTRY(enter_lp0)
	ldr		r4, [pc, #0xC8] //EMC base
	ldr		r5, [pc, #0xC8] //PMC base
	ldr		r6, [pc, #0xC8] //FLOW base
	ldr		r7, [pc, #0xC8] //TIMERUS base
	ldr		r8, [pc, #0xC8] //RTC base

	//Flush the write buffer
	dmb

	//Stall incoming EMC read/write transactions
	mov		r2, #3
	str		r2, [r4, #0x2B0]

	//Poll till EMC is idle
is_idle:
	ldr		r2, [r4, #0x2B4]
	tst		r2, #4
	beq		is_idle

	//Put SDRAM into self refresh
	mov		r2, #1
	str		r2, [r4, #0xE0]
	ldr		r2, [r4, #0x10]
	ands	r2, r2, #3, 8
	moveq	r0, #1, 24
	movne	r0, #3, 24

	//Poll until all devices are in self refresh
is_self:
	ldr		r2, [r4, #0x2B4]
	and		r2, r2, r0
	teq		r0, r2
	bne		is_self
	mov		r2, #1
	str		r2, [r5, #0x20]

	//Set SIDE_EFFECT_LP0
	ldr		r2, [r5]
	orr		r2, r2, #1, 18
	str		r2, [r5]

	//Set CPU island to power gate when halted
	ldr		r2, [r6, #8]
	orr		r2, r2, #1
	str		r2, [r6, #8]

	//r0 = RTC_BASE
	mov r0, r8
	//setup rtc wake
	ldr r2, [r0, #0x10] //milli
	ldr r2, [r0, #0x8] //shadow

	add r2, r2, #DEBUG_FORCE_RTC_WAKEUP
rtc_idle1:
	ldr r1, [r0, #0x4]
	tst r1, #0x1
	bne rtc_idle1
	str r2, [r0, #0x14]
rtc_idle2:
	ldr r1, [r0, #0x4]
	tst r1, #0x1
	bne rtc_idle2
	//intr mask alarm0
	mov r2, #1
	str r2, [r0, #0x28]
rtc_idle3:
	ldr r1, [r0, #0x4]
	tst r1, #0x1
	bne rtc_idle3
	//Save the microsecond count before LP0
	ldr		r2, [r7]
	str		r2, [r5, #0x134]

	//Halt the CPU without any wakeup events
	mov		r2, #1, 2
	str		r2, [r6]
do_wfi:
	dsb
	wfi
	b		do_wfi
	andvc	pc, r0, r0, lsl #8
	andvc	lr, r0, r0, lsl #8
	andvs	r7, r0, r0
	andvs	r5, r0, r0, lsl r0
	.word	0x7000e000
enter_lp0_end:
ENDPROC(enter_lp0)

ENTRY(exit_power_state)
	//Switch to SVC state
	cpsid if, #0x13

	ldr	r5, =PMC_PA_BASE

	//Check which core we are by checking the MPIDR
	mrc	p15, 0, r2, c0, c0, 5
	ands	r2, r2, #0x3
	bne	restore_slave

	//Disable DPD sample
	mov	r3, #0	
	str	r3, [r5, #APBDEV_PMC_DPD_SAMPLE_0]

	//Disable DPD enable	
	str	r3, [r5, #APBDEV_PMC_DPD_ENABLE_0]

restore_slave:

	//Get the physical pointer to cpu context save area
	ldr	r0, [r5, #APBDEV_PMC_SCRATCH37_0]
	mov	r3, #0x800
	//r0 = r0 + r2 * r3
	smlabb	r0, r2, r3, r0

	//Perform ARM restore (r0 = context save ptr)	
	b	ArmCortexA9PhysicalRestore

ArmCortexA9PhysicalRestored:

	//After PhysicalRestore, we are now back in virtual space
	//r0  = power state
	//r11 = pointer to CPU-specific context save area VA

	//Check if power state is POWER_STATE_LP0
	cmp		r0, #2
	bne		skip_pll
	ldr		r0, =g_pPMC
	ldr		r0, [r0]
	ldr		r1, =g_pTimerus
	ldr		r1, [r1]

	//Read from LP0 exit time from SCRATCH1
	ldr		r2, [r0, #0x54]
	add		r2, r2, #300
pll_wait:
	ldr		r3, [r1]
	cmp		r3, r2
	blt		pll_wait

	//Put CPU clock source on PLLX
	ldr		r0, =g_pCLK_RST_CONTROLLER
	ldr		r0, [r0]
	ldr		r1, =0x20008888
	str		r1, [r0, #0x20]

skip_pll:
	//Restore the cpu virtual context
	b	ArmCortexA9VirtualRestore

ArmCortexA9VirtualRestored:

	//Everything should be restored now. Increment active CPUs
	ldr	r1, =g_NumActiveCPUs
	ldr	r2, [r1]
	adds	r2, r2, #1
	str	r2, [r1]
	dmb

	//Check which core we are by checking the MPIDR
	mrc	p15, 0, r2, c0, c0, 5
	ands	r2, r2, #0x3
	bne	skip_local_timer_restore

	//Restore the local timers
	bl	restore_local_timers

skip_local_timer_restore:
	//Restore the stack registers
	ldmfd	sp!, {r0}
	ldmfd	sp!, {r0-r12, lr}

	//Restore the CPSR stored in r2
	msr	CPSR_fsxc, r2
	mov	r0, #1
	bx	lr
ENDPROC(exit_power_state)

ENTRY(ArmCortexA9Save)
	add	r0, r0, #0x44
	str	sp, [r0], #4
	stmia	r0!, {r1 - r12, lr}
	sub	r11, r0, #0x7C
	mrs	r4, cpsr
	mrs	r5, spsr
	stmia	r0!, {r4, r5}
	movs	r4, #0x44
	movs	r5, #0
	movs	r6, #0
loop:
	str	r5, [r11, r6]
	add	r6, r6, #4
	cmp	r6, r4
	bne	loop
	str	r11, [r11, #0x34]
	str	r1, [r11, #0x38]
	mov	r4, r1
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOffDMsave
	mov	r6, #0x1000
	sub	r6, r6, #1
	bic	r5, r4, r6
	//VA to PA translation register in r5
	mcr	p15, 0, r5, c7, c8, 1
	isb
	//r5 = PA register
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	//Translation failed!
	bne	.
	bic	r5, r5, r6
	and	r4, r4, r6
	orr	r4, r4, r5
MmuOffDMsave:
	str	r4, [r11, #0x3C]
	str	r0, [r11, #0x18]
	mrc	p15, 0, r8, c9, c12, 0
	bic	r1, r8, #1
	mcr	p15, 0, r1, c9, c12, 0
	isb
	mrc	p15, 0, r9, c9, c12, 3
	mrc	p15, 0, r10, c9, c12, 5
	stmia	r0!, {r8 - r10}
	ubfx	r9, r8, #11, #5
	tst	r9, r9
	beq	PMonsavecontinue
PMonsaveloop:
	subs	r9, r9, #1
	mcr	p15, 0, r9, c9, c12, 5
	isb
	mrc	p15, 0, r3, c9, c13, 1
	mrc	p15, 0, r4, c9, c13, 2
	stmia	r0!, {r3, r4}
	bne	PMonsaveloop
PMonsavecontinue:
	mrc	p15, 0, r1, c9, c13, 0
	mrc	p15, 0, r2, c9, c14, 0
	mrc	p15, 0, r3, c9, c14, 1
	mrc	p15, 0, r4, c9, c12, 1
	stmia	r0!, {r1 - r4}
	str	r0, [r11, #0x10]
	cps	0x1f	//sys
	str	sp, [r0], #4
	str	lr, [r0], #4
	cps	0x17	//abt
	str	sp, [r0], #4
	mrs	r4, spsr
	stmia	r0!, {r4, lr}
	cps	0x1b	//und	
	str	sp, [r0], #4
	mrs	r4, spsr
	stmia	r0!, {r4, lr}
	cps	0x12	//irq
	str	sp, [r0], #4
	mrs	r4, spsr
	stmia	r0!, {r4, lr}
	cps	0x11	//fiq
	str	sp, [r0], #4
	mrs	r4, spsr
	stmia	r0!, {r4, r8 - r12, lr}
	cps	0x13  //svc
	mrc	p15, 2, r3, c0, c0, 0
	str	r3, [r0], #4
	mrc	p15, 0, r4, c1, c0, 1
	mrc	p15, 0, r5, c1, c0, 0
	mrc	p15, 0, r6, c1, c0, 2
	str	r4, [r11, #4]
	str	r5, [r11, #8]
	str	r6, [r11, #0xC] //VFPSave
	str	r0, [r11, #0x1C]
	mrc	p15, 0, r9, c1, c0, 2
	orr	r2, r9, #0xf00000
	mcr	p15, 0, r2, c1, c0, 2
	isb
	mrc	p15, 0, r2, c1, c0, 2
	and	r2, r2, #0xf00000
	cmp	r2, #0xf00000
	beq	do_fpu_saveVFPsave
	movs	r2, #0
	str	r2, [r11, #0x1C]
	b	exit_fpu_saveVFPsave
do_fpu_saveVFPsave:
	VFPFMRX	r10,FPEXC
	str	r10, [r0], #4
	ldr	r2, =0x40000000
	VFPFMXR	FPEXC,r2
	VFPFMRX	r2,FPSCR
	str	r2, [r0], #4
	VFPFSTMIA r0, r1	@ fstmiad r0!, {d0-d15}.  clobbers r1
	VFPFMRX	r2, MVFR0
	and	r2, r2, #0xF
	cmp	r2, #2
	blt	exit_fpu_saveVFPsave
	//fstmiad	r0!, {d16-d31} //??
	VFPFMXR	FPEXC,r10
exit_fpu_saveVFPsave:
	mcr	p15, 0, r9, c1, c0, 2
	str	r0, [r11, #0x28]
	ldr	r1, [r11, #0x38]
	mov	r6, #1, 24
	add	r1, r1, r6
	ldr	r2, [r1, #4]
	ldr	r3, [r1, #8]
	ldr	r4, [r1]
	stmia	r0!, {r2 - r4}
	mov	r2, #0
	str	r2, [r1, #4]
	mrc	p15, 0, r7, c0, c0, 5
	ubfx	r7, r7, #0, #2
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000
	add	r1, r1, r6
	cmp	r7, #0
	movne	r2, #1
	bne	next1CA9GICsave
	ldr	r2, [r1, #4]
	ubfx	r2, r2, #0, #5
	add	r2, r2, #1
next1CA9GICsave:
	mov	r3, r2
loop1CA9GICsave:
	ldr	r5, [r1, #0x100]
	str	r5, [r0], #4
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop1CA9GICsave
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000	//
	add	r1, r1, r6
	mov	r3, r2, lsl #3
loop2CA9GICsave:
	ldr	r4, [r1, #0x400]
	ldr	r5, [r1, #0x800]
	stmia	r0!, {r4, r5}
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop2CA9GICsave
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000	//
	add	r1, r1, r6
	mov	r3, r2, lsl #1
loop3CA9GICsave:
	ldr	r4, [r1, #0xC00]
	str	r4, [r0], #4
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop3CA9GICsave
	cmp	r7, #0
	bne	continueCA9GICsave
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000	//
	add	r1, r1, r6
	ldr	r2, [r1]	//distributor control
	str	r2, [r0], #4
continueCA9GICsave:
	mov	r4, r0
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOffDMmmu
	mov	r6, #0x1000	//
	sub	r6, r6, #1
	bic	r5, r4, r6
	mcr	p15, 0, r5, c7, c8, 1
	isb
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	bne	.
	bic	r5, r5, r6
	and	r4, r4, r6
	orr	r4, r4, r5
MmuOffDMmmu:
	str	r4, [r11, #0x20]
	mrc	p15, 0, r5, c2, c0, 0
	mrc	p15, 0, r6, c2, c0, 1
	mrc	p15, 0, r7, c2, c0, 2
	stmia	r0!, {r5 - r7}
	mrc	p15, 0, r4, c3, c0, 0
	mrc	p15, 0, r5, c7, c4, 0
	mrc	p15, 0, r6, c10, c2, 0
	mrc	p15, 0, r7, c10, c2, 1
	stmia	r0!, {r4 - r7}
	mrc	p15, 0, r4, c12, c0, 0
	str	r4, [r0], #4
	mrc	p15, 0, r4, c13, c0, 1
	mrc	p15, 0, r5, c13, c0, 2
	mrc	p15, 0, r6, c13, c0, 3
	mrc	p15, 0, r7, c13, c0, 4
	stmia	r0!, {r4 - r7}
	//Disable the D-Cache
	mrc	p15, 0, r4, c1, c0, 0
	bic	r4, r4, #4
	mcr	p15, 0, r4, c1, c0, 0
	isb
	mov	r1, #0
	mcr	p15, 2, r1, c0, c0, 0
	isb
	mrc	p15, 1, r7, c0, c0, 0
	ubfx	r3, r7, #0, #3
	add	r3, r3, #4
	ubfx	r4, r7, #13, #15
	ubfx	r5, r7, #3, #10
	cmp	r5, #3
	cmpeq	r3, #5
	bne	PWRDNcleangeneric
	mov	r1, r4, lsl #5
	add	r2, r1, #0x40000000
	add	r3, r1, #0x80000000
	add	r4, r1, #0xc0000000
PWRDNcleanoptloop:
	mcr	p15, 0, r1, c7, c14, 2
	mcr	p15, 0, r2, c7, c14, 2
	mcr	p15, 0, r3, c7, c14, 2
	mcr	p15, 0, r4, c7, c14, 2
	subs	r1, r1, #0x20
	sub	r2, r2, #0x20
	sub	r3, r3, #0x20
	sub	r4, r4, #0x20
	bpl	PWRDNcleanoptloop
	b	PWRDNcleanend
PWRDNcleangeneric:
	clz	r6, r5
PWRDNcleanloopl:
	mov	r2, r5
PWRDNcleanloop2:
	mov	r7, r2, lsl r6
	mov	r1, r4, lsl r3
	orr	r7, r7, r1
	mcr	p15, 0, r7, c7, c14, 2
	subs	r2, r2, #1
	bge	PWRDNcleanloop2
	subs	r4, r4, #1
	bge	PWRDNcleanloopl
PWRDNcleanend:
	dsb
	mrc	p15, 0, r4, c1, c0, 1
	bic	r4, r4, #0x40
	mcr	p15, 0, r4, c1, c0, 1
	isb
	mrc	p15, 0, r7, c0, c0, 5
	ubfx	r7, r7, #0, #2
	cmp	r7, #0
	bne	NotCPU0save
	mov	r4, r0
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOffDMscu
	mov	r6, #0x1000
	sub	r6, r6, #1
	bic	r5, r4, r6
	mcr	p15, 0, r5, c7, c8, 1
	isb
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	bne	.
	bic	r5, r5, r6
	and	r4, r4, r6
	orr	r4, r4, r5
MmuOffDMscu:
	str	r4, [r11, #0x24]
	ldr	r1, [r11, #0x38]
	ldr	r2, [r1, #0x40]
	ldr	r3, [r1, #0x44]
	ldr	r4, [r1, #0x50]
	ldr	r5, [r1, #0x54]
	ldr	r6, [r1]
	stmia	r0!, {r2 - r6}
NotCPU0save:
	mov	r4, r0
DMpl310:
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOff
	mov	r6, #0x1000
	sub	r6, r6, #1
	bic	r5, r4, r6
	mcr	p15, 0, r5, c7, c8, 1
	isb
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	bne	.
	bic	r5, r5, r6
	and	r4, r4, r6
	orr	r4, r4, r5
MmuOff:
	str	r4, [r11, #0x30]
	ldr	r1, [r11, #0x38]
	mov	r2, #0x3200	//
	add	r1, r1, r2
	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	ldr	r4, [r1, #8]
	ldr	r5, [r1, #0xC]
	ldr	r6, [r1, #0x10]
	ldr	r7, [r1, #0x14]
	stmia	r0!, {r2 - r7}
	add	r1, r1, #0x700
	ldr	r2, [r1], #4
	ldr	r3, [r1], #4
	ldr	r4, [r1], #4
	ldr	r5, [r1], #4
	stmia	r0!, {r2 - r5}
	ldr	r2, [r1], #4
	ldr	r3, [r1], #4
	ldr	r4, [r1], #4
	ldr	r5, [r1], #4
	stmia	r0!, {r2 - r5}
	ldr	r2, [r1], #4
	ldr	r3, [r1], #4
	ldr	r4, [r1], #4
	ldr	r5, [r1], #4
	stmia	r0!, {r2 - r5}
	ldr	r2, [r1], #4
	ldr	r3, [r1], #4
	ldr	r4, [r1], #4
	ldr	r5, [r1], #4
	stmia	r0!, {r2 - r5}
	add	r1, r1, #0x10
	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	stmia	r0!, {r2, r3}
	add	r1, r1, #0x2b0
	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	stmia	r0!, {r2, r3}
	ldr	r2, [r1, #0x340]
	str	r2, [r0], #4
	sub	r1, r1, #0xB00
	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	ldr	r4, [r1, #8]
	ldr	r5, [r1, #0xC]
	stmia	r0!, {r2 - r5}
CheckWayOperationsSYSCACHEsave:
	ldr	r2, [r1, #0x6FC]
	cmp	r2, #0
	bne	CheckWayOperationsSYSCACHEsave
CheckUnlockOperationSYSCACHEsave:
	ldr	r2, [r1, #0x854]
	cmp	r2, #0
	bne	CheckUnlockOperationSYSCACHEsave
	ldr	r4, [r11, #0x20]
	mov	r2, #0x1F
	bic	r4, r4, r2
	ldr	r1, [r11, #0x38]
	mov	r2, #0x3700
	add	r1, r1, r2
	mov	r3, r0
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOffDMsyscache1
	mov	r6, #0x1000
	sub	r6, r6, #1
	bic	r5, r3, r6
	mcr	p15, 0, r5, c7, c8, 1
	isb
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	bne	.
	bic	r5, r5, r6
	and	r3, r3, r6
	orr	r3, r3, r5
MmuOffDMsyscache1:
	str	r4, [r1, #0xB0]
	add	r4, r4, #0x20
	cmp	r4, r3
	blt	MmuOffDMsyscache1
	mov	r4, r11
	mrc	p15, 0, r5, c1, c0, 0
	tst	r5, #1
	beq	MmuOffDMsyscache2
	mov	r6, #0x1000
	sub	r6, r6, #1
	bic	r5, r4, r6
	mcr	p15, 0, r5, c7, c8, 1
	isb
	mrc	p15, 0, r5, c7, c4, 0
	tst	r5, #1
	bne	.
	bic	r5, r5, r6
	and	r4, r4, r6
	orr	r4, r4, r5
MmuOffDMsyscache2:
	mov	r2, #0x44
	add	r2, r2, r4
SYSCACHEclean2:
	str	r4, [r1, #0xB0]
	add	r4, r4, #0x20
	cmp	r4, r2
	blt	SYSCACHEclean2
NotCPU0savex:
	str	r0, [r11, #0x14]
_lc010_006156_:	
	mrc	p14, 0, r1, c0, c1, 0
	str	r1, [r0], #4
	mrc	p14, 0, r1, c0, c6, 0
	mrc	p14, 0, r2, c0, c7, 0
	mrc	p14, 0, r3, c7, c9, 6
	stmia	r0!, {r1 - r3}
	mrc	p14, 0, r2, c0, c0, 4
	mrc	p14, 0, r3, c0, c0, 5
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c1, 4
	mrc	p14, 0, r3, c0, c1, 5
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c2, 4
	mrc	p14, 0, r3, c0, c2, 5
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c3, 4
	mrc	p14, 0, r3, c0, c3, 5
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c4, 4
	mrc	p14, 0, r3, c0, c4, 5
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c5, 4
	mrc	p14, 0, r3, c0, c5, 5
	stmia	r0!, {r2, r3}
_lc025_006469_:
	mrc	p14, 0, r2, c0, c0, 6
	mrc	p14, 0, r3, c0, c0, 7
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c1, 6
	mrc	p14, 0, r3, c0, c1, 7
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c2, 6
	mrc	p14, 0, r3, c0, c2, 7
	stmia	r0!, {r2, r3}
	mrc	p14, 0, r2, c0, c3, 6
	mrc	p14, 0, r3, c0, c3, 7
	stmia	r0!, {r2, r3}
_lc099_006768_:
	mov	r0, r11
	b	ArmCortexA9Saved
ENDPROC(ArmCortexA9Save)

ENTRY(ArmCortexA9PhysicalRestore)
	cpsid	aif, #0x13	//
	mov	r11, r0
	mcr	p15, 0, r0, c7, c5, 0
	mcr	p15, 0, r0, c7, c5, 6
	mov	r1, #0x1800	//
	mcr	p15, 0, r1, c1, c0, 0
	isb
	mov	r1, #0
	mcr	p15, 2, r1, c0, c0, 0
	isb
	mrc	p15, 1, r7, c0, c0, 0
	ubfx	r3, r7, #0, #3
	add	r3, r3, #4
	ubfx	r4, r7, #13, #15
	ubfx	r5, r7, #3, #10
	cmp	r5, #3
	cmpeq	r3, #5	
	bne	PWRUPinvgeneric
	mov	r1, r4, lsl #5
	add	r2, r1, #0x40000000
	add	r3, r1, #0x80000000
	add	r4, r1, #0xc0000000
PWRUPinvoptloop:
	mcr	p15, 0, r1, c7, c6, 2
	mcr	p15, 0, r2, c7, c6, 2
	mcr	p15, 0, r3, c7, c6, 2
	mcr	p15, 0, r4, c7, c6, 2
	subs	r1, r1, #0x20
	sub	r2, r2, #0x20
	sub	r3, r3, #0x20
	sub	r4, r4, #0x20
	bpl	PWRUPinvoptloop
	b	PWRUPinvend
PWRUPinvgeneric:
	clz	r6, r5
PWRUPinvloopl:
	mov	r2, r5
PWRUPinvloop2:
	mov	r7, r2, lsl r6
	mov	r1, r4, lsl r3
	orr	r7, r7, r1
	mcr	p15, 0, r7, c7, c6, 2
	subs	r2, r2, #1
	bge	PWRUPinvloop2
	subs	r4, r4, #1
	bge	PWRUPinvloopl
PWRUPinvend:
	dsb
	mrc	p15, 0, r7, c0, c0, 5
	ubfx	r7, r7, #0, #2

	cmp	r7, #0
	bne	NotCPU0restore
CPU0restore:
	ldr	r0, [r11, #0x24]
	cmp	r0, #0
	beq	CA9SCUrestorecontinue
	ldr	r1, [r11, #0x3C]
	ldmia	r0!, {r2 - r6}
	str	r2, [r1, #0x40]
	str	r3, [r1, #0x44]
	str	r4, [r1, #0x50]
	str	r5, [r1, #0x54]
	mov	r7, #0x10000
	sub	r7, r7, #1
	str	r7, [r1, #0xC]
	str	r6, [r1]
CA9SCUrestorecontinue:
	ldr	r0, [r11, #0x30]
	cmp	r0, #0
	beq	SYSCACHErestorecontinue
	ldr	r1, [r11, #0x3C]
	mov	r2, #0x3100
	add	r1, r1, r2
	movs	r2, #0
SYSCACHErestorel2_cache_disable_loop:
	ldr	r3, [r1]
	and	r3, r3, #1
	cmp	r3, #0
	beq	SYSCACHErestorel2_cache_disable_skip
	str	r2, [r1]
	dsb
	b	SYSCACHErestorel2_cache_disable_loop
SYSCACHErestorel2_cache_disable_skip:
	ldr	r0, [r11, #0x30]
	ldmia	r0!, {r2 - r7}
	add	r1, r1, #0x100
	str	r2, [r1]
	str	r3, [r1, #4]
	str	r4, [r1, #8]
	str	r5, [r1, #0xC]
	str	r6, [r1, #0x10]
	str	r7, [r1, #0x14]
	add	r1, r1, #0x700
	ldmia	r0!, {r2 - r5}
	str	r2, [r1], #4
	str	r3, [r1], #4
	str	r4, [r1], #4
	str	r5, [r1], #4
	ldmia	r0!, {r2 - r5}
	str	r2, [r1], #4
	str	r3, [r1], #4
	str	r4, [r1], #4
	str	r5, [r1], #4
	ldmia	r0!, {r2 - r5}
	str	r2, [r1], #4
	str	r3, [r1], #4
	str	r4, [r1], #4
	str	r5, [r1], #4
	ldmia	r0!, {r2 - r5}
	str	r2, [r1], #4
	str	r3, [r1], #4
	str	r4, [r1], #4
	str	r5, [r1], #4
	add	r1, r1, #0x10
	ldmia	r0!, {r2, r3}
	str	r2, [r1]
	str	r3, [r1, #4]
	add	r1, r1, #0x2B0
	ldmia	r0!, {r2, r3}
	str	r2, [r1]
	str	r3, [r1, #4]
	ldr	r2, [r0], #4
	str	r2, [r1, #0x340]
	sub	r1, r1, #0xB00
	ldmia	r0!, {r2 - r5}
	str	r3, [r1, #4]
	str	r4, [r1, #8]
	str	r5, [r1, #0xC]
	str	r2, [r1]
	dsb
SYSCACHErestorecontinue:	
NotCPU0restore:
	ldr	r0, [r11, #0x20]
	ldmia	r0!, {r5 - r7}
	mcr	p15, 0, r5, c2, c0, 0
	mcr	p15, 0, r6, c2, c0, 1
	mcr	p15, 0, r7, c2, c0, 2
	ldmia	r0!, {r4 - r7}
	mcr	p15, 0, r4, c3, c0, 0
	mcr	p15, 0, r5, c7, c4, 0
	mcr	p15, 0, r6, c10, c2, 0
	mcr	p15, 0, r7, c10, c2, 1
	ldr	r4, [r0], #4
	mcr	p15, 0, r4, c12, c0, 0
	ldmia	r0!, {r4 - r7}
	mcr	p15, 0, r4, c13, c0, 1
	mcr	p15, 0, r5, c13, c0, 2
	mcr	p15, 0, r6, c13, c0, 3
	mcr	p15, 0, r7, c13, c0, 4
	ldr	r11, [r11, #0x34]
	ldr	lr, =VirtualRestore	
	mov	r1, #0
	mcr	p15, 0, r1, c8, c7, 0
	isb
	mov	r2, #0
	orr	r2, #(1<<11)  //Z bit on
	orr	r2, #(1<<12)  //I cache on
	orr	r2, #(1<<28)  //Enable Tex Remap
	orr	r2, #(1<<0)	//MMU on
	andeq	r0, r0, r0
	andeq	r0, r0, r0
	andeq	r0, r0, r0
	andeq	r0, r0, r0
.align 5	
	mcr	p15, 0, r2, c1, c0, 0
	bx	lr
	mov	r0, r0
	mov	r0, r0
	mov	r0, r0
	mov	r0, r0
VirtualRestore:
	mcr	p15, 0, r1, c8, c7, 0
	dsb
	isb
	add	r0, r11, #0x44
	ldr	r0, [r0]
	ldr	r0, [r0]
	b	ArmCortexA9PhysicalRestored
	andeq	r1, r0, r1, lsl #16
	andeq	r0, r0, r0
.ltorg
	
ENDPROC(ArmCortexA9PhysicalRestore)
	
ENTRY(ArmCortexA9VirtualRestore)	
	mrc	p15, 0, r7, c0, c0, 5
	tst	r7, r7
	bpl	CA9GICrestorecontinue
	ubfx	r7, r7, #0, #2
	ldr	r0, [r11, #0x28]
	cmp	r0, #0
	beq	CA9GICrestorecontinue
	ldr	r1, [r11, #0x38]
	mov	r6, #1, 24
	add	r1, r1, r6
	ldmia	r0!, {r2 - r4}
	str	r2, [r1, #4]
	str	r3, [r1, #8]
	str	r4, [r1]
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000
	add	r1, r1, r6
	cmp	r7, #0
	movne	r2, #1
	bne	next1CA9GICrestore
	ldr	r2, [r1, #4]
	ubfx	r2, r2, #0, #5
	add	r2, r2, #1
next1CA9GICrestore:
	mov	r3, r2
loop1CA9GICrestore:
	ldr	r5, [r0], #4
	str	r5, [r1, #0x100]
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop1CA9GICrestore
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000
	add	r1, r1, r6
	mov	r3, r2, lsl #3
loop2CA9GICrestore:
	ldmia	r0!, {r4, r5}
	str	r4, [r1, #0x400]
	str	r5, [r1, #0x800]
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop2CA9GICrestore
	ldr	r1, [r11, #0x38]
	mov	r6, #0x1000 //
	add	r1, r1, r6
	mov	r3, r2, lsl #1
loop3CA9GICrestore:
	ldr	r4, [r0], #4
	str	r4, [r1, #0xC00]
	add	r1, r1, #4
	subs	r3, r3, #1
	bne	loop3CA9GICrestore
	cmp	r7, #0
	bne	CA9GICrestorecontinue
	ldr	r1, [r11, #0x38]
	mov	r6, #1, 20
	add	r1, r1, r6
	ldr	r2, [r0], #4
	str	r2, [r1]
CA9GICrestorecontinue:
	ldr	r0, [r11, #0x1C]
	cmp	r0, #0
	beq	exit_fpu_restoreVFPrestore
	mrc	p15, 0, r2, c1, c0, 2
	orr	r2, r2, #0x00F00000
	mcr	p15, 0, r2, c1, c0, 2
	ldr	r2, =0x40000000
	VFPFMXR	FPEXC,r2
	ldmia	r0!, {r9, r10}
	VFPFLDMIA r0, r4	@ r4 clobbered (immediately loaded in exit_fpu_re)
	VFPFMRX	r2, MVFR0
	and	r2, r2, #0xF
	cmp	r2, #2
	blt	complete_fpu_restoreVFPrestore
	//fldmiad	r0!, {d16-d31} //??
complete_fpu_restoreVFPrestore:
	VFPFMXR	FPSCR,r10
	VFPFMXR	FPEXC,r9
exit_fpu_restoreVFPrestore:
	ldr	r0, [r11, #0x10]
	cps	#0x1f	//sys
	ldr	sp, [r0], #4
	ldr	lr, [r0], #4
	cps	#0x17	//abt
	ldr	sp, [r0], #4
	ldmia	r0!, {r4, lr}
	msr	spsr_cxsf, r4
	cps	#0x1b	//und
	ldr	sp, [r0], #4
	ldmia	r0!, {r4, lr}
	msr	spsr_cxsf, r4
	cps	#0x12	//irq
	ldr	sp, [r0], #4
	ldmia	r0!, {r4, lr}
	msr	spsr_cxsf, r4
	cps	0x11	//fiq
	ldr	sp, [r0], #4
	ldmia	r0!, {r4, r8 - r12, lr}
	msr	spsr_cxsf, r4
	cps	0x13	//svc
	ldr	r3, [r0], #4
	mcr	p15, 2, r3, c0, c0, 0
	ldr	r4, [r11, #4]
	ldr	r5, [r11, #8]
	mcr	p15, 0, r4, c1, c0, 1
	isb
	mcr	p15, 0, r5, c1, c0, 0
	isb
	ldr	r6, [r11, #0xC]
	mcr	p15, 0, r6, c1, c0, 2
	isb 
	ldr	r0, [r11, #0x14]
	cmp	r0, #0
	beq	SkipDbgRestore
DBGrestore:
	ldr	r5, [r0], #4
	ldmia	r0!, {r1 - r3}
	mcr	p14, 0, r1, c0, c6, 0
	mcr	p14, 0, r2, c0, c7, 0
	mcr	p14, 0, r3, c7, c8, 6
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c0, 4
	mcr	p14, 0, r3, c0, c0, 5
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c1, 4
	mcr	p14, 0, r3, c0, c1, 5
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c2, 4
	mcr	p14, 0, r3, c0, c2, 5
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c3, 4
	mcr	p14, 0, r3, c0, c3, 5
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c4, 4
	mcr	p14, 0, r3, c0, c4, 5
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c5, 4
	mcr	p14, 0, r3, c0, c5, 5
_lc025_008056_:
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c0, 6
	mcr	p14, 0, r3, c0, c0, 7
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c1, 6
	mcr	p14, 0, r3, c0, c1, 7
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c2, 6
	mcr	p14, 0, r3, c0, c2, 7
	ldmia	r0!, {r2, r3}
	mcr	p14, 0, r2, c0, c3, 6
	mcr	p14, 0, r3, c0, c3, 7
_lc040_008312_:
	isb
	mcr	p14, 0, r5, c0, c2, 2
_lc099_008377_:
	isb
SkipDbgRestore:
	ldr	r0, [r11, #0x18]
	ldmia	r0!, {r8 - r10}
	mov	r1, #0
	mvn	r2, #0
	mcr	p15, 0, r2, c9, c14, 2
	mcr	p15, 0, r2, c9, c12, 3
	isb
	ubfx	r12, r8, #11, #5
	tst	r12, r12
	beq	PMonrestorecontinue
	mov	r3, r12
	mov	r4, #1
	mov	r4, r4, lsl r3
	sub	r4, r4, #1
PMonrestoreloop1:
	subs	r3, r3, #1
	mcr	p15, 0, r3, c9, c12, 5
	isb
	mrc	p15, 0, r5, c9, c13, 1
	bfc	r5, #0, #8
	mcr	p15, 0, r5, c9, c13, 1
	mcr	p15, 0, r2, c9, c13, 2
	isb
	bne	PMonrestoreloop1
	mov	r3, #1
	bic	r5, r9, #1<<31
	mcr	p15, 0, r5, c9, c12, 1
	mcr	p15, 0, r3, c9, c12, 0
	isb
	mcr	p15, 0, r9, c9, c12, 4
	isb
	mcr	p15, 0, r4, c9, c12, 2
PMonrestoreloop2:
	subs	r12, r12, #1
	mcr	p15, 0, r12, c9, c12, 5
	isb
	ldmia	r0!, {r3, r4}
	mcr	p15, 0, r3, c9, c13, 1
	mcr	p15, 0, r4, c9, c13, 2
	isb
	bne	PMonrestoreloop2
PMonrestorecontinue:
	tst	r9, #0x80000000
	beq	PMonrestorecontinue2
	mcr	p15, 0, r2, c9, c13, 0
	isb
	mov	r3, #0x80000000
	mcr	p15, 0, r3, c9, c12, 1
	isb
PMonrestoreloop3:
	mrc	p15, 0, r4, c9, c12, 3
	movs	r4, r4
	bpl	PMonrestoreloop3
	mcr	p15, 0, r3, c9, c12, 2
PMonrestorecontinue2:
	mcr	p15, 0, r1, c9, c12, 0
	isb
	ldmia	r0!, {r1 - r4}
	mcr	p15, 0, r1, c9, c13, 0
	mcr	p15, 0, r2, c9, c14, 0
	mcr	p15, 0, r3, c9, c14, 1
	mcr	p15, 0, r4, c9, c12, 1
	mcr	p15, 0, r10, c9, c12, 5
	isb
	mcr	p15, 0, r8, c9, c12, 0
	isb
	add	r0, r11, #0x7C
	ldmia	r0, {r1, r2}
	msr	cpsr_cxsf, r1
	msr	spsr_cxsf, r2
	sub	r0, r0, #0x38
	ldr	sp, [r0], #4
	ldmia	r0, {r1 - r12, lr}
	sub	r0, r0, #0x48
	b	ArmCortexA9VirtualRestored
.ltorg
ENDPROC(ArmCortexA9VirtualRestore)
