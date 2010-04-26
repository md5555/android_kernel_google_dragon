/* linux/arch/arm/plat-s5p/sleep.S
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 power Manager (Suspend-To-RAM) support
 * Based on S3C2410 sleep code by:
 * 	Ben Dooks, (c) 2004 Simtec Electronics
 *
 * Based on PXA/SA1100 sleep code by:
 *	Nicolas Pitre, (c) 2002 Monta Vista Software Inc
 *	Cliff Brake, (c) 2001
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <asm/asm-offsets.h>
#include <asm/memory.h>
#include <asm/system.h>

#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-mem.h>
#include <plat/regs-serial.h>

/* CONFIG_DEBUG_RESUME is dangerous if your bootloader does not
 * reset the UART configuration, only enable if you really need this!
*/
	.text

	/* s3c_cpu_save
	 *
	 * entry:
	 *	r0 = save address (virtual addr of s3c_sleep_save_phys)
	*/

ENTRY(s3c_cpu_save)

	stmfd	sp!, { r3 - r12, lr }

	mrc	p15, 0, r4, c13, c0, 0	@ FCSE/PID
	mrc	p15, 0, r5, c3, c0, 0	@ Domain ID
	mrc	p15, 0, r6, c2, c0, 0	@ Translation Table BASE0
	mrc	p15, 0, r7, c2, c0, 1	@ Translation Table BASE1
	mrc	p15, 0, r8, c2, c0, 2	@ Translation Table Control
	mrc	p15, 0, r9, c1, c0, 0	@ Control register
	mrc	p15, 0, r10, c1, c0, 1	@ Auxiliary control register
	mrc	p15, 0, r11, c1, c0, 2	@ Co-processor access controls
	mrc	p15, 0, r12, c10, c2, 0	@ Read PRRR
	mrc	p15, 0, r3, c10, c2, 1	@ READ NMRR

	stmia	r0, { r3 - r13 }

	@@ write our state back to RAM
	bl	s3c_pm_cb_flushcache

	@@ jump to final code to send system to sleep
	ldr	r0, =pm_cpu_sleep
	@@ldr	pc, [ r0 ]
	ldr	r0, [ r0 ]
	mov	pc, r0

	@@ return to the caller, after having the MMU
	@@ turned on, this restores the last bits from the
	@@ stack
resume_with_mmu:
	/* delete added mmu table list */
	ldr	r9 , =(PAGE_OFFSET - PHYS_OFFSET)
	add	r4, r4, r9
	str	r12, [r4]

	ldmfd	sp!, { r3 - r12, pc }

	.ltorg

	@@ the next bits sit in the .data segment, even though they
	@@ happen to be code... the s5pv210_sleep_save_phys needs to be
	@@ accessed by the resume code before it can restore the MMU.
	@@ This means that the variable has to be close enough for the
	@@ code to read it... since the .text segment needs to be RO,
	@@ the data segment can be the only place to put this code.

	.data

	.global	s3c_sleep_save_phys
s3c_sleep_save_phys:
	.word	0


	/* sleep magic, to allow the bootloader to check for an valid
	 * image to resume to. Must be the first word before the
	 * s5pv210_cpu_resume entry.
	*/

	.word	0x2bedf00d

	/* s3c_cpu_resume
	 *
	 * resume code entry for bootloader to call
	 *
	 * we must put this code here in the data segment as we have no
	 * other way of restoring the stack pointer after sleep, and we
	 * must not write to the code segment (code is read-only)
	*/

ENTRY(s3c_cpu_resume)
	mov	r0, #PSR_I_BIT | PSR_F_BIT | SVC_MODE
	msr	cpsr_c, r0

	@@ load UART to allow us to print the two characters for
	@@ resume debug

	mov	r1, #0
	mcr	p15, 0, r1, c8, c7, 0		@@ invalidate TLBs
	mcr	p15, 0, r1, c7, c5, 0		@@ invalidate I Cache

	ldr	r0, s3c_sleep_save_phys	@ address of restore block
	ldmia	r0, { r3 - r13 }

	mcr	p15, 0, r4, c13, c0, 0	@ FCSE/PID
	mcr	p15, 0, r5, c3, c0, 0	@ Domain ID

	mcr	p15, 0, r8, c2, c0, 2	@ Translation Table Control
	mcr	p15, 0, r7, c2, c0, 1	@ Translation Table BASE1
	mcr	p15, 0, r6, c2, c0, 0	@ Translation Table BASE0

	mcr	p15, 0, r10, c1, c0, 1	@ Auxiliary control register

	mov	r0, #0
	mcr	p15, 0, r0, c8, c7, 0	@ Invalidate I & D TLB

	mov	r0, #0			@ restore copro access controls
	mcr	p15, 0, r11, c1, c0, 2	@ Co-processor access controls
	mcr 	p15, 0, r0, c7, c5, 4

	mcr	p15, 0, r12, c10, c2, 0	@ write PRRR
	mcr	p15, 0, r3, c10, c2, 1	@ write NMRR

	/* calculate first section address into r8 */
	mov	r4, r6
	ldr	r5, =0x3fff
	bic	r4, r4, r5
	ldr	r11, =0xe010f000
	ldr	r10, [r11, #0]
	mov	r10, r10 ,LSR #18
	bic	r10, r10, #0x3
	orr	r4, r4, r10

	/* calculate mmu list value into r9 */
	mov 	r10, r10, LSL #18
	ldr	r5, =0x40e
	orr	r10, r10, r5

	/* back up originally data */

	ldr	r12, [r4]

	/* Added list about mmu */
	str	r10, [r4]

	ldr	r2, =resume_with_mmu
	mcr	p15, 0, r9, c1, c0, 0		@ turn on MMU, etc

        nop
        nop
        nop
        nop
        nop					@ second-to-last before mmu

	mov	pc, r2				@ go back to virtual address

	.ltorg
