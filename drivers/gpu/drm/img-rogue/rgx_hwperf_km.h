/*************************************************************************/ /*!
@File
@Title          RGX HWPerf Types and Defines Header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description	Common data types definitions for hardware performance API
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/
#ifndef RGX_HWPERF_KM_H_
#define RGX_HWPERF_KM_H_

/* 
 * This header file holds the HWPerf related macros and types needed by the
 * code in the Kernel Mode (KM) server/driver module and its content is
 * intended to be suitable for distribution under a public software license.
 * The definitions within are common and may be used in user-mode, kernel-mode
 * and firmware compilation units.
 */
 
#if defined (__cplusplus)
extern "C" {
#endif

#define RGX_HWPERF_V2_FORMAT 2

#include "rgx_common.h"


/*! The number of indirectly addressable TPU_MSC blocks in the GPU */
#define RGX_HWPERF_PHANTOM_INDIRECT_BY_DUST MAX((RGX_FEATURE_NUM_CLUSTERS>>1),1)

/*! The number of indirectly addressable USC blocks in the GPU */
#define RGX_HWPERF_PHANTOM_INDIRECT_BY_CLUSTER (RGX_FEATURE_NUM_CLUSTERS)

#if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE)

 /*! Defines the number of performance counter blocks that are directly
  * addressable in the RGX register map. */
 #define RGX_HWPERF_MAX_DIRECT_ADDR_BLKS 1 /* JONES */
 #define RGX_HWPERF_INDIRECT_BY_PHANTOM (RGX_NUM_PHANTOMS)
 #define RGX_HWPERF_PHANTOM_NONDUST_BLKS 1 /* BLACKPEARL */
 #define RGX_HWPERF_PHANTOM_DUST_BLKS 2 /* TPU, TEXAS */
 #define RGX_HWPERF_PHANTOM_DUST_CLUSTER_BLKS 2 /* USC, PBE */

#elif defined(RGX_FEATURE_XT_TOP_INFRASTRUCTURE)

 #if defined(RGX_FEATURE_RAY_TRACING)
  /*! Defines the number of performance counter blocks that are directly
   * addressable in the RGX register map. */
  #define RGX_HWPERF_MAX_DIRECT_ADDR_BLKS 7 /* TORNADO, TA, BF, BT, RT, BX_TU, SH */
 #else /*#if defined(RAY_TRACING) */
  /*! Defines the number of performance counter blocks that are directly
   * addressable in the RGX register map. */
  #define RGX_HWPERF_MAX_DIRECT_ADDR_BLKS 2 /* TORNADO, TA */
 #endif /*#if defined(RAY_TRACING) */

 #define	RGX_HWPERF_INDIRECT_BY_PHANTOM (RGX_NUM_PHANTOMS)
 #define RGX_HWPERF_PHANTOM_NONDUST_BLKS 2 /* RASTER, TEXAS */
 #define	RGX_HWPERF_PHANTOM_DUST_BLKS 1 /* TPU */
 #define	RGX_HWPERF_PHANTOM_DUST_CLUSTER_BLKS 1 /* USC */

#else	/* if defined(RGX_FEATURE_S7_TOP_INFRASTRUCTURE) */

 /*! Defines the number of performance counter blocks that are directly
  * addressable in the RGX register map. */
 #define RGX_HWPERF_MAX_DIRECT_ADDR_BLKS 3 /* TA, RASTER, HUB */
 #define	RGX_HWPERF_INDIRECT_BY_PHANTOM 0  /* PHANTOM is not there is Rogue1. Just using it to keep nameing same as later series (RogueXT n Rogue XT+) */
 #define	RGX_HWPERF_PHANTOM_NONDUST_BLKS 0
 #define RGX_HWPERF_PHANTOM_DUST_BLKS 1 /* TPU */
 #define RGX_HWPERF_PHANTOM_DUST_CLUSTER_BLKS 1 /* USC */

#endif

/*! The number of indirect addressable layout blocks in the GPU with performance counters */
#define RGX_HWPERF_MAX_INDIRECT_ADDR_BLKS 	(RGX_HWPERF_PHANTOM_DUST_BLKS * RGX_HWPERF_PHANTOM_INDIRECT_BY_DUST +\
											RGX_HWPERF_PHANTOM_DUST_CLUSTER_BLKS * RGX_HWPERF_PHANTOM_INDIRECT_BY_CLUSTER +\
											RGX_HWPERF_PHANTOM_NONDUST_BLKS      * RGX_HWPERF_INDIRECT_BY_PHANTOM)

/*! The number of custom non-mux counter blocks supported */
#define RGX_HWPERF_MAX_CUSTOM_BLKS 4

/*! The number of counters supported in each non-mux counter block */
#define RGX_HWPERF_MAX_CUSTOM_CNTRS 8

/******************************************************************************
 * 	Data Stream Common Types
 *****************************************************************************/

/* These structures are used on both GPU and CPU and must be a size that is a
 * multiple of 64 bits, 8 bytes to allow the FW to write 8 byte quantities
 * at 8 byte aligned addresses.  RGX_FW_STRUCT_*_ASSERT() is used to check this.
 */
 
/*! Type used to encode the event that generated the HW performance packet.
 * NOTE: When this type is updated the corresponding hwperfbin2json tool source
 * needs to be updated as well. Also need to update the table in rgxhwperf.c.
 * The RGX_HWPERF_EVENT_MASK_* macros will also need updating when adding new
 * types.
 */
typedef enum
{
	RGX_HWPERF_INVALID				= 0x00,
	/* FW types 0x01..0x07 */
	RGX_HWPERF_FW_BGSTART			= 0x01,
	RGX_HWPERF_FW_BGEND				= 0x02,
	RGX_HWPERF_FW_IRQSTART			= 0x03,

	RGX_HWPERF_FW_IRQEND			= 0x04,
	RGX_HWPERF_FW_DBGSTART			= 0x05,
	RGX_HWPERF_FW_DBGEND			= 0x06,

	/* HW types 0x08..0x18 */
	RGX_HWPERF_HW_TAKICK			= 0x08,
	RGX_HWPERF_HW_TAFINISHED		= 0x09,
	RGX_HWPERF_HW_3DTQKICK			= 0x0A,
/*	RGX_HWPERF_HW_3DTQFINISHED		= 0x17, */
/*	RGX_HWPERF_HW_3DSPMKICK			= 0x11, */
/*	RGX_HWPERF_HW_3DSPMFINISHED		= 0x18, */
	RGX_HWPERF_HW_3DKICK			= 0x0B,

	RGX_HWPERF_HW_3DFINISHED		= 0x0C,
	RGX_HWPERF_HW_CDMKICK			= 0x0D,
	RGX_HWPERF_HW_CDMFINISHED		= 0x0E,
	RGX_HWPERF_HW_TLAKICK			= 0x0F,

	RGX_HWPERF_HW_TLAFINISHED		= 0x10,
	RGX_HWPERF_HW_3DSPMKICK			= 0x11,
	RGX_HWPERF_HW_PERIODIC			= 0x12,
	RGX_HWPERF_HW_RTUKICK			= 0x13,
	
	RGX_HWPERF_HW_RTUFINISHED		= 0x14,
	RGX_HWPERF_HW_SHGKICK			= 0x15,
	RGX_HWPERF_HW_SHGFINISHED		= 0x16,
	RGX_HWPERF_HW_3DTQFINISHED		= 0x17,

	RGX_HWPERF_HW_3DSPMFINISHED		= 0x18,

	/* other types 0x1A..0x1F */
	RGX_HWPERF_CLKS_CHG				= 0x1A,
	RGX_HWPERF_GPU_STATE_CHG		= 0x1B,

	/* power types 0x20..0x27 */
	RGX_HWPERF_PWR_EST_REQUEST		= 0x20,
	RGX_HWPERF_PWR_EST_READY		= 0x21,
	RGX_HWPERF_PWR_EST_RESULT		= 0x22,
	RGX_HWPERF_PWR_CHG				= 0x23,

	/* context switch types 0x30..0x31 */
	RGX_HWPERF_CSW_START			= 0x30,
	RGX_HWPERF_CSW_FINISHED			= 0x31,

	/* ufo types 0x38 */
	RGX_HWPERF_UFO					= 0x38,

	/* last */
	RGX_HWPERF_LAST_TYPE,

	/* This enumeration must have a value that is a power of two as it is
	 * used in masks and a filter bit field (currently 64 bits long).
	 */
	RGX_HWPERF_MAX_TYPE				= 0x40
} RGX_HWPERF_EVENT_TYPE;

typedef enum {
	RGX_HWPERF_HOST_INVALID        = 0x00,
	RGX_HWPERF_HOST_ENQ            = 0x01,

	/* last */
	RGX_HWPERF_HOST_LAST_TYPE,

	/* This enumeration must have a value that is a power of two as it is
	 * used in masks and a filter bit field (currently 32 bits long).
	 */
	RGX_HWPERF_HOST_MAX_TYPE       = 0x20
} RGX_HWPERF_HOST_EVENT_TYPE;

/* The event type values are incrementing integers for use as a shift ordinal
 * in the event filtering process at the point events are generated.
 * This scheme thus implies a limit of 63 event types.
 */
static_assert(RGX_HWPERF_LAST_TYPE < RGX_HWPERF_MAX_TYPE, "Too many HWPerf event types");

/*! Type obsolete and will be removed in a later release, use RGXFWIF_DM */
typedef RGXFWIF_DM RGX_HWPERF_DM;
#define RGX_HWPERF_DM_GP	RGXFWIF_DM_GP
#define RGX_HWPERF_DM_2D	RGXFWIF_DM_2D
#define RGX_HWPERF_DM_TA	RGXFWIF_DM_TA
#define RGX_HWPERF_DM_3D	RGXFWIF_DM_3D
#define RGX_HWPERF_DM_CDM	RGXFWIF_DM_CDM
#define RGX_HWPERF_DM_RTU	RGXFWIF_DM_RTU
#define RGX_HWPERF_DM_SHG   RGXFWIF_DM_SHG
#define RGX_HWPERF_DM_LAST	RGXFWIF_DM_LAST


/******************************************************************************
 * 	Packet Format Version 2 Types
 *****************************************************************************/

/*! Signature ASCII pattern 'HWP2' found in the first word of a HWPerfV2 packet
 */
#define HWPERF_PACKET_V2_SIG		0x48575032
/*! Signature ASCII pattern 'HWPA' found in the first word of a HWPerfV2a packet
 */
#define HWPERF_PACKET_V2A_SIG		0x48575041

/*! Signature ASCII pattern 'HWPB' found in the first word of a HWPerfV2b packet
 */
#define HWPERF_PACKET_V2B_SIG		0x48575042

#define HWPERF_PACKET_ISVALID(_ptr) (((_ptr) == HWPERF_PACKET_V2_SIG) || ((_ptr) == HWPERF_PACKET_V2A_SIG)|| ((_ptr) == HWPERF_PACKET_V2B_SIG))

/*! This structure defines version 2 of the packet format which is
 * based around a header and a variable length data payload structure.
 * The address of the next packet can be found by adding the ui16Size field
 * in the header to the current packet address.
 * Producers of packets must always ensure the size field is a multiple of 8
 * as packets must start on an 8-byte granular address.
 */
typedef struct
{
	/* HEADER - packet header fields common to all packet types */
	IMG_UINT32  ui32Sig;        /*!< Always the value HWPERF_PACKET_SIG */

	IMG_UINT32  ui32Size;       /*!< Overall packet size in bytes, includes
	                             * header and payload. Size is a 16-bit field
	                             * stored in the 16 LSb. 16 MSb reserved.
	                             * Use RGX_HWPERF_MAKE_SIZE_* and RGX_HWPERF_GET_SIZE
	                             * macros to set/get, never write directly. */

	IMG_UINT32  eTypeId;        /*!< Includes event type and META thread ID in
	                             * the 19 LSb. 13 MSb reserved.
	                             * Use RGX_HWPERF_MAKE_TYPEID and RGX_HWPERF_GET_*
	                             * macros to set/get, never write directly. */

	IMG_UINT32  ui32Ordinal;    /*!< Sequential number of the packet */
	IMG_UINT64  ui64Timestamp;   /*!< Depending on the side that packet oridinated
	                              * may be either CPU timestamp or value of RGX_CR_TIMER
	                              * at event. */

	/* PAYLOAD - bytes from this point on in the buffer are from the
	 * RGX_HWPERF_V2_PACKET_DATA union which encodes the payload data specific
	 * to the event type set in the header. When the structure in the union
	 * has a variable length member e.g. HW packets the payload length
	 * varies.
	 */
} RGX_HWPERF_V2_PACKET_HDR, *RGX_PHWPERF_V2_PACKET_HDR;

RGX_FW_STRUCT_OFFSET_ASSERT(RGX_HWPERF_V2_PACKET_HDR, ui64Timestamp);

RGX_FW_STRUCT_SIZE_ASSERT(RGX_HWPERF_V2_PACKET_HDR);


/*! Mask for use with the IMG_UINT32 ui32Size header field */
#define RGX_HWPERF_SIZE_MASK			0xFFFFU

/*! Macro which takes a structure name and provides the packet size for
 * a fixed size payload packet for assignment to the ui16Size field. */
#define RGX_HWPERF_MAKE_SIZE_FIXED(_struct)       ((IMG_UINT32)(RGX_HWPERF_SIZE_MASK&(sizeof(RGX_HWPERF_V2_PACKET_HDR)+sizeof(_struct))))

/*! Macro which takes the number of bytes written in the data payload of a
 * packet for a variable size payload packet, rounds it up to 8 bytes where
 * it may be assigned to the ui16Size field. */
#define RGX_HWPERF_MAKE_SIZE_VARIABLE(_size)       ((IMG_UINT32)(RGX_HWPERF_SIZE_MASK&(sizeof(RGX_HWPERF_V2_PACKET_HDR)+PVR_ALIGN(_size, 8))))

/*! Macro to obtain the size of the packet */
#define RGX_HWPERF_GET_SIZE(_packet_addr)    ((IMG_UINT16)(((_packet_addr)->ui32Size) & RGX_HWPERF_SIZE_MASK))

/*! Macro to obtain the size of the packet data */
#define RGX_HWPERF_GET_DATA_SIZE(_packet_addr)   (RGX_HWPERF_GET_SIZE(_packet_addr) - sizeof(RGX_HWPERF_V2_PACKET_HDR))


/*! Masks for use with the IMG_UINT32 eTypeId header field */
#define RGX_HWPERF_TYPEID_MASK			0x7FFFFU
#define RGX_HWPERF_TYPEID_THREAD_MASK	0x08000U
#define RGX_HWPERF_TYPEID_STREAM_MASK	0x70000U
#define RGX_HWPERF_TYPEID_EVENT_MASK	(RGX_HWPERF_MAX_TYPE-1)

/*! Meta thread macros for encoding the ID into the type field of a packet */
#define RGX_HWPERF_META_THREAD_SHIFT	15U
#define RGX_HWPERF_META_THREAD_ID0		0x0U
#define RGX_HWPERF_META_THREAD_ID1		0x1U
/*! Obsolete, kept for source compatibility */
#define RGX_HWPERF_META_THREAD_MASK		0x1U
/*! Stream ID macros for encoding the ID into the type field of a packet */
#define RGX_HWPERF_STREAM_SHIFT			16U
typedef enum {
	RGX_HWPERF_STREAM_ID0_FW,
	RGX_HWPERF_STREAM_ID1_HOST,
	RGX_HWPERF_STREAM_ID_LAST,
} RGX_HWPERF_STREAM_ID;

/* Checks if all stream IDs can fit under RGX_HWPERF_TYPEID_STREAM_MASK. */
static_assert((RGX_HWPERF_STREAM_ID_LAST - 1) < (RGX_HWPERF_TYPEID_STREAM_MASK >> RGX_HWPERF_STREAM_SHIFT),
		"To many HWPerf stream IDs.");

/*! Macros used to set the packet type and encode meta thread ID (0|1) within */
#define RGX_HWPERF_MAKE_TYPEID(_stream,_type,_thread)\
		((IMG_UINT32) ((RGX_HWPERF_TYPEID_STREAM_MASK&((_stream)<<RGX_HWPERF_STREAM_SHIFT)) | \
		(RGX_HWPERF_TYPEID_THREAD_MASK&((_thread)<<RGX_HWPERF_META_THREAD_SHIFT)) | \
		(RGX_HWPERF_TYPEID_EVENT_MASK&(_type))))

/*! Obtains the event type that generated the packet */
#define RGX_HWPERF_GET_TYPE(_packet_addr)            (((_packet_addr)->eTypeId) & RGX_HWPERF_TYPEID_EVENT_MASK)

/*! Obtains the META Thread number that generated the packet */
#define RGX_HWPERF_GET_THREAD_ID(_packet_addr)       (((((_packet_addr)->eTypeId)&RGX_HWPERF_TYPEID_THREAD_MASK) >> RGX_HWPERF_META_THREAD_SHIFT))

/*! Obtain stream id */
#define RGX_HWPERF_GET_STREAM_ID(_packet_addr)       (((((_packet_addr)->eTypeId)&RGX_HWPERF_TYPEID_STREAM_MASK) >> RGX_HWPERF_STREAM_SHIFT))

/*! Macros to obtain a typed pointer to a packet or data structure given a packet address */
#define RGX_HWPERF_GET_PACKET(_buffer_addr)            ((RGX_HWPERF_V2_PACKET_HDR*)  (_buffer_addr))
#define RGX_HWPERF_GET_PACKET_DATA_BYTES(_packet_addr) ((IMG_BYTE*) ( ((IMG_BYTE*)(_packet_addr)) +sizeof(RGX_HWPERF_V2_PACKET_HDR) ) )
#define RGX_HWPERF_GET_NEXT_PACKET(_packet_addr)       ((RGX_HWPERF_V2_PACKET_HDR*)  ( ((IMG_BYTE*)(_packet_addr))+(RGX_HWPERF_SIZE_MASK&(_packet_addr)->ui32Size)) )

/*! Obtains a typed pointer to a packet header given the packed data address */
#define RGX_HWPERF_GET_PACKET_HEADER(_packet_addr)     ((RGX_HWPERF_V2_PACKET_HDR*)  ( ((IMG_BYTE*)(_packet_addr)) - sizeof(RGX_HWPERF_V2_PACKET_HDR) ))


/*! Masks for use with the IMG_UINT32 ui32BlkInfo field */
#define RGX_HWPERF_BLKINFO_BLKCOUNT_MASK	0xFFFF0000U
#define RGX_HWPERF_BLKINFO_BLKOFFSET_MASK	0x0000FFFFU

/*! Shift for the NumBlocks and counter block offset field in ui32BlkInfo */
#define RGX_HWPERF_BLKINFO_BLKCOUNT_SHIFT	16U
#define RGX_HWPERF_BLKINFO_BLKOFFSET_SHIFT 0U

/*! Macro used to set the block info word as a combination of two 16-bit integers */
#define RGX_HWPERF_MAKE_BLKINFO(_numblks,_blkoffset) ((IMG_UINT32) ((RGX_HWPERF_BLKINFO_BLKCOUNT_MASK&((_numblks) << RGX_HWPERF_BLKINFO_BLKCOUNT_SHIFT)) | (RGX_HWPERF_BLKINFO_BLKOFFSET_MASK&((_blkoffset) << RGX_HWPERF_BLKINFO_BLKOFFSET_SHIFT))))

/*! Macro used to obtain get the number of counter blocks present in the packet */
#define RGX_HWPERF_GET_BLKCOUNT(_blkinfo)            ((_blkinfo & RGX_HWPERF_BLKINFO_BLKCOUNT_MASK) >> RGX_HWPERF_BLKINFO_BLKCOUNT_SHIFT)

/*! Obtains the offset of the counter block stream in the packet */
#define RGX_HWPERF_GET_BLKOFFSET(_blkinfo)           ((_blkinfo & RGX_HWPERF_BLKINFO_BLKOFFSET_MASK) >> RGX_HWPERF_BLKINFO_BLKOFFSET_SHIFT)

/* This is the maximum frame contexts that are supported in the driver at the moment */
#define RGX_HWPERF_HW_MAX_WORK_CONTEXT               2
/*! This structure holds the field data of a Hardware packet.
 */
#define RGX_HWPERF_HW_DATA_FIELDS_LIST \
IMG_UINT32 ui32DMCyc;         /*!< DataMaster cycle count register, 0 if none */\
IMG_UINT32 ui32FrameNum;      /*!< Frame number */\
IMG_UINT32 ui32PID;           /*!< Process identifier */\
IMG_UINT32 ui32DMContext;     /*!< RenderContext for a TA,3D, Compute context for CDM, etc. */\
IMG_UINT32 ui32RenderTarget;  /*!< RenderTarget for a TA,3D, 0x0 otherwise */\
IMG_UINT32 ui32ExtJobRef;     /*!< Externally provided job reference used to track work for debugging purposes */\
IMG_UINT32 ui32IntJobRef;     /*!< Internally provided job reference used to track work for debugging purposes */\
IMG_UINT32 ui32TimeCorrIndex; /*!< Index to the time correlation at the time the packet was generated */\
IMG_UINT32 ui32BlkInfo;       /*!< <31..16> NumBlocks <15..0> Counterblock stream offset */\
IMG_UINT32 ui32WorkContext;   /*!< Work context number. Frame number for RTU DM, 0x0 otherwise */

typedef struct
{
	RGX_HWPERF_HW_DATA_FIELDS_LIST
} RGX_HWPERF_HW_DATA_FIELDS;

RGX_FW_STRUCT_SIZE_ASSERT(RGX_HWPERF_HW_DATA_FIELDS);

/*! Masks for use with the RGX_HWPERF_UFO_EV eEvType field */
#define RGX_HWPERF_UFO_STREAMSIZE_MASK 0xFFFF0000U
#define RGX_HWPERF_UFO_STREAMOFFSET_MASK 0x0000FFFFU

/*! Shift for the UFO count and data stream fields */
#define RGX_HWPERF_UFO_STREAMSIZE_SHIFT 16U
#define RGX_HWPERF_UFO_STREAMOFFSET_SHIFT 0U

/*! Macro used to set ufo stream info word as a combination of two 16-bit integers */
#define RGX_HWPERF_MAKE_UFOPKTINFO(_ssize,_soff)\
        ((IMG_UINT32) ((RGX_HWPERF_UFO_STREAMSIZE_MASK&((_ssize) << RGX_HWPERF_UFO_STREAMSIZE_SHIFT)) |\
        (RGX_HWPERF_UFO_STREAMOFFSET_MASK&((_soff) << RGX_HWPERF_UFO_STREAMOFFSET_SHIFT))))

/*! Macro used to obtain ufo count*/
#define RGX_HWPERF_GET_UFO_STREAMSIZE(_streaminfo)\
        ((_streaminfo & RGX_HWPERF_UFO_STREAMSIZE_MASK) >> RGX_HWPERF_UFO_STREAMSIZE_SHIFT)

/*! Obtains the offset of the ufo stream in the packet */
#define RGX_HWPERF_GET_UFO_STREAMOFFSET(_streaminfo)\
        ((_streaminfo & RGX_HWPERF_UFO_STREAMOFFSET_MASK) >> RGX_HWPERF_UFO_STREAMOFFSET_SHIFT)

typedef enum {
	RGX_HWPERF_KICK_TYPE_TA3D,
	RGX_HWPERF_KICK_TYPE_TQ2D,
	RGX_HWPERF_KICK_TYPE_TQ3D,
	RGX_HWPERF_KICK_TYPE_CDM,
	RGX_HWPERF_KICK_TYPE_RS,
	RGX_HWPERF_KICK_TYPE_VRDM,
	RGX_HWPERF_KICK_TYPE_LAST
} RGX_HWPERF_KICK_TYPE;

typedef struct
{
	RGX_HWPERF_KICK_TYPE ui32EnqType;
	IMG_UINT32 ui32PID;
	IMG_UINT32 ui32ExtJobRef;
	IMG_UINT32 ui32IntJobRef;
	IMG_UINT32 ui32FWCtx;
} RGX_HWPERF_HOST_ENQ_DATA;

/******************************************************************************
 * 	API Types
 *****************************************************************************/

/*! Counter block IDs for all the hardware blocks with a performance
 * counting module. Directly addressable blocks must have a value between 0..15.
 * First hex digit represents a group number and the second hex digit represents
 * the unit within the group. Group 0 is the direct group, all others are
 * indirect groups.
 */
typedef enum
{
	/* Directly addressable counter blocks */
	RGX_CNTBLK_ID_TA			= 0x0000,
	RGX_CNTBLK_ID_RASTER		= 0x0001, /* Non-cluster grouping cores */
	RGX_CNTBLK_ID_HUB			= 0x0002, /* Non-cluster grouping cores */
	RGX_CNTBLK_ID_TORNADO		= 0x0003, /* XT cores */
	RGX_CNTBLK_ID_JONES			= 0x0004, /* S7 cores */
	RGX_CNTBLK_ID_BF			= 0x0005, /* Doppler unit */
	RGX_CNTBLK_ID_BT			= 0x0006, /* Doppler unit */
	RGX_CNTBLK_ID_RT			= 0x0007, /* Doppler unit */
	RGX_CNTBLK_ID_BX_TU			= 0x0008, /* Doppler unit */
	RGX_CNTBLK_ID_SH			= 0x0009, /* Ray tracing unit */

	RGX_CNTBLK_ID_DIRECT_LAST,

	/* Indirectly addressable counter blocks */
	RGX_CNTBLK_ID_TPU_MCU0		= 0x0010, /* Addressable by Dust */
	RGX_CNTBLK_ID_TPU_MCU1		= 0x0011,
	RGX_CNTBLK_ID_TPU_MCU2		= 0x0012,
	RGX_CNTBLK_ID_TPU_MCU3		= 0x0013,
	RGX_CNTBLK_ID_TPU_MCU4		= 0x0014,
	RGX_CNTBLK_ID_TPU_MCU5		= 0x0015,
	RGX_CNTBLK_ID_TPU_MCU6		= 0x0016,
	RGX_CNTBLK_ID_TPU_MCU7		= 0x0017,
	RGX_CNTBLK_ID_TPU_MCU_ALL	= 0x4010,

	RGX_CNTBLK_ID_USC0			= 0x0020, /* Addressable by Cluster */
	RGX_CNTBLK_ID_USC1			= 0x0021,
	RGX_CNTBLK_ID_USC2			= 0x0022,
	RGX_CNTBLK_ID_USC3			= 0x0023,
	RGX_CNTBLK_ID_USC4			= 0x0024,
	RGX_CNTBLK_ID_USC5			= 0x0025,
	RGX_CNTBLK_ID_USC6			= 0x0026,
	RGX_CNTBLK_ID_USC7			= 0x0027,
	RGX_CNTBLK_ID_USC8			= 0x0028,
	RGX_CNTBLK_ID_USC9			= 0x0029,
	RGX_CNTBLK_ID_USC10			= 0x002A,
	RGX_CNTBLK_ID_USC11			= 0x002B,
	RGX_CNTBLK_ID_USC12			= 0x002C,
	RGX_CNTBLK_ID_USC13			= 0x002D,
	RGX_CNTBLK_ID_USC14			= 0x002E,
	RGX_CNTBLK_ID_USC15			= 0x002F,
	RGX_CNTBLK_ID_USC_ALL		= 0x4020,

	RGX_CNTBLK_ID_TEXAS0		= 0x0030, /* Addressable by Phantom in XT, Dust in S7 */
	RGX_CNTBLK_ID_TEXAS1		= 0x0031,
	RGX_CNTBLK_ID_TEXAS2		= 0x0032,
	RGX_CNTBLK_ID_TEXAS3		= 0x0033,
	RGX_CNTBLK_ID_TEXAS4		= 0x0034,
	RGX_CNTBLK_ID_TEXAS5		= 0x0035,
	RGX_CNTBLK_ID_TEXAS6		= 0x0036,
	RGX_CNTBLK_ID_TEXAS7		= 0x0037,
	RGX_CNTBLK_ID_TEXAS_ALL		= 0x4030,

	RGX_CNTBLK_ID_RASTER0		= 0x0040, /* Addressable by Phantom, XT only */
	RGX_CNTBLK_ID_RASTER1		= 0x0041,
	RGX_CNTBLK_ID_RASTER2		= 0x0042,
	RGX_CNTBLK_ID_RASTER3		= 0x0043,
	RGX_CNTBLK_ID_RASTER_ALL	= 0x4040,

	RGX_CNTBLK_ID_BLACKPEARL0	= 0x0050, /* Addressable by Phantom, S7 only */
	RGX_CNTBLK_ID_BLACKPEARL1	= 0x0051,
	RGX_CNTBLK_ID_BLACKPEARL2	= 0x0052,
	RGX_CNTBLK_ID_BLACKPEARL3	= 0x0053,
	RGX_CNTBLK_ID_BLACKPEARL_ALL= 0x4050,

	RGX_CNTBLK_ID_PBE0			= 0x0060, /* Addressable by Cluster, S7 only */
	RGX_CNTBLK_ID_PBE1			= 0x0061,
	RGX_CNTBLK_ID_PBE2			= 0x0062,
	RGX_CNTBLK_ID_PBE3			= 0x0063,
	RGX_CNTBLK_ID_PBE4			= 0x0064,
	RGX_CNTBLK_ID_PBE5			= 0x0065,
	RGX_CNTBLK_ID_PBE6			= 0x0066,
	RGX_CNTBLK_ID_PBE7			= 0x0067,
	RGX_CNTBLK_ID_PBE8			= 0x0068,
	RGX_CNTBLK_ID_PBE9			= 0x0069,
	RGX_CNTBLK_ID_PBE10			= 0x006A,
	RGX_CNTBLK_ID_PBE11			= 0x006B,
	RGX_CNTBLK_ID_PBE12			= 0x006C,
	RGX_CNTBLK_ID_PBE13			= 0x006D,
	RGX_CNTBLK_ID_PBE14			= 0x006E,
	RGX_CNTBLK_ID_PBE15			= 0x006F,
	RGX_CNTBLK_ID_PBE_ALL		= 0x4060,

	RGX_CNTBLK_ID_LAST			= 0x0070,

	RGX_CNTBLK_ID_CUSTOM0       = 0x7FF0,
	RGX_CNTBLK_ID_CUSTOM1       = 0x7FF1,
	RGX_CNTBLK_ID_CUSTOM2       = 0x7FF2,
	RGX_CNTBLK_ID_CUSTOM3       = 0x7FF3,

} RGX_HWPERF_CNTBLK_ID;

/* Masks for the counter block ID*/
#define RGX_CNTBLK_ID_GROUP_MASK     (0x00F0U)
#define RGX_CNTBLK_ID_GROUP_SHIFT    (4)
#define RGX_CNTBLK_ID_UNIT_ALL_MASK  (0x4000U)
#define RGX_CNTBLK_ID_UNIT_MASK		 (0xf)

/*! Identifier for each counter in a performance counting module */
typedef enum
{
	RGX_CNTBLK_COUNTER0_ID	  = 0,
	RGX_CNTBLK_COUNTER1_ID	  = 1,
	RGX_CNTBLK_COUNTER2_ID	  = 2,
	RGX_CNTBLK_COUNTER3_ID	  = 3,
	RGX_CNTBLK_COUNTER4_ID	  = 4,
	RGX_CNTBLK_COUNTER5_ID	  = 5
	/* RGX_HWPERF_CNTRS_IN_BLK has the maximum number of counters per block*/

} RGX_HWPERF_CNTBLK_COUNTER_ID;

/*! Mask macros for use with RGXCtrlHWPerf() API.
 * RGX_HWPERF_EVENT_ALL is obsolete, use RGX_HWPERF_EVENT_MASK_ALL
 */
#define RGX_HWPERF_EVENT_MASK_NONE          (IMG_UINT64_C(0x0000000000000000))
#define RGX_HWPERF_EVENT_MASK_ALL           (IMG_UINT64_C(0xFFFFFFFFFFFFFFFF))
#define RGX_HWPERF_EVENT_MASK_ALL_FW        (IMG_UINT64_C(0x000000000000007E))
#define RGX_HWPERF_EVENT_MASK_HW_KICKFINISH (IMG_UINT64_C(0x0000000001FBFF00))
#define RGX_HWPERF_EVENT_MASK_HW_PERIODIC   (IMG_UINT64_C(0x0000000000040000))
#define RGX_HWPERF_EVENT_MASK_ALL_HW        (RGX_HWPERF_EVENT_MASK_HW_KICKFINISH \
                                            | RGX_HWPERF_EVENT_MASK_HW_PERIODIC)
#define RGX_HWPERF_EVENT_MASK_ALL_PWR_EST   (IMG_UINT64_C(0X0000000700000000))
#define RGX_HWPERF_EVENT_MASK_ALL_PWR       (IMG_UINT64_C(0X0000000800000000))
#define RGX_HWPERF_EVENT_MASK_VALUE(e)      (((IMG_UINT64)1)<<(e))

/*! Type used in the RGX API RGXConfigureAndEnableHWPerfCounters()
 * It is used to configure the performance counter module in a layout
 * block and allows one or more counters in the block to be 
 * configured in one operation based on the counter select mask. The bit
 * shifts for this are the values in RGX_HWPERF_CNTBLK_COUNTER_ID. This mask
 * also encodes which values in the arrays are valid, for example, if bit 1 set
 * then aui8Mode[1], aui16GroupSelect[1], aui16BitSelect[1], aui32BatchMax[1],
 * and aui32BatchMin[1] must be set. If these array elements are all set to 
 * 0 then the counter will not count and will not be in the HW event, 
 * effectively disabling the counter from the callers point of view. 
 * If any are non zero then the counter will be included in the HW event.
 *
 * Each layout block has 4 or 6 counters that can be programmed independently to
 * profile the performance of a HW block. Each counter can be configured to
 * accumulate statistics from 1 of 32 counter groups defined for that block.
 * Each counter group can have up to 16	signals/bits defined that can be
 * selected. Each counter may accumulate in one of two modes.
 * See hwdefs/regapiperf.h for block/group/signal definitions.
 */
 typedef struct _RGX_HWPERF_CONFIG_CNTBLK_
{
	/*! Counter block ID, see RGX_HWPERF_CNTBLK_ID */
	IMG_UINT16 ui16BlockID;

	/*! 4 or 6 LSBs are a mask of which counters to configure. Bit 0 is counter 0,
	 * bit 1 is counter 1 on so on. */
	IMG_UINT8   ui8CounterSelect;

	/*! 4 or 6 LSBs 0 for counting 1's in the group, 1 for treating the group
	 * signals as a number for unsigned addition. Bit 0 is counter 0, bit 1 is
	 * counter 1 on so on. This member relates to the MODE field
	 * in the RGX_CR_<N>_PERF_SELECTm register for each counter */
	IMG_UINT8	ui8Mode;

	/*! 5 or 6 LSBs used as the GROUP_SELECT field in the RGX_CR_<N>_PERF_SELECTm
	 * register. Array index 0 is counter 0, index 1 is counter 1 and so on. */
	IMG_UINT8	aui8GroupSelect[RGX_HWPERF_CNTRS_IN_BLK];

	/*! 16 LSBs used as the BIT_SELECT field in the RGX_CR_<N>_PERF_SELECTm
	 * register. Array indexes relate to counters as above. */
	IMG_UINT16  aui16BitSelect[RGX_HWPERF_CNTRS_IN_BLK];

	/*! 14 LSBs used as the BATCH_MAX field in the RGX_CR_<N>_PERF_SELECTm
	 * register. Array indexes relate to counters as above. */
	IMG_UINT32  aui32BatchMax[RGX_HWPERF_CNTRS_IN_BLK];

	/*! 14 LSBs used as the BATCH_MIN field in the RGX_CR_<N>_PERF_SELECTm
	 * register. Array indexes relate to counters as above. */
	IMG_UINT32  aui32BatchMin[RGX_HWPERF_CNTRS_IN_BLK];
} UNCACHED_ALIGN RGX_HWPERF_CONFIG_CNTBLK;

RGX_FW_STRUCT_SIZE_ASSERT(RGX_HWPERF_CONFIG_CNTBLK);

typedef enum {
	RGX_HWPERF_UFO_EV_UPDATE,
	RGX_HWPERF_UFO_EV_CHECK_SUCCESS,
	RGX_HWPERF_UFO_EV_PRCHECK_SUCCESS,
	RGX_HWPERF_UFO_EV_CHECK_FAIL,
	RGX_HWPERF_UFO_EV_PRCHECK_FAIL,

	RGX_HWPERF_UFO_EV_LAST
} RGX_HWPERF_UFO_EV;

/*! Data stream tuple. */
typedef union
{
	struct
	{
		IMG_UINT32 ui32FWAddr;
		IMG_UINT32 ui32Value;
	} sCheckSuccess;
	struct
	{
		IMG_UINT32 ui32FWAddr;
		IMG_UINT32 ui32Value;
		IMG_UINT32 ui32Required;
	} sCheckFail;
	struct
	{
		IMG_UINT32 ui32FWAddr;
		IMG_UINT32 ui32OldValue;
		IMG_UINT32 ui32NewValue;
	} sUpdate;
} RGX_HWPERF_UFO_DATA_ELEMENT;

/*! This structure holds data for ufo packet. */
typedef struct
{
	RGX_HWPERF_UFO_EV eEvType;
	IMG_UINT32 ui32TimeCorrIndex;
	IMG_UINT32 ui32PID;
	IMG_UINT32 ui32ExtJobRef;
	IMG_UINT32 ui32IntJobRef;
	IMG_UINT32 ui32FWCtx;
	IMG_UINT32 ui32StreamInfo;
	IMG_UINT32 ui32DM;
	IMG_UINT32 ui32Padding;

	IMG_UINT32 aui32StreamData[sizeof(RGX_HWPERF_UFO_DATA_ELEMENT) / sizeof(IMG_UINT32)];
	/*!< Data stream contains UFO objects for specific command.
	 *
	 * Data stream consists of tuples containing UFO related data. Depending
	 * on the UFO event type there are three tuple formats:
	 *      For UPDATE packet tuple consist of two 32bit values:
	 *          <32bit> :        UFO's firmware address
	 *          <32bit> :        old UFO's value
	 *          <32bit> :        update value
	 *      For PRCHECK/CHECK SUCCESS packets tuple consists of two 32bit
	 *      values:
	 *          <32bit> :        UFO's firmware address
	 *          <32bit> :        UFO's/fence value
	 *      For PRCHECK/CHECK FAIL packets tuple consists of three 32bit values:
	 *          <32bit> :        UFO's firmware address
	 *          <32bit> :        UFO's value
	 *          <32bit> :        fence value
	 *
	 * An example of data stream:
	 *          <UFO0addr>, <UFO0val>, <Fnc0Val>, <UFO1addr>, <UFO1val>, <Fnc1Val> ...
	 *
	 * The array size is at least large enough to fit in one tuple. Real
	 * size is however dynamic and reflects number of all tuples that are fit
	 * into the array.
	 */
} RGX_HWPERF_UFO_DATA;

RGX_FW_STRUCT_SIZE_ASSERT(RGX_HWPERF_UFO_DATA);

#if defined (__cplusplus)
}
#endif

#endif /* RGX_HWPERF_KM_H_ */

/******************************************************************************
 End of file
******************************************************************************/

