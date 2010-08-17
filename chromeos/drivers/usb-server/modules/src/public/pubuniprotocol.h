#ifndef _PUBUNIPROTOCOL_H_
#define _PUBUNIPROTOCOL_H_

#include "nettypes.h"

#pragma pack(push,1)

enum
{
	UNRB_FUNCTION_GET_DESCRIPTOR = 0,
	UNRB_FUNCTION_SET_DESCRIPTOR,
	UNRB_FUNCTION_SELECT_CONFIGURATION,
	UNRB_FUNCTION_SELECT_INTERFACE,
	UNRB_FUNCTION_CONTROL_TRANSFER,
	UNRB_FUNCTION_BULK_TRANSFER,
	UNRB_FUNCTION_ISOCH_TRANSFER,
	UNRB_FUNCTION_INTERRUPT_TRANSFER,
	UNRB_FUNCTION_CLEAR_STALL,
	UNRB_FUNCTION_GET_CURRENT_FRAME_NUMBER,
	UNRB_FUNCTION_RAW_DATA,
	UNRB_FUNCTION_GET_PORT_STATUS,
	UNRB_FUNCTION_RESET_PORT,
	UNRB_FUNCTION_CANCEL,
	UNRB_FUNCTION_ABORT_ENDPOINT,
};

enum
{
	UNRB_STATUS_SUCCESS = 0,
	UNRB_STATUS_PENDING,
	UNRB_STATUS_CANCELLED,
	UNRB_STATUS_ISO_TRANSFER_INCOMPLETE,
	UNRB_STATUS_TIMEOUT,
	UNRB_STATUS_REQUEST_FAILED,
	UNRB_STATUS_NO_DEVICE,
	UNRB_STATUS_SHORT_TRANSFER,
	UNRB_STATUS_BABBLE_DETECTED,
	UNRB_STATUS_DATA_UNDERRUN,
	UNRB_STATUS_DATA_OVERRUN,
	UNRB_STATUS_STALLED,
	UNRB_STATUS_CRC_ERROR,
	UNRB_STATUS_PROTOCOL_ERROR,
	UNRB_STATUS_NOT_ENOUGH_MEMORY,
	UNRB_STATUS_NOT_ENOUGH_BANDWIDTH,
	UNRB_STATUS_UNSUCCESSFUL,
	UNRB_STATUS_MESSAGE_TOO_LONG
};


#define UNRB_FLAG_DIRECTION_IN 			0x01
#define UNRB_FLAG_SHORT_TRANSFER_OK 		0x02
#define UNRB_FLAG_ISO_TRANSFER_ASAP          	0x04
#define UNRB_FLAG_DIRECTION_OUT 			0x08
                        	
typedef struct
{
	uint64 UniqueId;
	uint32 Size;
	uint32 Function;
	uint32 Status;
	uint32 Context;
} __attribute__((packed)) UNRB_HEADER, *PUNRB_HEADER;

typedef struct
{
	UNRB_HEADER Header;
	uint8 RequestType;
	uint8 RequestRecipient;
	uint8 DescType;
	uint8 DescIndex;
	uint16 LangId;
	uint32 BufferSize;
} __attribute__((packed)) UNRB_DESCRIPTOR_REQUEST, *PUNRB_DESCRIPTOR_REQUEST;


typedef struct
{
	UNRB_HEADER Header;
	uint8 ConfigurationNum;
	uint8 NumAlternates;
	struct
	{
		uint8 InterfaceNum;
		uint8 AlternateNum;
	} __attribute__((packed)) Alternates[1];
} __attribute__((packed)) UNRB_SELECT_CONFIGURATION, *PUNRB_SELECT_CONFIGURATION;

typedef struct
{
	UNRB_HEADER Header;
	uint8 InterfaceNum;
	uint8 AlternateNum;
} __attribute__((packed)) UNRB_SELECT_INTERFACE, *PUNRB_SELECT_INTERFACE;

#define UNRB_RT_TYPE_STANDARD			0x00
#define UNRB_RT_TYPE_CLASS				0x01
#define UNRB_RT_TYPE_VENDOR				0x02
#define UNRB_RT_TYPE_RESERVED			0x03

#define UNRB_RT_RECIPIENT_DEVICE		0x00
#define UNRB_RT_RECIPIENT_INTERFACE		0x01
#define UNRB_RT_RECIPIENT_ENDPOINT		0x02
#define UNRB_RT_RECIPIENT_OTHER			0x03

#define UNRB_RT_DIRECTION_IN			0x01
#define UNRB_RT_DIRECTION_OUT			0x00

typedef struct
{
	UNRB_HEADER Header;
	uint8 Endpoint;
	uint8 Flags;
	union
	{
		struct
		{
			uint8 btRecipient:5;
			uint8 btType:2;
			uint8 btDirection:1;
		} __attribute__((packed)) RequestTypeBits;
		uint8 RequestType;
	} __attribute__((packed));
	uint8 Request;
	uint16 Value;
	uint16 Index;
	uint32 BufferSize;
} __attribute__((packed)) UNRB_CONTROL_TRANSFER, *PUNRB_CONTROL_TRANSFER;

typedef struct
{
	UNRB_HEADER Header;
	uint32 BufferSize;
	uint8 Endpoint;
	uint8 Flags;
} __attribute__((packed)) UNRB_BULK_TRANSFER, *PUNRB_BULK_TRANSFER;

typedef struct
{
	UNRB_HEADER Header;
	uint32 BufferSize;
	uint32 Interval;
	uint8 Endpoint;
	uint8 Flags;
} __attribute__((packed)) UNRB_INTERRUPT_TRANSFER, *PUNRB_INTERRUPT_TRANSFER;

typedef struct
{
	UNRB_HEADER Header;
	uint8 Flags;
	uint8 Endpoint;	

} __attribute__((packed)) UNRB_CLEAR_STALL, *PUNRB_CLEAR_STALL;

typedef struct
{
	UNRB_HEADER Header;
	uint32 FrameNumber;
} __attribute__((packed)) UNRB_GET_CURRENT_FRAME_NUMBER, *PUNRB_GET_CURRENT_FRAME_NUMBER;

#define GET_UNRB_ISOCH_TRANSFER_PACKETSIZE(punrb) (sizeof(UNRB_ISOCH_TRANSFER) - sizeof(UNRB_ISOCH_PACKET_DESCRIPTOR) + \
			sizeof(UNRB_ISOCH_PACKET_DESCRIPTOR)*(punrb)->IsochTransfer.NumberOfPackets)

typedef struct
{
	uint32 Offset;
	uint32 Length;
	uint32 Status;
} __attribute__((packed)) UNRB_ISOCH_PACKET_DESCRIPTOR, *PUNRB_ISOCH_PACKET_DESCRIPTOR;

typedef struct
{
	UNRB_HEADER Header;
	uint8 Endpoint;
	uint8 Flags;
	uint32 Interval;
	uint32 StartFrame;
	uint32 NumberOfPackets;
	uint32 ErrorCount;
	uint32 BufferSize;
	UNRB_ISOCH_PACKET_DESCRIPTOR IsochPackets[1];
} __attribute__((packed)) UNRB_ISOCH_TRANSFER, *PUNRB_ISOCH_TRANSFER;


#define GET_UNRB_RAW_DATA_PACKETSIZE(NumberOfDataBytes) (sizeof(UNRB_RAW_DATA)-1+(NumberOfDataBytes))
#define GET_UNRB_RAW_DATA_DATASIZE(pUnrbRawData) ((pUnrbRawData)->Header.Size-(sizeof(UNRB_RAW_DATA)-1))

typedef struct
{
	UNRB_HEADER Header;
	uint8 Data[1];
} __attribute__((packed)) UNRB_RAW_DATA, *PUNRB_RAW_DATA;


#define UNRB_PORT_STATUS_ENABLED		0x01
#define UNRB_PORT_STATUS_CONNECTED		0x02
typedef struct
{
	UNRB_HEADER Header;
	uint32 PortStatus;
} __attribute__((packed)) UNRB_GET_PORT_STATUS, *PUNRB_GET_PORT_STATUS;

typedef struct
{
	UNRB_HEADER Header;
} __attribute__((packed)) UNRB_RESET_PORT, *PUNRB_RESET_PORT;

typedef struct
{
	UNRB_HEADER Header;
} __attribute__((packed)) UNRB_CANCEL, *PUNRB_CANCEL;

typedef struct
{
	UNRB_HEADER Header;
	uint8 Endpoint;
	uint8 Flags;
} __attribute__((packed)) UNRB_ABORT_ENDPOINT, *PUNRB_ABORT_ENDPOINT;

typedef union
{
	UNRB_HEADER Header;
	UNRB_DESCRIPTOR_REQUEST		DescriptorRequest;
	UNRB_SELECT_CONFIGURATION	SelectConfiguration;
	UNRB_SELECT_INTERFACE		SelectInterface;
	UNRB_CONTROL_TRANSFER		ControlTransfer;
	UNRB_BULK_TRANSFER			BulkTransfer;
	UNRB_INTERRUPT_TRANSFER		InterruptTransfer;
	UNRB_CLEAR_STALL			ClearStall;
	UNRB_GET_CURRENT_FRAME_NUMBER	GetCurrentFrameNumber;
	UNRB_ISOCH_TRANSFER			IsochTransfer;
	UNRB_RAW_DATA				RawData;
	UNRB_GET_PORT_STATUS		GetPortStatus;
	UNRB_RESET_PORT				ResetPort;
	UNRB_CANCEL					Cancel;
	UNRB_ABORT_ENDPOINT			AbortEndpoint;
} __attribute__((packed)) UNRB, *PUNRB;

#pragma pack(pop)

#endif //_PUBUNIPROTOCOL_H_
