#ifndef _NETTYPES_
#define _NETTYPES_

#ifdef WIN32

typedef char				int8;
typedef short int			int16;
typedef long int			int32;
typedef __int64				int64;
typedef unsigned char		uint8;
typedef unsigned short int	uint16;
typedef unsigned long int	uint32;
typedef unsigned __int64	uint64;

#else

#include <linux/types.h>

typedef int8_t			int8;
typedef int16_t			int16;
typedef int32_t			int32;
typedef int64_t			int64;

typedef u_int8_t		uint8;
typedef u_int16_t		uint16;
typedef u_int32_t		uint32;
typedef u_int64_t		uint64;

#ifndef __KERNEL__

#include <endian.h>
#include <byteswap.h>

#if __BYTE_ORDER == __BIG_ENDIAN
 #define to_host16(x) bswap_16((x))
 #define to_host32(x) bswap_32((x))
 #define to_host64(x) bswap_64((x))
 #define to_net16(x) bswap_16((x))
 #define to_net32(x) bswap_32((x))
 #define to_net64(x) bswap_64((x))
  #else
 #define to_host16(x) (x)
 #define to_host32(x) (x)
 #define to_host64(x) (x)
 #define to_net16(x) (x)
 #define to_net32(x) (x)
 #define to_net64(x) (x)
#endif

# endif

#endif // WIN32

#endif // _NETTYPES_
