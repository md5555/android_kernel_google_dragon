/*************************************************************************/ /*!
@File
@Title          Common header containing type definitions for portability
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Contains variable and structure definitions. Any platform
                specific types should be defined in this file.
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

#if !defined (__IMG_DEFS_H__)
#define __IMG_DEFS_H__

#include <stddef.h>

#include "img_types.h"

#if defined (NO_INLINE_FUNCS)
	#define	INLINE
	#define	FORCE_INLINE
#else
#if defined (__cplusplus)
	#define INLINE					inline
	#define	FORCE_INLINE			inline
#else
#if	!defined(INLINE)
	#define	INLINE					__inline
#endif
#if (defined(UNDER_WDDM) || defined(WINDOWS_WDF)) && defined(_X86_)
	#define	FORCE_INLINE			__forceinline
#else
	#define	FORCE_INLINE			static __inline
#endif
#endif
#endif

/* True if the GCC version is at least the given version. False for older
 * versions of GCC, or other compilers.
 */
#define GCC_VERSION_AT_LEAST(major, minor)						\
	(defined(__GNUC__) && (										\
		__GNUC__ > (major) ||									\
		(__GNUC__ == (major) && __GNUC_MINOR__ >= (minor))))

/* Use this in any file, or use attributes under GCC - see below */
#ifndef PVR_UNREFERENCED_PARAMETER
#define	PVR_UNREFERENCED_PARAMETER(param) ((void)(param))
#endif

/* static_assert(condition, "message to print if it fails");
 *
 * Assert something at compile time. If the assertion fails, try to print
 * the message, otherwise do nothing. static_assert is available if:
 *
 * - It's already defined as a macro (e.g. by <assert.h> in C11)
 * - We're using MSVC which exposes static_assert unconditionally
 * - We're using a C++ compiler that supports C++11
 * - We're using GCC 4.6 and up in C mode (in which case it's available as
 *   _Static_assert)
 *
 * In all other cases, fall back to an equivalent that makes an invalid
 * declaration.
 */
#if !defined(static_assert) && !defined(_MSC_VER) && \
		(!defined(__cplusplus) || __cplusplus < 201103L)
	/* static_assert isn't already available */
	#if GCC_VERSION_AT_LEAST(4, 6) && !defined(__cplusplus)
		#define static_assert _Static_assert
	#else
		#define static_assert(expr, message) \
			extern int _static_assert_failed[2*!!(expr) - 1] __attribute__((unused))
	#endif
#endif

/*! Macro to calculate the n-byte aligned value from that supplied rounding up.
 * n must be a power of two.
 *
 * Both arguments should be of a type with the same size otherwise the macro may
 * cut off digits, e.g. imagine a 64 bit address in _x and a 32 bit value in _n.
 */
#define PVR_ALIGN(_x, _n)   (((_x)+((_n)-1)) & ~((_n)-1))


/* The best way to supress unused parameter warnings using GCC is to use a
 * variable attribute.  Place the unref__ between the type and name of an
 * unused parameter in a function parameter list, eg `int unref__ var'. This
 * should only be used in GCC build environments, for example, in files that
 * compile only on Linux. Other files should use UNREFERENCED_PARAMETER */
#ifdef __GNUC__
#define unref__ __attribute__ ((unused))
#else
#define unref__
#endif

#if __GNUC__ >= 3
# define IMG_LIKELY(x) __builtin_expect (!!(x), 1)
# define IMG_UNLIKELY(x) __builtin_expect (!!(x), 0)
#else
# define IMG_LIKELY(x) (x)
# define IMG_UNLIKELY(x) (x)
#endif

#if defined(_WIN32)

#if defined(WINDOWS_WDF)

	/*
	 * For WINDOWS_WDF drivers we don't want these defines to overwrite calling conventions propagated through the build system.
	 * This 'empty' choice helps to resolve all the calling conv issues.
	 *
	 */
	#define IMG_CALLCONV
	#define C_CALLCONV

	#define IMG_INTERNAL
	#define IMG_RESTRICT __restrict

	/*
	 * The proper way of dll linking under MS compilers is made of two things:
	 * - decorate implementation with __declspec(dllexport)
	 *   this decoration helps compiler with making the so called 'export library'
	 * - decorate forward-declaration (in a source dependent on a dll) with __declspec(dllimport),
	 *   this decoration helps compiler with making faster and smaller code in terms of calling dll-imported functions
	 *
	 * Usually these decorations are performed by having a single macro define that expands to a proper __declspec()
	 * depending on the translation unit, dllexport inside the dll source and dllimport outside the dll source.
	 * Having IMG_EXPORT and IMG_IMPORT resolving to the same __declspec() makes no sense, but at least works.
	 */
	#define IMG_IMPORT __declspec(dllexport)
	#define IMG_EXPORT __declspec(dllexport)

#else

	#define IMG_CALLCONV __stdcall
	#define IMG_INTERNAL
	#define	IMG_EXPORT	__declspec(dllexport)
	#define IMG_RESTRICT __restrict
	#define C_CALLCONV	__cdecl

	/*
	 * IMG_IMPORT is defined as IMG_EXPORT so that headers and implementations match.
	 * Some compilers require the header to be declared IMPORT, while the implementation is declared EXPORT 
	 */
	#define	IMG_IMPORT	IMG_EXPORT

#endif

#if defined(UNDER_WDDM)
	#ifndef	_INC_STDLIB
		#if defined(__mips)
			/* do nothing */
		#elif defined(UNDER_MSBUILD)
			_CRTIMP __declspec(noreturn) void __cdecl abort(void);
		#else
			_CRTIMP void __cdecl abort(void);
		#endif
	#endif
	#if defined(EXIT_ON_ABORT)
		#define IMG_ABORT()	exit(1);
	#else
		#define IMG_ABORT()	abort();
	#endif
//	#define IMG_ABORT()	img_abort()
#endif /* UNDER_WDDM */
#else
	#if defined(LINUX) || defined(__METAG) || defined(__QNXNTO__)

		#define IMG_CALLCONV
		#define C_CALLCONV
		#if defined(__linux__) || defined(__QNXNTO__)
			#define IMG_INTERNAL	__attribute__((visibility("hidden")))
		#else
			#define IMG_INTERNAL
		#endif
		#define IMG_EXPORT		__attribute__((visibility("default")))
		#define IMG_IMPORT
		#define IMG_RESTRICT	__restrict__

	#else
		#error("define an OS")
	#endif
#endif

// Use default definition if not overridden
#ifndef IMG_ABORT
	#if defined(EXIT_ON_ABORT)
		#define IMG_ABORT()	exit(1)
	#else
		#define IMG_ABORT()	abort()
	#endif
#endif

#if defined(__GNUC__)
#define IMG_FORMAT_PRINTF(x,y)		__attribute__((format(printf,x,y)))
#else
#define IMG_FORMAT_PRINTF(x,y)
#endif

#if defined(__GNUC__)
#define IMG_WARN_UNUSED_RESULT		__attribute__((warn_unused_result))
#else
#define IMG_WARN_UNUSED_RESULT
#endif

#if defined(_MSC_VER) || defined(CC_ARM)
	#define IMG_NORETURN __declspec(noreturn)
#else
	#if defined(__GNUC__)
		#define IMG_NORETURN __attribute__((noreturn))
	#else
		#define IMG_NORETURN
	#endif
#endif

#define MAX(a,b) 					(((a) > (b)) ? (a) : (b))
#define MIN(a,b) 					(((a) < (b)) ? (a) : (b))

/* Get a structures address from the address of a member */
#define IMG_CONTAINER_OF(ptr, type, member) \
	(type *) ((IMG_UINT8 *) (ptr) - offsetof(type, member))

/* The number of elements in a fixed-sized array, IMGs ARRAY_SIZE macro */
#define IMG_ARR_NUM_ELEMS(ARR) \
	(sizeof(ARR) / sizeof((ARR)[0]))

/* To guarantee that __func__ can be used, define it as a macro here if it
   isn't already provided by the compiler. */
#if defined(_MSC_VER)
#define __func__ __FUNCTION__
#endif

#if defined(__cplusplus)
/* C++ Specific:
 * Disallow use of copy and assignment operator within a class.
 * Should be placed under private. */
#define IMG_DISALLOW_COPY_AND_ASSIGN(C) \
	C(const C&); \
	void operator=(const C&)
#endif

#if defined(SUPPORT_PVR_VALGRIND)
	#if !defined(__METAG)
		#include "/usr/include/valgrind/memcheck.h"

		#define VALGRIND_HEADER_PRESENT

		#define VG_MARK_INITIALIZED(pvData,ui32Size)  VALGRIND_MAKE_MEM_DEFINED(pvData,ui32Size)
	#else
		#define VG_MARK_INITIALIZED(pvData,ui32Size) do { } while(0)
	#endif
#else

	#define VG_MARK_INITIALIZED(pvData,ui32Size) do { } while(0)
#endif


#endif /* #if !defined (__IMG_DEFS_H__) */
/*****************************************************************************
 End of file (IMG_DEFS.H)
*****************************************************************************/

