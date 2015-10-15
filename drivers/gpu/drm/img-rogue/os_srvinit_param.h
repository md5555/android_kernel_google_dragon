/*************************************************************************/ /*!
@File
@Title          Services initialisation parameters header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Services initialisation parameter support for the Linux kernel.
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

#ifndef __OS_SRVINIT_PARAM_H__
#define __OS_SRVINIT_PARAM_H__


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

#include "img_defs.h"
#include "img_types.h"

typedef struct
{
	IMG_CHAR *pszValue;
	IMG_UINT32 ui32Value;
} SRV_INIT_PARAM_UINT32_LOOKUP;

void *SrvInitParamOpen(void);
void SrvInitParamClose(void *pvState);

#define	SrvInitParamInitBOOL(name, defval) \
	static bool __SrvInitParam_ ## name = defval; \
	module_param_named(name, __SrvInitParam_ ## name, bool, S_IRUGO);

void _SrvInitParamGetBOOL(IMG_BOOL *pbValue, bool bDefault);

#define	SrvInitParamGetBOOL(state, name, value) \
		_SrvInitParamGetBOOL(&(value), __SrvInitParam_ ## name)

#define	SrvInitParamInitUINT32(name, defval) \
	static unsigned int __SrvInitParam_ ## name = defval; \
	module_param_named(name, __SrvInitParam_ ## name, uint, S_IRUGO);

void _SrvInitParamGetUINT32(IMG_UINT32 *pui32Value, unsigned int uiDefault);

#define	SrvInitParamGetUINT32(state, name, value) \
		_SrvInitParamGetUINT32(&(value), __SrvInitParam_ ## name)

#define	SrvInitParamInitUINT32BitField(name, inival, lookup) \
	static unsigned int __SrvInitParam_ ## name = inival; \
	static SRV_INIT_PARAM_UINT32_LOOKUP * \
		__SrvInitParamLookup_ ## name = &lookup[0]; \
	static const unsigned int __SrvInitParamSize_ ## name = \
					ARRAY_SIZE(lookup); \
	static char * __SrvInitParamArray_ ## name [ARRAY_SIZE(lookup)]; \
	static unsigned int _SrvInitParamNum_ ## name = 0; \
	static const char * __SrvInitParamName_ ## name = #name; \
	module_param_array_named(name, __SrvInitParamArray_ ## name, charp, &_SrvInitParamNum_ ## name, S_IRUGO);

bool _SrvInitParamGetUINT32BitField(IMG_UINT32 *puiValue, unsigned int uiDefault, const char **ppszValues, unsigned int uiNum, const SRV_INIT_PARAM_UINT32_LOOKUP *psLookup, unsigned int uiSize, const char *pszName);

#define	SrvInitParamGetUINT32BitField(state, name, value) \
		_SrvInitParamGetUINT32BitField(&(value), __SrvInitParam_ ## name, (const char **)__SrvInitParamArray_ ## name,  _SrvInitParamNum_ ## name, __SrvInitParamLookup_ ## name, __SrvInitParamSize_ ## name, __SrvInitParamName_ ## name)

#define	SrvInitParamInitUINT32List(name, defval, lookup) \
	static unsigned int __SrvInitParam_ ## name = defval; \
	static SRV_INIT_PARAM_UINT32_LOOKUP * \
		__SrvInitParamLookup_ ## name = &lookup[0]; \
	static const unsigned int __SrvInitParamSize_ ## name = \
					ARRAY_SIZE(lookup); \
	static const char * __SrvInitParamName_ ## name = #name; \
	static char * __SrvInitParamString_ ## name = NULL; \
	module_param_named(name, __SrvInitParamString_ ## name, charp, S_IRUGO);

bool _SrvInitParamGetUINT32List(IMG_UINT32 *puiValue, unsigned int uiDefault, const char *pszValue, const SRV_INIT_PARAM_UINT32_LOOKUP *psLookup, unsigned int uiSize, const char *pszName);

#define	SrvInitParamGetUINT32List(state, name, value) \
		_SrvInitParamGetUINT32List(&(value), __SrvInitParam_ ## name, __SrvInitParamString_ ## name, __SrvInitParamLookup_ ## name, __SrvInitParamSize_ ## name,  __SrvInitParamName_ ## name)


#endif /* __OS_SRVINIT_PARAM_H__ */
