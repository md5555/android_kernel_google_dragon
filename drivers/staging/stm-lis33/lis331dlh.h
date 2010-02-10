/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
*
* File Name          : LIS331DLH.h
* Author             : MH - C&I BU - Application Team
* Version            : V 1.1
* Date               : 24/11/2009
* Description        : LIS331DLH sensor API
*
********************************************************************************
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/ 

#ifndef __LIS331DLH__
#define __LIS331DLH__

#include <linux/ioctl.h>  /* For IOCTL macros */


#define LIS331DLH_IOCTL_BASE 'a'
/** The following define the IOCTL command values via the ioctl macros */
#define LIS331DLH_IOCTL_SET_DELAY   _IOW(LIS331DLH_IOCTL_BASE, 0, int)
#define LIS331DLH_IOCTL_GET_DELAY   _IOR(LIS331DLH_IOCTL_BASE, 1, int)
#define LIS331DLH_IOCTL_SET_ENABLE  _IOW(LIS331DLH_IOCTL_BASE, 2, int)
#define LIS331DLH_IOCTL_GET_ENABLE  _IOR(LIS331DLH_IOCTL_BASE, 3, int)
#define LIS331DLH_IOCTL_SET_G_RANGE _IOW(LIS331DLH_IOCTL_BASE, 4, int)



#define LIS331DLH_G_2G 0x00
#define LIS331DLH_G_4G 0x10
#define LIS331DLH_G_8G 0x30


#ifdef __KERNEL__
struct lis331dlh_platform_data {
  int poll_interval;
  int min_interval;

  u8 g_range;

  u8 axis_map_x;
  u8 axis_map_y;
  u8 axis_map_z;

  u8 negate_x;
  u8 negate_y;
  u8 negate_z;

  int (*init)(void);
  void (*exit)(void);
  int (*power_on)(void);
  int (*power_off)(void);

};

#endif /* __KERNEL__ */

#endif  /* __LIS331DLH__ */


