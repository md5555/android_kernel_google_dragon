#ifndef __ACERHK_H__
#define __ACERHK_H__

#include <linux/ioctl.h>

#define ACERHK_MINOR   MISC_DYNAMIC_MINOR

#define ACERHK_GET_KEYCOUNT      _IOR('p', 0x01, char)  /* Get number of cached key presses */ 
#define ACERHK_GET_KEYID         _IOR('p', 0x02, char)  /* Get first key in queue */ 
#define ACERHK_CONNECT           _IO('p', 0x03)         /* ? */ 
#define ACERHK_DISCONNECT        _IO('p', 0x04)         /* ? */ 
#define ACERHK_GET_THERMAL_EVENT _IOR('p', 0x05, short) /* ? */ 
#define ACERHK_MAIL_LED_OFF      _IO('p', 0x10)         /* switch mail LED off */
#define ACERHK_MAIL_LED_ON       _IO('p', 0x11)         /* switch mail LED on (blinking) */
#define ACERHK_START_POLLING     _IO('p', 0x12)         /* poll keys in kernel, send real key events */
#define ACERHK_STOP_POLLING      _IO('p', 0x13)         /* stop key polling in kernel */
#define ACERHK_GET_KEY_MAP       _IOR('p', 0x20, int)  /* Get mapping of key names to key events, */ 
#define ACERHK_SET_KEY_MAP       _IOW('p', 0x21, int)  /* Set mapping of key names to key events */ 

/* all possible keys (known to me) */
typedef enum e_key_names {
  k_none      =  0,
  k_help      =  1, /* Fn+F1 */
  k_setup     =  2, /* Fn+F2 */
  k_p1        =  3,
  k_p2        =  4,
  k_p3        =  5,
  k_www       =  6,
  k_mail      =  7,
  k_wireless  =  8,
  k_power     =  9, /* Fn+F3 */
  k_mute      = 10, /* Fn+F8 */
  k_volup     = 11, /* Fn+Up */
  k_voldn     = 12, /* Fn+Down */
  k_res       = 13, /* resolution change on Medion MD 40100 */
  k_close     = 14, /* if lid is closed in tablet mode */
  k_open      = 15, /* if lid is opend in tablet mode */
  k_wireless2 = 16, /* second wireless button on TM 243LC */
  k_play      = 17, /* Play/Pause found on AOpen */
  k_stop      = 18, /* Stop/Eject found on AOpen */
  k_prev      = 19, /* Prev found on AOpen */
  k_next      = 20, /* Next found on AOpen */
  k_display   = 21  /* Change internal/external display on MD 42200 */
} t_key_names;
#define NR_KEY_NAMES 22
typedef unsigned int t_map_name2event[NR_KEY_NAMES];

#ifdef __KERNEL__

/* available features */
#define TM_F_WLAN_EC1     0x00000010
#define TM_F_BLUE_EC1     0x00000020
#define TM_F_WLAN_EC2     0x00000040
#define TM_F_BLUE_EC2     0x00000080
#define TM_F_MUTE_LED_EC  0x00001000
#define TM_F_MAIL_LED     0x00010000
#define TM_F_MAIL_LED_EC  0x00020000
#define TM_F_MAIL_LED_EC2 0x00040000
#define TM_F_MAIL_LED_EC3 0x00080000

#define TM_F_CONNECT      0x00100000
#define TM_F_THERMAL      0x00200000
#define TM_F_PBUTTON      0x00400000
#define TM_F_WBUTTON      0x00800000

typedef enum acer_type {
  TM_unknown,
  /* 200, 210, 520, 600 and 730 series, Medion MD42200 */
  TM_old,
  /* C100, C110, 220, 230, 240, 260, 350, 360, 610, 620, 630, 740 series
     Medion MD40100, Aspire 1600, FS Amilo */
  TM_new,
  /* Aspire 13xx, 14xx, 1700, TM 290, 650, 660, 800 */
  TM_dritek
} t_acer_type;

struct register_buffer {
  unsigned int eax;
  unsigned int ebx;
  unsigned int ecx;
  unsigned int edx;
  unsigned int edi;
  unsigned int esi;
  unsigned int ebp;
};

typedef asmlinkage void (*bios_call) (struct register_buffer *);

#endif

#endif
