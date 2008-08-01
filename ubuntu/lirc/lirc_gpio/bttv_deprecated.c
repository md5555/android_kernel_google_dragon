//Temporarily added until a resolution is reached upstream
//For kernel 2.6.22

#include <linux/version.h>
#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,22)
#include "extra_2.6.22/bttv.h"
#include "extra_2.6.22/bttvp.h"
#else
#include "../drivers/media/video/bt8xx/bttv.h"
#include "../drivers/media/video/bt8xx/bttvp.h"
#endif

struct bttv bttvs[BTTV_MAX];
unsigned int bttv_debug;
unsigned int bttv_num; /* number of Bt848s in use */

int bttv_get_cardinfo(unsigned int card, int *type, unsigned *cardid)
{
    printk("The bttv_* interface is obsolete and will go away,\n"
           "please use the new, sysfs based interface instead.\n");
    if (card >= bttv_num) {
        return -1;
    }
    *type = bttvs[card].c.type;
    *cardid = bttvs[card].cardid;
    return 0;
}

wait_queue_head_t* bttv_get_gpio_queue(unsigned int card)
{
    struct bttv *btv;

    if (card >= bttv_num) {
        return NULL;
    }

    btv = &bttvs[card];
    if (bttvs[card].shutdown) {
        return NULL;
    }
    return &btv->gpioq;
}


