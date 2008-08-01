//Temporarily re-added for Ubuntu Kernel 2.6.22 until upstream
//Resolution is reached.
extern wait_queue_head_t* bttv_get_gpio_queue(unsigned int card);
extern int bttv_get_cardinfo(unsigned int card, int *type,
                 unsigned int *cardid);

