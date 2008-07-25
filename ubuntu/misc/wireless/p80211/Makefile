obj-$(CONFIG_P80211) := p80211.o

p80211-objs := p80211mod.o \
		p80211conv.o \
		p80211req.o \
		p80211wep.o \
		p80211wext.o \
		p80211netdev.o

EXTRA_CFLAGS += -DWLAN_TXMGMT_MINBRATE -DRSN_INTERNAL_ROAM \
	-DWLAN_HOSTIF=WLAN_NONE -I$(src)
