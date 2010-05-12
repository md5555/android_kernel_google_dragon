export WORKAREA="`pwd`/.."
export ANDROID_ENV=Y
export ATH_PLATFORM=NATIVEMMC-SDIO

while [ "$#" -eq 0 ]
do
	echo "Need argument"
	exit -1
done

case $1 in
	1)
		echo "******************"
		echo "* Insert drivers *"
		echo "******************"
		${WORKAREA}/host/support/loadAR6000.sh
		;;
	2)
		${WORKAREA}/host/support/loadAR6000.sh unloadall
		;;
	3)
		echo "********************"
		echo "* Config interface *"
		echo "********************"
		ifconfig eth1 up
		ifconfig eth1 192.168.0.88
#		ifconfig eth1 192.168.11.88
		ifconfig
		;;
	4)
		echo "************"
		echo "* Scan APs *"
		echo "************"
		cp libiw.so.28 /lib/
		cp libstdc++.so.6 /lib/
		host/.output/${ATH_PLATFORM}/image/wmiconfig -i eth1 --filter=all
		sleep 1
		./iwlist eth1 scanning
		;;
	5)
		echo "*****************"
		echo "* Connect to AP *"
		echo "*****************"
		./iwconfig eth1 essid wa_si_AP
#		./iwconfig eth1 essid Buffalo
		sleep 2
		./iwconfig
		;;
	6)
		echo "************************"
		echo "* Ping AP(192.168.0.1) *"
		echo "************************"
		ping 192.168.0.1
#		ping 192.168.11.1
		;;
	7)
		echo "*************"
		echo "* Load TCMD *"
		echo "*************"
		${WORKAREA}/host/support/loadtestcmd.sh
		;;
	8)
		echo "****************************"
		echo "* TCMD continuous TX start *"
		echo "****************************"
		host/.output/${ATH_PLATFORM}/image/athtestcmd -i eth1 --tx tx99 --txfreq 2412 --txrate 11
		;;
	9)
		echo "**************************"
		echo "* TCMD continuous TX off *"
		echo "**************************"
		host/.output/${ATH_PLATFORM}/image/athtestcmd -i eth1 --tx off
		;;
	10)
		echo "******************************************"
		echo "* Insert driver for open source SD stack *"
		echo "******************************************"
		;;
	*)
		echo "Unsupported argument"
		exit -1
esac
