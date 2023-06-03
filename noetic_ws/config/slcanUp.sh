#!/bin/bash

HELP_MSG="\nScript usage:\tslcanUp [VALUE]\n
Where [VALUE] has possible options
\t0 - 10 Kbps
\t1 - 20 Kbps
\t2 - 50 Kbps
\t3 - 100 Kbps
\t4 - 125 Kbps
\t5 - 250 Kbps
\t6 - 500 Kbps
\t7 - 800 Kbps
\t8 - 1 Mbps"

for arg in "$@"
do
    case $arg in
        -h|--help)
            echo "$HELP_MSG"
            exit 0
            ;;
        -*)
            echo "WRONG OPTION: try ./test [-h|--help]"
            shift
            exit 1
            ;;
        *)
            OTHER_ARGUMENTS+=("$1")
            shift
            ;;
    esac
done

ARG_NUM=${#OTHER_ARGUMENTS[@]}
if [ $ARG_NUM -gt "1" ]
then
    echo "Too many arguments\ntry slcanUp [-h|--help]"
    return 1
fi

sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan


DEVICE=$(find /dev/serial/by-id/ -name '*USBtin*')
#DEVICE=${$(find /dev/serial/by-id/ -name '*USBtin*')##*/}
echo "Binding can port on device ${DEVICE}"

if [ -z $OTHER_ARGUMENTS ]
then
	BAUD=5
	echo "No input provided using standard baud rate BAUD=5"
else
	BAUD=$OTHER_ARGUMENTS
	echo "Baud rate selected: $BAUD"
fi

sudo slcan_attach -f -s ${BAUD} -o ${DEVICE}
sudo slcand ${DEVICE} slcan0
sudo ifconfig slcan0 up
echo
ifconfig -v | grep -A 5 can0
