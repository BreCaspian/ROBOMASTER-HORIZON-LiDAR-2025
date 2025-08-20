#!/bin/sh

################################################################################
#
# load.sh
# for gev/cxp/cmlframegrabber.ko load
#
################################################################################

# Variables
DIRNAME=$(cd `dirname $0`; pwd)
DRIVER_HOME=${DIRNAME}
HOST_VERSION=`uname -r`
HOST_ARCH=`uname -m | sed -e 's/i.86/i686/' -e 's/^armv.*/arm/'`

# pcie array
PCIE_ARRAY=("gev" "cxp" "xof" "cml" "virtualserial")

#Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    load.sh - Load For Ethernet driver module "
    echo "              gevframegrabber.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash load.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Load For Ethernet module and configure the system to"
    echo "    be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

#Print out the error and exit 
# $1 Error message
# $2 Exit code
ErrorOut()
{
	echo ""
	echo "Error: $1"
	echo ""
	exit $2
}

# Parse the input arguments
for i in $*
do
    case $i in        
        --help)
            DisplayHelp
            exit 0
        ;;
        *)
        # unknown option
        DisplayHelp
        exit 1
        ;;
    esac
done

# Check required priviledge
if [ `whoami` != root ]; then
	ErrorOut "This script can only be run by root user or sudoer" 1
fi

# Sanity check
if [ "`uname -m`" = "x86_64" ]\
   || [ "`uname -m`" = "i386" ]\
   || [ "`uname -m`" = "aarch64" ]\
   || [ "`uname -m`" = "armv7l" ]\
   || [ "`uname -m`" = "i686" ]
then
	echo "The arch is: `uname -m`"
else
	ErrorOut "Driver Not Support the arch `uname -m` " 1
fi


for pcie in ${PCIE_ARRAY[@]} 
do
    MODULE_NAME=${pcie}framegrabber

	if [ $pcie = "virtualserial" ]; then
		MODULE_NAME=mvfg${pcie}
	fi

    # Do not re-load if not needed
    IMAGEFILTER_LOADED=`lsmod | grep -o ${MODULE_NAME}`
    if [ "$IMAGEFILTER_LOADED" = "${MODULE_NAME}" ];then
	    continue
    fi

    if [ ! -f $DRIVER_HOME/${MODULE_NAME}.ko ]; then
	    echo "${MODULE_NAME}.ko isn't build yet " 
        continue
    fi

    # Load the module
	if [ $pcie = "virtualserial" ]; then
		echo "Loading MVFG Virtual Serial for `uname -m` ..."
	else
		echo "Loading MVFG ${pcie} Frame Grabber for `uname -m` ..."
	fi
    
    /sbin/insmod $DRIVER_HOME/${MODULE_NAME}.ko
    if [ $? -ne 0 ]; then
        echo "[ERROR]:Install ${MODULE_NAME} failed"
        continue
    fi

    echo "Install ${MODULE_NAME} complete!"
done

sleep 1

chmod 777 /dev/mvfg*
chmod 777 /dev/ttyvirserial*

echo "end..."

