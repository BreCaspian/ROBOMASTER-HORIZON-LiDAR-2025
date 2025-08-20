#!/bin/sh
DIRNAME=`dirname $0`
PWD=`pwd`

PIDFILE="/var/run/MvFGLogServer.pid"

USER_ID=`id -u`
# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "LogServer can only be Removed by root user or sudoer"
exit 1
fi

if [ -f "/var/tmp/log/MVFG/MvFGSdkLog/MvFGSDK_00.log" ]; then
        chattr -a /var/tmp/log/MVFG/MvFGSdkLog/MvFGSDK_00.log
fi

if [ -f "/var/tmp/log/MVFG/MvFGSdkLog/MvFGSDK_01.log" ]; then
        chattr -a /var/tmp/log/MVFG/MvFGSdkLog/MvFGSDK_01.log
fi

# ARCH=$(uname -m | sed 's/x86_//;s/i[3-6]86/32/')

if [ -f /etc/debian_version ]; then
    OS=Debian  # XXX or Ubuntu??
    VER=$(cat /etc/debian_version)
else
    OS=$(uname -s)
    VER=$(uname -r)
fi

service MvFGLogServer stop  2> /dev/null

if [ "Debian" = "$OS" ]; then
	update-rc.d -f MvFGLogServer remove 1> /dev/null
else
	echo "Can not recognize system"
fi

rm -f /etc/init.d/MvFGLogServer

rm -f $PIDFILE

