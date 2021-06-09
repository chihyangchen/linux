#!/bin/sh
BUILDSH=make-armhf-kernel.sh
CONF=am437x_defconfig
OUTPUT=build/am437x
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
