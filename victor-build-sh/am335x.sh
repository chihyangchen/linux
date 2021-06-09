#!/bin/sh
BUILDSH=make-armhf-kernel.sh
CONF=am335x_defconfig
OUTPUT=build/am335x
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
