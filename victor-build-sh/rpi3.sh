#!/bin/sh
BUILDSH=make-aarch64-kernel.sh
CONF=bcmrpi3_defconfig
OUTPUT=build/rpi3
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
