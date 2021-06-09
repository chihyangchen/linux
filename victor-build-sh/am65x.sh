#!/bin/sh
BUILDSH=make-aarch64-kernel.sh
CONF=tisdk_am65xx-evm-rt_defconfig
OUTPUT=build/am65x
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
