#!/bin/sh
BUILDSH=make-aarch64-kernel.sh
CONF=imx8mq-evk_defconfig
OUTPUT=build/imx8mq
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
