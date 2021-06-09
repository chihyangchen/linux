#!/bin/sh
BUILDSH=make-aarch64-kernel.sh
CONF=cn9130_crb_defconfig
OUTPUT=build/cn913x
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
