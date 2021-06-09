#!/bin/sh
BUILDSH=make-aarch64-kernel.sh
CONF=cn9130_moxa_tau_defconfig
OUTPUT=build/cn9130_moxa_tau
ROOTDIR=$0
ROOTDIR=${ROOTDIR%/*}
. $ROOTDIR/func.sh
exit 0
