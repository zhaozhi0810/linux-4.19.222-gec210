#!/bin/sh
make -j4

if [ $? -eq 0 ]
then
	cp arch/arm/boot/zImage /mnt/hgfs/vm_share/tftp/
	cp arch/arm/boot/dts/s5pv210-smdkv210.dtb /mnt/hgfs/vm_share/tftp/
fi



