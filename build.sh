#!/bin/sh

TOOLCHAIN="arm-none-eabi-"
CORES=8
VERSION="Test-`date '+%Y%m%d-%H%M'`"

export KBUILD_BUILD_VERSION=$VERSION
export ARCH=arm
export CROSS_COMPILE=$TOOLCHAIN
export CC="${CROSS_COMPILE}gcc"

if [ "$1" = "config" ]; then
	make menuconfig ARCH=arm CROSS_COMPILE=$TOOLCHAIN
	exit
fi

echo "Compiling the kernel ($VERSION)..."
rm -f arch/arm/boot/zImage
make -j$CORES CROSS_COMPILE=$TOOLCHAIN ARCH=arm KALLSYMS_EXTRA_PASS=1

if test -f "arch/arm/boot/zImage"; then
	OUTFILE=$VERSION-zImage
    echo "  OBJCOPY $OUTFILE"
	cp arch/arm/boot/zImage ./$OUTFILE
    echo "  CONFIG  config-$VERSION"
	cat .config | grep -v "is not set" | grep -v "^#" > ./config-$VERSION

else
	echo "Make failed to produce zImage"
fi

echo "Done"
