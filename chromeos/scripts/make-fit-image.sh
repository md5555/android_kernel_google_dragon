#!/bin/sh

OUT_DIR=$1
KERNEL_DIR=$2
ARCH=$3
DTB_DIR=$4
LZ4_PATH=$5

compression="none"
if [ -d "${LZ4_PATH}" ]; then
	export PATH=${LZ4_PATH}:${PATH}
	compression="lz4"
fi

cd ${OUT_DIR} 

${KERNEL_DIR}/chromeos/scripts/generate-its-script.sh \
	-a ${ARCH} -c ${compression} arch/${ARCH}/boot/Image ${DTB_DIR}/*.dtb \
	| ${OUT_DIR}/scripts/dtc/dtc -I dts -O dtb -p 1024 \
	> ${OUT_DIR}/arch/${ARCH}/boot/Image.fit

