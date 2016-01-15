#!/bin/sh
lzma -z -f -k arch/arm64/boot/Image
scripts/generate-its-script.sh arch/arm64/boot/Image.lzma arch/arm64/boot/dts/tegra/*smaug*.dtb | dtc -I dts -O dtb -p 1024 > Image.fit

