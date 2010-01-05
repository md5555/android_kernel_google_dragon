build_arch	= i386
header_arch	= x86
asm_link	= x86
defconfig	= defconfig
flavours        = chromeos-intel-menlow chromium-i386
build_image	= bzImage
kernel_file	= arch/$(build_arch)/boot/bzImage
install_file	= vmlinuz
disable_d_i	= yes
no_dumpfile	= yes

generic-pae_sub	= virtual

loader		= grub
