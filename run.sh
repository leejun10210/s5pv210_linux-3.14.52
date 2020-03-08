
#!/bin/bash

if [ $1 == "install" ];then
	cp -rf arch/arm/boot/zImage /tftpboot/
	cp -rf arch/arm/boot/zImage /mnt/hgfs/shared/tftpboot/
	exit
elif [ $1 == "cp" ];then
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/configs/s5pv210_defconfig  arch/arm/configs/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/plat-samsung/devs.c  arch/arm/plat-samsung/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/plat-samsung/Kconfig  arch/arm/plat-samsung/

	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/include/mach/map.h  arch/arm/mach-s5pv210/include/mach/

	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/setup-sdhci-gpio.c  arch/arm/mach-s5pv210/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/common.c  arch/arm/mach-s5pv210/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/Kconfig  arch/arm/mach-s5pv210/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/Makefile  arch/arm/mach-s5pv210/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/mach-smdkv210.c  arch/arm/mach-s5pv210/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/plat-samsung/include/plat/iic-core.h arch/arm/plat-samsung/include/plat/

	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/plat-samsung/include/plat/devs.h arch/arm/plat-samsung/include/plat/
	cp -rf /mnt/hgfs/shared/linux-3.14.52/include/linux/platform_data/s3c-hsotg.h  include/linux/platform_data/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/usb/host/ehci-exynos.c  drivers/usb/host/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/arch/arm/mach-s5pv210/setup-usb-phy.c  arch/arm/mach-s5pv210/

#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/usb/host/Makefile  drivers/usb/host/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/usb/host/Kconfig  drivers/usb/host/


#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/mmc/host/sdhci-s3c.c  drivers/mmc/host/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/mmc/core/host.c  drivers/mmc/core/

#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/video/backlight/platform_lcd.c drivers/video/backlight/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/video/s3c-fb.c drivers/video/

#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/input/misc/kxtj9.c drivers/input/misc/


#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/crypto/Kconfig drivers/crypto/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/crypto/Makefile drivers/crypto/

#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/input/touchscreen/gslX680.* drivers/input/touchscreen/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/input/touchscreen/Makefile drivers/input/touchscreen/
#	cp -rf /mnt/hgfs/shared/linux-3.14.52/drivers/input/touchscreen/Kconfig drivers/input/touchscreen/

	make ARCH=arm s5pv210_defconfig
elif [ $1 == "clean" ];then
	make ARCH=arm distclean
	make ARCH=arm s5pv210_defconfig
#	make ARCH=arm s5pc100_defconfig
else
	echo "only comoile"	
fi
#	make ARCH=arm CROSS_COMPILE=/home/lincor/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi- spl/u-boot-spl.bin
#	make ARCH=arm CROSS_COMPILE=/home/lincor/toolchains/arm-2009q3/bin/arm-none-linux-gnueabi-
#	make ARCH=arm CROSS_COMPILE=/home/lincor/toolchains/gcc-arm-8.3-2019.03-x86_64-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
	make ARCH=arm CROSS_COMPILE=/home/lincor/toolchains/fsl-linaro-toolchain/bin/arm-none-linux-gnueabi-
#	make ARCH=arm CROSS_COMPILE=/home/lincor/toolchains/fsl-linaro-toolchain/bin/arm-none-linux-gnueabi- 
