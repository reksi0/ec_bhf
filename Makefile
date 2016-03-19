obj-m		:= ec_bhf.o
KERNELDIR	?= /tmp/kernel_dir
PWD       	:= $(shell pwd)
CROSS_PREFIX 	?= arm-linux-gnueabihf-

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=${CROSS_PREFIX}

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

