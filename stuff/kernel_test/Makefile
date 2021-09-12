TARGET_MODULE=bcm2835_smi_fb
obj-m+=$(TARGET_MODULE).o


all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=/home/pi/kernel_test modules

fb-overlay:
	dtc -I dts -O dtb -o smi-fb.dtbo smi-fb.dts

#smi-overlay:
#	dtc -I dts -O dtb -o smi-custom.dtbo smi-custom.dts

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=/home/pi/kernel_test clean
	rm test

load:
	../startup/boot.sh
	dtoverlay smi-fb.dtbo
	insmod $(TARGET_MODULE).ko
#	dtoverlay smi-fb.dtbo

unload:
	rmmod $(TARGET_MODULE).ko
	dtoverlay -r smi-fb

modules_install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
	$(DEPMOD)  
