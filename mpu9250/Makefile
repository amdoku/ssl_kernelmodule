modulename := mpu9250

obj-m += $(modulename).o
# For multible files per module.ko specify like this:
# module-objs := file1.o file2.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

deploy:
	scp $(modulename).ko "$(DEPLOYSSH):$(DEPLOYSSHPATH)/$(modulename).ko"
	ssh $(DEPLOYSSH) "rmmod $(modulename)"
	ssh $(DEPLOYSSH) "insmod $(DEPLOYSSHPATH)/$(modulename).ko"

