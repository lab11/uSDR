
KERNELDIR	:=  $(INSTALL_ROOT)/linux

SDR_MOD		= sdr_m2s

obj-m		+= $(SDR_MOD).o

modules		:
	make -C $(KERNELDIR) M=`pwd` modules CFLAGS="" LDFLAGS=""
	cp $(SDR_MOD).ko ~/samba_share/nfs/m2s

clean:
	make -C $(KERNELDIR) M=`pwd` clean
	rm -f modules.order
	rm -f *.cmd
	rm -f *.o
	rm -f *.ko
	rm -f *.mod.c
