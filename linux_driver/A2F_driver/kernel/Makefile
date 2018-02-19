
KERNELDIR	:=  $(INSTALL_ROOT)/linux

SDR_MOD		= sdr

obj-m		+= $(SDR_MOD).o

modules		:
	make -C $(KERNELDIR) M=`pwd` modules CFLAGS="" LDFLAGS=""
	cp $(SDR_MOD).ko ~/samba_share/nfs/

clean:
	make -C $(KERNELDIR) M=`pwd` clean
	rm -f modules.order
