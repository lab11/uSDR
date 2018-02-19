
CFLAGS		:= -Os -mcpu=cortex-m3 -mthumb -I $(INSTALL_ROOT)/A2F/root/usr/include
LDFLAGS		:= -mcpu=cortex-m3 -mthumb -L $(INSTALL_ROOT)/A2F/root/usr/lib

CC		= $(CROSS_COMPILE_APPS)gcc

#EXEC = rx_only 
#EXEC = led_counter
#EXEC = app_interrupt
#EXEC = RGB_rx
#EXEC = RGB_single_tx
EXEC = RGB_sweep

all: $(EXEC)
	cp $(EXEC) ~/samba_share/nfs
	chmod +x ~/samba_share/nfs/$(EXEC)

rx_only: rx_only.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

app_interrupt: app_interrupt.o include/mss_sdr.o

led_counter: led_counter.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

RGB_rx: RGB_rx.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

RGB_tx: RGB_tx.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

RGB_single_tx: RGB_single_tx.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

RGB_sweep: RGB_sweep.o include/maxim2831.o include/tx_packet.o include/rx_packet.o include/radio_config.o include/mss_sdr.o

include/maxim2831.c: include/maxim2831.h

include/tx_packet.c: include/tx_packet.h include/radio_config.h

include/rx_packet.c: include/rx_packet.h

include/radio_config.c: include/radio_config.h include/maxim2831.h include/rx_packet.h

include/mss_sdr.c: include/mss_sdr.h

DIRS := ${shell find ${ROOT_DIRECTORY} -mindepth 1 -maxdepth 1 -type d -print | grep -v svn}
clean:
	-rm -f $(EXEC) *.gdb *.o
	rm -f ~/samba_share/nfs/$(EXEC)
	for dir in $(DIRS) ; do \
		cd $$dir; \
		rm -f ./*.gdb; \
		rm -f ./*.o; \
		cd ..; \
	done

.PHONY: all
