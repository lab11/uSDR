
CFLAGS		:= -Os -mcpu=cortex-m3 -mthumb -I $(INSTALL_ROOT)/A2F/root/usr/include
LDFLAGS		:= -mcpu=cortex-m3 -mthumb -L $(INSTALL_ROOT)/A2F/root/usr/lib

CC		= $(CROSS_COMPILE_APPS)gcc

EXEC = led_counter
#EXEC = user_sample

all: $(EXEC)
	cp $(EXEC) ~/samba_share/nfs/m2s
	chmod +x ~/samba_share/nfs/m2s/$(EXEC)

INCLUDE_PATH = include

USER_DRIVER = mss_pdma
		
OBJS = 	user_driver/$(USER_DRIVER).o \
		$(EXEC).o \
		include/m2s_sdr.o \
		include/maxim2831.o \
		include/tx_packet.o \
		include/rx_packet.o \
		include/radio_config.o

$(EXEC): user_driver/$(USER_DRIVER).o include/sdr_def.h $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS)  -o $(EXEC) -lrt

#$(EXEC).o: $(EXEC).c
#	$(CC) $(CFLAGS) -c -o $(EXEC).o $(EXEC).c

DIRS := ${shell find ${ROOT_DIRECTORY} -mindepth 1 -maxdepth 1 -type d -print | grep -v svn}
clean:
	-rm -f $(EXEC) *.gdb *.o
	rm -f include/*.o
	rm -f user_driver/*.o
	rm -f ~/samba_share/nfs/m2s/$(EXEC)
	for dir in $(DIRS) ; do \
		cd $$dir; \
		rm -f ./*.gdb; \
		rm -f ./*.o; \
		cd ..; \
	done

.PHONY: all
