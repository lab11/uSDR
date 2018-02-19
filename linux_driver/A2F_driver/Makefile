
all:
	$(MAKE) -C kernel/
	$(MAKE) -C user_app/

clean:
	$(MAKE) -C kernel/ clean
	$(MAKE) -C user_app/ clean

backup:
	cp -fr * ~/Projects/sdr/linux_driver

.PHONY: all
