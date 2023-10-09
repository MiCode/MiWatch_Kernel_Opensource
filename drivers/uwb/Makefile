KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

build:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD modules

modules_install: build
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD INSTALL_MOD_STRIP=1 modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD clean

all: build

.PHONY: all build modules_install clean
