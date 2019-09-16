obj-m+=udt1cri_usb.o

KERNEL_UNAME ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KERNEL_UNAME)/build/
SRC := $(shell pwd)
DEPMOD := depmod -a

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

modules_install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
	$(DEPMOD)	

run:
	modprobe --remove udt1cri_usb || true
	$(MAKE) modules_install
	sudo modprobe udt1cri_usb

test:
	g++ -lgtest_main -lgtest -lpthread ./tests/mcba_tests.cpp -o ./tests/mcba_tests
	./tests/mcba_tests --gtest_break_on_failure

