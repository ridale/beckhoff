# Makefile for the beckhoff kernel driver
#
#/////////// 2.4 version makefile //////////////
#KERNELDIR = /home/richard/kernel/linux-2.6.13.4
#HDRS = $(KERNELDIR)/include
#
#include $(KERNELDIR)/.config
#
#CFLAGS =  -O2 -Wall -D__KERNEL__ -DMODULE -I$(HDRS)
#
#all: bhkldrv.o
#bhkldrv.o:
#	gcc -c -o bhkldrv.o bhkldrv.c $(CFLAGS)
#
#clean:
#	rm -f bhkldrv.o

#//////////// 2.6 version makefile /////////////
ifneq ($(KERNELRELEASE),)
obj-m:= bhkldrv.o
else
KDIR:= /lib/modules/$(shell uname -r)/build
PWD:= $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
endif


