# Makefile

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

ifeq (${KERNELRELEASE},)
    KERNEL_SOURCE := ../linux
    PWD := $(shell pwd)
default:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} modules

clean:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} clean

else
	obj-m := cc2420.o
	CFLAGS_cc2420.o := -DDEBUG -Wall
endif

remote_install:
	scp cc2420.ko pi@raspberrypi.local:
