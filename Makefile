obj-m += fpga_sim/fpga_sim.o
obj-m += dma/dma.o  

KVERSION=$(shell uname -r)

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
	g++ -o ioctl_sim/ioctl_sim ioctl_sim/ioctl_sim.cpp
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
	rm ioctl_sim/ioctl_sim
