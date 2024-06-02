#!/bin/sh

start()
{
	echo "Loading DMA driver into Kernel"
	sudo insmod ./dma/dma.ko
	sudo mknod /dev/dma0 c 344 0

	echo "Loading FPGA simulator into Kernel"
	sudo insmod ./fpga_sim/fpga_sim.ko
	sudo mknod /dev/fpga_sim0 c 345 0
}

stop()
{
	echo "Unoading FPGA simulator"
	sudo rmmod fpga_sim
	sudo rm /dev/fpga_sim0

	echo "Unoading DMA driver"
	sudo rmmod dma
	sudo rm /dev/dma0
}

restart ()
{
	stop
	start
}

case "$1" in
    stop) stop ;;
    status) status ;;
    start) start ;;
    restart) restart;;
esac








