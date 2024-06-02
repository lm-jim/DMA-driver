#include <fcntl.h>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdio>
#include <sys/ioctl.h>

#include "ioctl_sim.h"

void imprimir_dmaAddrConf(DmaAddrConfig dmaConfSt);
int enviar_configuracion_fpga(DmaAddrConfig dmaConfSt);
void imprimir_bufferDatos(DmaReadPtrInfo dmaReadPtr);
int sim_DATA_D_flow();

static int fd_dma, fd_fpga;
static DmaAddrConfig dmaAddrConf;
static DmaReadPtrInfo dmaReadPtr;

int main(int argc, char* argv[]) {
	/** We define the file descriptor with which we will open the dma device, and the structure from which we will read the information **/
	
	/** Initialize read buffer **/
	dmaReadPtr.buffer = (char*) malloc(BYTES_4K);

	// O_RDWR Open device in read and write mode
	// O_SYNC Open device in synchronization mode so changes are displayed in real time

	if ((fd_dma = open("/dev/dma0", O_RDWR | O_SYNC)) == -1) {
		std::cout << "Error opening device /dev/dma0" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "Descriptor /dev/dma0 opened successfully" << std::endl;
	}

	if ((fd_fpga = open("/dev/fpga_sim0", O_RDWR | O_SYNC)) == -1) {
		std::cout << "Error opening device /dev/fpga_sim0" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "Descriptor /dev/fpga_sim0 opened successfully" << std::endl;
	}

	char instruccion = *argv[1];
	int retorno = -1;

	switch(instruccion){

		/* DMA DRIVER COMMANDS */
		case 'i':
			retorno = ioctl(fd_dma, DMA_INIT, 0);
			break;
		case 'p':
			retorno = ioctl(fd_dma, DMA_PRINT_BUFFER, 0);
			break;
		case 'r':
			retorno = ioctl(fd_dma, DMA_ADDR_CONF, &dmaAddrConf);
			imprimir_dmaAddrConf(dmaAddrConf);
			break;
		case 'a':
			retorno = ioctl(fd_dma, DMA_READ_DATA_A, &dmaReadPtr);
			imprimir_bufferDatos(dmaReadPtr);
			break;
		case 'b':
			retorno = ioctl(fd_dma, DMA_READ_DATA_B, &dmaReadPtr);
			imprimir_bufferDatos(dmaReadPtr);
			break;
		case 'c':
			retorno = ioctl(fd_dma, DMA_READ_DATA_C, &dmaReadPtr);
			imprimir_bufferDatos(dmaReadPtr);
			break;
		case 'd':
			retorno = sim_DATA_D_flow();			
			break;

		/* FPGA SIM COMMANDS */
		case '0':
			retorno = ioctl(fd_fpga, FPGA_PRINT_BUFFER, 0);
			break;
		case '1':
			retorno = ioctl(fd_fpga, FPGA_WRITE_DATA_A, &dmaReadPtr);
			break;
		case '2':
			retorno = ioctl(fd_fpga, FPGA_WRITE_DATA_B, &dmaReadPtr);
			break;
		case '3':
			retorno = ioctl(fd_fpga, FPGA_WRITE_DATA_C, &dmaReadPtr);
			break;
		case '4':
			retorno = ioctl(fd_fpga, FPGA_WRITE_DATA_D, &dmaReadPtr);
			break;
		default:
			std::cout << "Invalid argument" << std::endl;
			break;
	}

	close(fd_dma);
	close(fd_fpga);
	free(dmaReadPtr.buffer);
}

void imprimir_dmaAddrConf(DmaAddrConfig dmaConfSt){

	int32_t bufferIndex;

	sleep(0.1);

	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		printf("RECEIVED: DATA BUFFER[%i]: %llx to %llx\n", bufferIndex, 
		(long long unsigned int)dmaConfSt.baseAddrData[bufferIndex], (long long unsigned int)dmaConfSt.highAddrData[bufferIndex]);

		printf("RECEIVED: DESCRIPTOR BUFFER[%i]: %llx to %llx\n", bufferIndex, 
		(long long unsigned int)dmaConfSt.baseAddrDesc[bufferIndex], (long long unsigned int)dmaConfSt.highAddrDesc[bufferIndex]);

		printf("RECEIVED: CONTROL BUFFER[%i]: %llx\n\n", bufferIndex, 
		(long long unsigned int)dmaConfSt.baseAddrCtl[bufferIndex], (long long unsigned int)dmaConfSt.highAddrDesc[bufferIndex]);
	}
	
}

void imprimir_bufferDatos(DmaReadPtrInfo dmaReadPtr){
	
	int i;

	sleep(0.1);
	
	printf("DATOS LEÍDOS: Buffer=");

	for (i = 0; i < ((dmaReadPtr.bytes > 128) ? 128 : dmaReadPtr.bytes); i++)
		printf("%02x ", *(dmaReadPtr.buffer + i));
	
	printf(" \nSizeBytes=%d\n", dmaReadPtr.bytes);
	
}

int sim_DATA_D_flow(){

	std::cout << "Waiting for information from DATA D..." << std::endl;
	for(int i = 0; i < 16;)
	{
		ioctl(fd_dma, DMA_READ_DATA_D, &dmaReadPtr);
		if(dmaReadPtr.bytes > 0)
		{
			imprimir_bufferDatos(dmaReadPtr);
			i++;
		}
		dmaReadPtr.bytes = 0;
		sleep(0.1);
		
	}
	
	return 0;
}

