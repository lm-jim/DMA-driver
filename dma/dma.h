#ifndef __DMA_H
#define __DMA_H

#include <linux/kernel.h>
#include <linux/fs.h>
#include "structures.h"

#define NUM_DESCRIPTORS 4
#define FW_WORD_SIZE 16 // bytes
#define BYTES_4K 4*1024

#define DATA_POINTER_MASK 0xFFFFFFFF
#define SIZE_MASK 0xFFFFFF00000000
#define ERROR_MASK 0xFF00000000000000 // 1 OK, 0 ERROR

/* Driver control definitions */
#define DMA_MAX_MINOR 1
#define DMA_MAJOR 344
#define DMA_NAME "dma"

/* IOCTL definitions */

#define IOC_MAGIC_DMA 0xD1

#define DMA_INIT				_IOWR(IOC_MAGIC_DMA, 	150, uint32_t)
#define DMA_PRINT_BUFFER 			_IOWR(IOC_MAGIC_DMA, 	151, uint32_t)
#define DMA_ADDR_CONF				_IOWR(IOC_MAGIC_DMA, 	152, DmaAddrConfig*)
#define DMA_READ_DATA_A				_IOWR(IOC_MAGIC_DMA, 	153, DmaReadPtrInfo*)
#define DMA_READ_DATA_B				_IOWR(IOC_MAGIC_DMA, 	154, DmaReadPtrInfo*)
#define DMA_READ_DATA_C				_IOWR(IOC_MAGIC_DMA, 	155, DmaReadPtrInfo*)
#define DMA_READ_DATA_D				_IOWR(IOC_MAGIC_DMA, 	156, DmaReadPtrInfo*)

#endif
