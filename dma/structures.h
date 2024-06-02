#ifndef __STRUCTURES_H
#define __STRUCTURES_H

#include <linux/kernel.h>

enum buffer_index
{
	DATA_A = 0,
	DATA_B = 1,
	DATA_C = 2,
	DATA_D = 3
};

typedef struct ParVirtBus{
	char * virt;
	phys_addr_t bus;
} ParVirtBus;

typedef struct DmaAddrConfig
{
	uint32_t baseAddrData[4];
	uint32_t highAddrData[4];
	uint32_t baseAddrDesc[4];
	uint32_t highAddrDesc[4];
	uint32_t baseAddrCtl[4];
} DmaAddrConfig;

typedef struct DmaReadPtrInfo
{
	char* buffer;
	long int bytes;
	uint32_t fdWord1;
	uint32_t fdWord2;
	uint32_t fdWord3;
	uint32_t fdWord4;
} DmaReadPtrInfo;

#endif
