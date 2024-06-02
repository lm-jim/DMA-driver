enum buffer_index
{
	DATA_A = 0,
	DATA_B = 1,
	DATA_C = 2,
	DATA_D = 3
};

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

#define BYTES_4K 4*1024

#define IOC_MAGIC_DMA 0xD1

#define DMA_INIT				_IOWR(IOC_MAGIC_DMA, 	150, uint32_t)
#define DMA_PRINT_BUFFER 			_IOWR(IOC_MAGIC_DMA, 	151, uint32_t)
#define DMA_ADDR_CONF				_IOWR(IOC_MAGIC_DMA, 	152, DmaAddrConfig*)
#define DMA_READ_DATA_A				_IOWR(IOC_MAGIC_DMA, 	153, DmaReadPtrInfo*)
#define DMA_READ_DATA_B				_IOWR(IOC_MAGIC_DMA, 	154, DmaReadPtrInfo*)
#define DMA_READ_DATA_C				_IOWR(IOC_MAGIC_DMA, 	155, DmaReadPtrInfo*)
#define DMA_READ_DATA_D				_IOWR(IOC_MAGIC_DMA, 	156, DmaReadPtrInfo*)

/*Definición de IOCTLS*/

#define IOC_MAGIC_FGPA_SIM 0xD2

#define FPGA_PRINT_BUFFER			_IOWR(IOC_MAGIC_FGPA_SIM, 	270, uint32_t)
#define FPGA_WRITE_DATA_A			_IOWR(IOC_MAGIC_FGPA_SIM, 	271, uint32_t)
#define FPGA_WRITE_DATA_B			_IOWR(IOC_MAGIC_FGPA_SIM, 	272, uint32_t)
#define FPGA_WRITE_DATA_C			_IOWR(IOC_MAGIC_FGPA_SIM, 	273, uint32_t)
#define FPGA_WRITE_DATA_D			_IOWR(IOC_MAGIC_FGPA_SIM, 	274, uint32_t)



