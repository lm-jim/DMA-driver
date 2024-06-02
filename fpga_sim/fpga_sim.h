#include <linux/kernel.h>
#include <linux/fs.h>
#include "structures.h"

/* Driver control definitions */
#define FPGA_SIM_MAX_MINOR 1
#define FPGA_SIM_MAJOR 345
#define FPGA_SIM_NAME "fpga_sim"

/* IOCTL definitions */

#define IOC_MAGIC_FGPA_SIM 0xD2

#define FPGA_PRINT_BUFFER			_IOWR(IOC_MAGIC_FGPA_SIM, 	270, uint32_t)
#define FPGA_WRITE_DATA_A			_IOWR(IOC_MAGIC_FGPA_SIM, 	271, uint32_t)
#define FPGA_WRITE_DATA_B			_IOWR(IOC_MAGIC_FGPA_SIM, 	272, uint32_t)
#define FPGA_WRITE_DATA_C			_IOWR(IOC_MAGIC_FGPA_SIM, 	273, uint32_t)
#define FPGA_WRITE_DATA_D			_IOWR(IOC_MAGIC_FGPA_SIM, 	274, uint32_t)
