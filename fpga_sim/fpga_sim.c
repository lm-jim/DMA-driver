#include "dma.h"
#include "fpga_sim.h"
#include "structures.h"
#include "linux/module.h"
#include "linux/time.h"
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/random.h>

/* Pointers to virtual and physical memory */
ParVirtBus  dataBuffer[4];
ParVirtBus  descriptorBuffer[4];
ParVirtBus  controlBuffer[4];

/* We import externally the semaphores from the DMA module. */
long tmo;
extern struct semaphore semaforoIO_DMA[4];

//////////////////////////////////////////////////////
//// DRIVER CONTROL BLOCK AND FUNCTION DEFINITION ////
//////////////////////////////////////////////////////

MODULE_AUTHOR("Luis Miguel Jimenez Aliaga");
MODULE_DESCRIPTION("FPGA simulator for DMA driver");
MODULE_LICENSE("Dual BSD/GPL");

/* Driver control block */
typedef struct
{
	struct cdev* cdev;
	dev_t dev;
	uint32_t major_num;
	uint32_t minor_num;
	uint32_t max_minor;
}dma_dev_cb;

dma_dev_cb driverCB;

/* Kernel module functions */
int32_t fpga_sw_init(void);
static int fpga_init_module(void);
static void fpga_exit_module(void);

/* file_operations functions */
ssize_t fpga_read(struct file *file, char *buff, size_t len, loff_t * offset);
int32_t fpga_open(struct inode *, struct file *);
ssize_t fpga_write(struct file *file, const char *buff, size_t len, loff_t * offset);
int32_t fpga_rls(struct inode*, struct file *);
long fpga_ioctl(struct file *file, uint32_t ioctl_num, unsigned long ioctl_param);

/* ioctl functions */
long printBuffer_ioctl(void);
long write_DATA_A_ioctl(void);
long write_DATA_B_ioctl(void);
long write_DATA_C_ioctl(void);
long write_DATA_D_ioctl(void);

/* Local functions */
long dmaAddrConfig(unsigned long ioctl_param);
void setDescriptorInfo(char* virtDescriptor_pointer, uint8_t err, uint32_t buffSize, uint32_t buffAddr);


static struct file_operations fops =
{
  .read = fpga_read,
  .open = fpga_open,
  .write = fpga_write,
  .release = fpga_rls,
  .unlocked_ioctl = fpga_ioctl
};

//////////////////////////////////////////
//// DRIVER FUNCTIONS | KERNEL MODULE ////
//////////////////////////////////////////

/**
 * fpga_sw_init - Initialize general software structures
 *
 * Function that will initialize the necessary structs to register the driver in the system.
 **/
int32_t fpga_sw_init(void)
{
	int resultado_Op;

	printk(KERN_INFO "Installing FPGA simulator... ");
	
	//Registering Major number and Minor number
	driverCB.dev = MKDEV(FPGA_SIM_MAJOR, 0);
	resultado_Op = register_chrdev_region(driverCB.dev, 1, FPGA_SIM_NAME);

	if (resultado_Op != 0)
	{
		printk(KERN_ALERT "FPGA: Failed to allocate major number\n");
		return resultado_Op;
	}
	else
	{
		//Inserting Major number and Minor number in the driver control block
		driverCB.major_num = MAJOR(driverCB.dev);
		driverCB.minor_num = MINOR(driverCB.dev);
		driverCB.max_minor = FPGA_SIM_MAX_MINOR;

		//Reserving memory for the struct cdev
		driverCB.cdev = cdev_alloc();

		//Link cdev with file_operations and establish this module as the owner
		driverCB.cdev->ops = &fops;
		driverCB.cdev->owner = THIS_MODULE;

		//We proceed to register the driver in the Kernel
		if(cdev_add(driverCB.cdev, driverCB.dev, 1) < 0)
		{
			printk(KERN_ERR "fpga_sim_drv_Install: Driver registration error\n");
			resultado_Op = -1;
		}
	}

	printk((resultado_Op==0) ? KERN_INFO "FPGA simulator module structs OK\n": KERN_ERR "FPGA simulator module structs ERROR\n");

	return resultado_Op;
}

/**
 * fpga_dma_connect - Connection to DMA driver Routine
 *
 * Function that attempts to read the physical addresses given by the DMA driver and stores them for future use.
 **/

int32_t fpga_dma_connect(void){

	DmaAddrConfig dmaAddrConf;
	DmaAddrConfig* dmaAddrConf_pointer = &dmaAddrConf;

	struct file* dmaFile;
	mm_segment_t old_fs;
	int error;

	//We save the current “file segment register” (which is in user space) and put it in kernel space
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	//filp_open: Used to access system files from kernel space.
	dmaFile = filp_open("/dev/dma0", O_RDWR | O_SYNC, 0);

	if(IS_ERR(dmaFile))
	{
		set_fs(old_fs);
		error = PTR_ERR(dmaFile);
		printk(KERN_ALERT "FPGA: Cannot open /dev/dma0. Check if DMA module is loaded. Error %d\n", error);
		return -1;
	}
	else
	{
		printk(KERN_INFO "FPGA: Requesting physical addresses \n");
		dmaFile->f_op->unlocked_ioctl(dmaFile, DMA_ADDR_CONF, (unsigned long) dmaAddrConf_pointer);
		filp_close(dmaFile, NULL);
	}

	//We return the “file segment register” to the initial value after operating on the file 
	set_fs(old_fs);
	
	dmaAddrConfig((unsigned long) dmaAddrConf_pointer);

	return 0;
}

/**
 * fpga_init_module - Driver Registration Routine
 *
 * fpga_init_module is the first routine executed when the module is initialized.
 **/

static int __init
fpga_init_module(void)
{
	int32_t resultado_Op = 0;
	
	printk(KERN_INFO "FPGA Simulator driver initializing...\n");
	
	//Register the driver in the system
	resultado_Op = fpga_sw_init();

	resultado_Op = fpga_dma_connect();

	printk((resultado_Op == 0) ? "FPGA Simulator initialized successfully\n": "Error initializing FGPA simulator module\n");
	
	if(resultado_Op != 0)
	{
		unregister_chrdev_region(driverCB.dev, driverCB.max_minor);
		cdev_del(driverCB.cdev);
	}
	
	tmo = usecs_to_jiffies(200000);
	return resultado_Op;
}

module_init(fpga_init_module);

/**
 * fpga_exit_module - Driver Exit Cleanup Routine
 *
 * fpga_exit_module is called just before removing the driver from memory.
 **/
static void __exit
fpga_exit_module(void)
{
	printk(KERN_INFO "FPGA: Removing module...\n");

	//Freeing the Major and Minor number
	unregister_chrdev_region(driverCB.dev, driverCB.max_minor);

	//Freeing device
	cdev_del(driverCB.cdev);

	printk(KERN_INFO "FPGA Simulator module removed\n");
}

module_exit(fpga_exit_module);

/**
 * fpga_open - Called when the device is opened.
 *
 * Returns 0 on success, negative value on failure
 **/
int32_t fpga_open(struct inode *inod, struct file *fil)
{
	return 0;
}

/**
 * fpga_read - Called when the device is read.
 *
 * Returns 0 on success, negative value on failure
 **/

ssize_t fpga_read(struct file *file, char *buff, size_t len, loff_t * offset)
{
	return 0;
}

/**
 * fpga_write - Called when the device is written.
 *
 * Returns 0 on success, negative value on failure
 **/

ssize_t fpga_write(struct file *file, const char *buff, size_t len, loff_t * offset)
{
	return 0;
}

/**
 * fpga_rls - Called when the device is released.
 *
 * Returns 0 on success, negative value on failure
 **/
int32_t fpga_rls(struct inode *inod, struct file *fil)
{
	return 0;
}

/**
 * fpga_ioctl - Ioctl's of the device.
 *
 * Function called when receiving an ioctl command
 **/
long fpga_ioctl(struct file *file, uint32_t ioctl_num, unsigned long ioctl_param)
{
	long result = 0;
	switch (ioctl_num)
	{
		case FPGA_PRINT_BUFFER:
			result = printBuffer_ioctl();
			break;
		case FPGA_WRITE_DATA_A:
			result = write_DATA_A_ioctl();
			break;
		case FPGA_WRITE_DATA_B:
			result = write_DATA_B_ioctl();
			break;
		case FPGA_WRITE_DATA_C:
			result = write_DATA_C_ioctl();
			break;
		case FPGA_WRITE_DATA_D:
			result = write_DATA_D_ioctl();
			break;
		default:
			printk(KERN_ALERT "FPGA_sim: Unknown ioctl call received.");
			result = -1;
			break;
	}

	return result;
}

/////////////////////////
//// IOCTL FUNCTIONS ////
/////////////////////////

long printBuffer_ioctl(void){
	
	int a;
	int32_t bufferIndex;

	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{

		//Print CONTROL BUFFER contents
		printk(KERN_INFO "CONTROL BUFFER[%i]: %016llx\n", bufferIndex, *((uint64_t*)controlBuffer[bufferIndex].virt));

		//Print DESCRIPTOR BUFFER contents
		if(descriptorBuffer[bufferIndex].virt != 0)
		{
			printk(KERN_INFO "DESCRIPTOR BUFFER[%i]:\n", bufferIndex);
			printk(KERN_INFO "%x: Er   Size        Address\n", (uint32_t)(virt_to_phys(((uint64_t*)descriptorBuffer[bufferIndex].virt))));
			for(a = 0; a < NUM_DESCRIPTORS; a = a + 1)
			{
				printk(KERN_INFO "%x: %02llx    %06llx      %08llx\n", (uint32_t)(virt_to_phys(((uint64_t*)descriptorBuffer[bufferIndex].virt + a * FW_WORD_SIZE))),
						(*((uint64_t*) (descriptorBuffer[bufferIndex].virt + a * FW_WORD_SIZE)) & ERROR_MASK) >> 56,
						(*((uint64_t*) (descriptorBuffer[bufferIndex].virt + a * FW_WORD_SIZE)) & SIZE_MASK) >> 32,
						*((uint64_t*) (descriptorBuffer[bufferIndex].virt + a * FW_WORD_SIZE)) & DATA_POINTER_MASK);
			}
		}

		//Print DATA BUFFER contents
		if(dataBuffer[bufferIndex].virt != 0)
		{
			for(a = 0; a < BYTES_4K; a = a + 8)
			{
				printk(KERN_INFO "%x: %02x %02x %02x %02x  %02x %02x %02x %02x\n", 
						(uint32_t) (virt_to_phys(((uint32_t*) (dataBuffer[bufferIndex].virt + a)))),
						*(dataBuffer[bufferIndex].virt + a),
						*(dataBuffer[bufferIndex].virt + a + 1),
						*(dataBuffer[bufferIndex].virt + a + 2),
						*(dataBuffer[bufferIndex].virt + a + 3),
						*(dataBuffer[bufferIndex].virt + a + 4),
						*(dataBuffer[bufferIndex].virt + a + 5),
						*(dataBuffer[bufferIndex].virt + a + 6),
						*(dataBuffer[bufferIndex].virt + a + 7));
			}
		}

	}
	
	return 0;
}

//Basic example, we fill the buffer of 3 descriptors with numbers from 1 to 3.
long write_DATA_A_ioctl(void){
	
	int i;
	uint32_t regionSize;
	char* direccionVirtual;
	uint32_t direccionFisica;

	if(down_timeout(&semaforoIO_DMA[DATA_A], tmo) == 0){
		
		//Region size, 1 word
		regionSize = FW_WORD_SIZE;

		//Set control buffer and descriptors
		for(i = 0; i < NUM_DESCRIPTORS; i++){

			direccionVirtual = dataBuffer[DATA_A].virt + (regionSize * i);
			direccionFisica = (uint32_t)(virt_to_phys((uint64_t*) direccionVirtual));			
			
			setDescriptorInfo(descriptorBuffer[DATA_A].virt + FW_WORD_SIZE * i, 1, regionSize, direccionFisica);
			
			//For each descriptor, we fill its region with numbers from 1 to 3.
			if(i != NUM_DESCRIPTORS - 1)
				memset(direccionVirtual, i + 1, regionSize);
		}

		//We write in the control buffer the address of the fourth descriptor
		setDescriptorInfo(controlBuffer[DATA_A].virt, 1, regionSize, direccionFisica);

		up(&semaforoIO_DMA[DATA_A]);
	}
	printk(KERN_INFO "FPGA Simulator: Written DATA_A \n");
	return 0;

}

//Same example as above, but with larger size, spacing, and each number will be written every 1 second. It is used to test synchronization.
long write_DATA_B_ioctl(void){

	int i;
	uint32_t regionSize;
	uint32_t regionOffset;
	char* direccionVirtual;
	uint32_t direccionFisica;

	uint32_t sleepTime = usecs_to_jiffies(1000000); //1 second

	if(down_timeout(&semaforoIO_DMA[DATA_B], tmo) == 0){
		
		//Region size, 4 words
		regionSize = FW_WORD_SIZE * 4;

		//Region spacing, 2 words
		regionOffset = FW_WORD_SIZE * 2;

		//Set control buffer and descriptors
		for(i = 0; i < NUM_DESCRIPTORS; i++){

			direccionVirtual = dataBuffer[DATA_B].virt + (regionSize * i)  + (regionOffset * i);
			direccionFisica = (uint32_t)(virt_to_phys((uint64_t*) direccionVirtual));
			
			setDescriptorInfo(descriptorBuffer[DATA_B].virt + FW_WORD_SIZE * i, 1, regionSize, direccionFisica);
			
			//For each descriptor, we fill its region with numbers from 1 to 3.
			if(i != NUM_DESCRIPTORS - 1)
				memset(direccionVirtual, i + 1, regionSize);
			msleep(sleepTime);
		}
		
		//We write in the control buffer the address of the fourth descriptor
		setDescriptorInfo(controlBuffer[DATA_B].virt, 1, regionSize, direccionFisica);

		up(&semaforoIO_DMA[DATA_B]);
	}
	printk(KERN_INFO "FPGA Simulator: Written DATA_B \n");
	return 0;
}

//Example where we have a single descriptor, but its data exceeds the maximum buffer size and goes around the start.
long write_DATA_C_ioctl(void){

	int i;
	uint32_t regionSize;
	char* direccionVirtual;
	uint32_t direccionFisica;

	if(down_timeout(&semaforoIO_DMA[DATA_C], tmo) == 0){
		
		//Region size, 1 word
		regionSize = FW_WORD_SIZE;

		//Set control buffer and descriptors
		direccionVirtual = dataBuffer[DATA_C].virt + BYTES_4K - FW_WORD_SIZE;
		direccionFisica = (uint32_t)(virt_to_phys((uint64_t*) direccionVirtual));
		
		//We set size x2, we are going to insert two regions (as if it were a single region of size 2, but going around at the start).
		setDescriptorInfo(descriptorBuffer[DATA_C].virt, 1, regionSize * 2, direccionFisica);

		//We set the other descriptors to 0
		for(i = 2; i < NUM_DESCRIPTORS; i++)
			setDescriptorInfo(descriptorBuffer[DATA_C].virt + FW_WORD_SIZE * i, 0, 0, 0);
		
		//Except for the second one, we will point it to the “next value” (otherwise we would read 0 in all descriptors and turn around)
		setDescriptorInfo(descriptorBuffer[DATA_C].virt + FW_WORD_SIZE, 1, regionSize, (uint32_t) virt_to_phys((uint64_t*)(dataBuffer[DATA_C].virt + FW_WORD_SIZE * 2)));

		//We write in the control buffer the address of the following descriptor
		setDescriptorInfo(controlBuffer[DATA_C].virt, 1, regionSize, (uint32_t) virt_to_phys((uint64_t*)(dataBuffer[DATA_C].virt + FW_WORD_SIZE * 2)));

		//We fill the regions with consecutive numbers.
		for(i = 1; i <= regionSize; i++)
			*(direccionVirtual + i - 1) = i;
		
		for(i = regionSize + 1; i <= regionSize * 2; i++)
			*(dataBuffer[DATA_C].virt + i - 1 - regionSize) = i;

		up(&semaforoIO_DMA[DATA_C]);
	}
	printk(KERN_INFO "FPGA Simulator: Written DATA_C \n");
	return 0;
}

//Simulation close to real case. 16 writes will be performed simulating 0.5s of writing and 0.25s of waiting.
long write_DATA_D_ioctl(void){

	int i, j, l, n = 0;
	unsigned int rand = 0;
	uint32_t regionSize;
	char* direccionVirtual;
	uint32_t direccionFisica;

	uint32_t writingTime = usecs_to_jiffies(500000); //0.5 seconds
	uint32_t idleTime = usecs_to_jiffies(250000); //0.25 seconds

	//Region size, we occupy the whole buffer
	regionSize = BYTES_4K / 4;
	
	//We initialize the descriptors
	for(i = 0; i < NUM_DESCRIPTORS; i++)
	{
		direccionVirtual = dataBuffer[DATA_D].virt + (regionSize * i);
		direccionFisica = (uint32_t)(virt_to_phys((uint64_t*) direccionVirtual));

		setDescriptorInfo(descriptorBuffer[DATA_D].virt + FW_WORD_SIZE * i, 1, regionSize, direccionFisica);
	}

	
	for(i = 0; i < 4; i++){

		//We perform a writing
		for(j = 0; j < NUM_DESCRIPTORS; j++){

			//We obtain the semaphore from the buffer
			if(down_timeout(&semaforoIO_DMA[DATA_D], tmo) == 0){

				direccionVirtual = dataBuffer[DATA_D].virt + (regionSize * j);
				direccionFisica = (uint32_t)(virt_to_phys((uint64_t*) direccionVirtual));
				
				//We fill this region with the iteration number. This time with random numbers, minus the first word, 
				//so we can be sure that what we are reading is ordered data.
				memset(direccionVirtual, ++n, FW_WORD_SIZE);
				
				for (l = FW_WORD_SIZE; l < regionSize; l++){
					get_random_bytes(&rand, sizeof(rand));
					memset(direccionVirtual + l, (rand % 128), 1);
				}
				
				//We set the control buffer to the following descriptor where we are about to write
				if(j == NUM_DESCRIPTORS - 1)
					setDescriptorInfo(controlBuffer[DATA_D].virt, 1, regionSize, (uint32_t) virt_to_phys((uint64_t*)(dataBuffer[DATA_D].virt)));
				else
					setDescriptorInfo(controlBuffer[DATA_D].virt, 1, regionSize, (uint32_t) virt_to_phys((uint64_t*)(dataBuffer[DATA_D].virt + regionSize * (j + 1))));
				printk(KERN_INFO " %016llx \n", *((uint64_t*)controlBuffer[DATA_D].virt));
				msleep(writingTime);

				up(&semaforoIO_DMA[DATA_D]);
			}

			//We wait before writing again
			msleep(idleTime);
		}
		
	}

	printk(KERN_INFO "FPGA Simulator: Written DATA_D \n");
	return 0;
}

/////////////////////////
//// LOCAL FUNCTIONS ////
/////////////////////////

long dmaAddrConfig(unsigned long ioctl_param){
	
	int32_t bufferIndex;
	DmaAddrConfig * dmaAddrConf_pointer = (DmaAddrConfig*) ioctl_param;
		
	printk(KERN_INFO "FPGA Simulator: Received physical addresses \n");

	if (dmaAddrConf_pointer == NULL)
		return -EFAULT;

	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		dataBuffer[bufferIndex].bus = dmaAddrConf_pointer->baseAddrData[bufferIndex];
		dataBuffer[bufferIndex].virt = (char*) phys_to_virt(dataBuffer[bufferIndex].bus);
		printk(KERN_INFO "RECEIVED: DATA BUFFER[%i]: %llx\n", bufferIndex, (long long unsigned int) dataBuffer[bufferIndex].bus);

		descriptorBuffer[bufferIndex].bus = dmaAddrConf_pointer->baseAddrDesc[bufferIndex];
		descriptorBuffer[bufferIndex].virt = (char*) phys_to_virt(descriptorBuffer[bufferIndex].bus);
		printk(KERN_INFO "RECEIVED: DESCRIPTOR BUFFER[%i]: %llx\n", bufferIndex, (long long unsigned int) descriptorBuffer[bufferIndex].bus);

		controlBuffer[bufferIndex].bus = dmaAddrConf_pointer->baseAddrCtl[bufferIndex];
		controlBuffer[bufferIndex].virt = (char*) phys_to_virt(controlBuffer[bufferIndex].bus);
		printk(KERN_INFO "RECEIVED: CONTROL BUFFER[%i]: %llx\n", bufferIndex, (long long unsigned int) controlBuffer[bufferIndex].bus);
	}
	
	return 0;
}

void setDescriptorInfo(char* virtDescriptor_pointer, uint8_t err, uint32_t buffSize, uint32_t buffAddr){

	uint64_t descData;

	descData = 0x0000000000000000;
	descData += (uint64_t) err << 56;
	descData += (uint64_t) buffSize << 32;
	descData += (uint64_t) buffAddr;
	memcpy((uint64_t*) virtDescriptor_pointer, &descData, sizeof(uint64_t));
	
}
