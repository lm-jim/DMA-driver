#include "dma.h"
#include "linux/module.h"
#include "linux/time.h"
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <asm/io.h>

/* Pointers to virtual and physical memory */
ParVirtBus  descriptorBuffer[4];
ParVirtBus  dataBuffer[4];
ParVirtBus  controlBuffer[4];

/* We create and export the semaphores so they are visible by the FPGA simulator. */
struct semaphore semaforoIO_DMA[4];
EXPORT_SYMBOL(semaforoIO_DMA);

/* Contains the physical address of the descriptor to be read */
static uint32_t descriptor_a_leer[4];

//////////////////////////////////////////////////////
//// DRIVER CONTROL BLOCK AND FUNCTION DEFINITION ////
//////////////////////////////////////////////////////

MODULE_AUTHOR("Luis Miguel Jimenez Aliaga");
MODULE_DESCRIPTION("DMA driver");
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
static int dma_init_module(void);
static void dma_exit_module(void);
int32_t dma_sw_init(void);

/* file_operations functions */
int32_t dma_dev_open(struct inode *, struct file *);
int32_t dma_dev_rls(struct inode*, struct file *);
ssize_t dma_dev_read(struct file *, char *, size_t, loff_t *);
ssize_t dma_dev_write(struct file * file, const char *, size_t, loff_t *);
long dma_dev_ioctl(struct file *file, uint32_t ioctl_num, unsigned long ioctl_param);

/* ioctl functions */
long initDma_ioctl(void);
long printBuffer_ioctl(void);
long dmaAddrConfig_ioctl(unsigned long ioctl_param);
long dmaReadPtr_ioctl(int32_t data, unsigned long ioctl_param);

static struct file_operations fops =
{
  .read = dma_dev_read,
  .open = dma_dev_open,
  .write = dma_dev_write,
  .release = dma_dev_rls,
  .unlocked_ioctl = dma_dev_ioctl
};

//////////////////////////////////////////
//// DRIVER FUNCTIONS | KERNEL MODULE ////
//////////////////////////////////////////

/**
 * dma_sw_init - Initialize general software structures
 *
 * Function that will initialize the necessary structs to register the driver in the system.
 **/
int32_t dma_sw_init(void)
{
	int resultado_Op;
	int i = 0;

	printk(KERN_INFO "Installing DMA driver... ");
	
	//Registering Major number and Minor number
	driverCB.dev = MKDEV(DMA_MAJOR, 0);
	resultado_Op = register_chrdev_region(driverCB.dev, 1, DMA_NAME);

	if (resultado_Op != 0)
	{
		printk(KERN_ALERT "DMA: Failed to allocate major number\n");
		return resultado_Op;
	}
	else
	{
		//Inserting Major number and Minor number in the driver control block
		driverCB.major_num = MAJOR(driverCB.dev);
		driverCB.minor_num = MINOR(driverCB.dev);
		driverCB.max_minor = DMA_MAX_MINOR;

		//Reserving memory for the struct cdev
		driverCB.cdev = cdev_alloc();

		//Link cdev with file_operations and establish this module as the owner
		driverCB.cdev->ops = &fops;
		driverCB.cdev->owner = THIS_MODULE;

		for(i = 0; i < 4; i++)
			sema_init(&semaforoIO_DMA[i],1);

		//We proceed to register the driver in the Kernel
		if(cdev_add(driverCB.cdev, driverCB.dev, 1) < 0)
		{
			printk(KERN_ERR "dma_drv_Install: Driver registration error\n");
			resultado_Op = -EFAULT;
		}
	}

	printk((resultado_Op == 0) ? KERN_INFO "DMA driver module structs OK\n": KERN_ERR "DMA driver module structs ERROR\n");

	if(resultado_Op != 0)
	{
		unregister_chrdev_region(driverCB.dev, driverCB.max_minor);
		cdev_del(driverCB.cdev);
		return resultado_Op;
	}

	return resultado_Op;
}

/**
 * dma_init_module - Driver Registration Routine
 *
 * dma_init_module is the first routine executed when the module is initialized.
 **/

static int __init
dma_init_module(void)
{
	int32_t resultado_Op = 0;
	int32_t bufferIndex;
	
	printk(KERN_INFO "DMA driver initializing...\n");
	
	//Register the driver in the system
	resultado_Op = dma_sw_init();

	///// MEMORY ALLOCATION FOR DATASETS A, B, C and D /////
 	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		//We allocate kernel memory (virtual)
		dataBuffer[bufferIndex].virt = kmalloc(BYTES_4K, GFP_KERNEL | GFP_DMA); 
		descriptorBuffer[bufferIndex].virt = kmalloc(NUM_DESCRIPTORS * FW_WORD_SIZE, GFP_DMA | GFP_KERNEL);
		controlBuffer[bufferIndex].virt = kmalloc(FW_WORD_SIZE, GFP_DMA | GFP_KERNEL);

		printk(KERN_INFO "dataBuffer[%i].virt: %llx\n", bufferIndex, (long long unsigned int)dataBuffer[bufferIndex].virt);
		printk(KERN_INFO "descriptorBuffer[%i].virt: %llx\n", bufferIndex, (long long unsigned int)descriptorBuffer[bufferIndex].virt);
		printk(KERN_INFO "controlBuffer[%i].virt: %llx\n", bufferIndex, (long long unsigned int)controlBuffer[bufferIndex].virt);

		//Initialize reserved memory to 0
		memset(dataBuffer[bufferIndex].virt, 0, BYTES_4K);
		memset(descriptorBuffer[bufferIndex].virt, 0, NUM_DESCRIPTORS * FW_WORD_SIZE);
		memset(controlBuffer[bufferIndex].virt, 0, FW_WORD_SIZE);

		//We save its conversion to physical memory (bus)
		dataBuffer[bufferIndex].bus = virt_to_phys(dataBuffer[bufferIndex].virt);
		descriptorBuffer[bufferIndex].bus = virt_to_phys(descriptorBuffer[bufferIndex].virt);
		controlBuffer[bufferIndex].bus = virt_to_phys(controlBuffer[bufferIndex].virt);
		
		printk(KERN_INFO "dataBuffer[%i].bus: %llx\n", bufferIndex, (long long unsigned int)dataBuffer[bufferIndex].bus);
		printk(KERN_INFO "descriptorBuffer[%i].bus: %llx\n", bufferIndex, (long long unsigned int)descriptorBuffer[bufferIndex].bus);
		printk(KERN_INFO "controlBuffer[%i].bus: %llx\n", bufferIndex, (long long unsigned int)controlBuffer[bufferIndex].bus);
		
		//Initialize to start reading from the first descriptor
		descriptor_a_leer[bufferIndex] = virt_to_phys(descriptorBuffer[bufferIndex].virt);
	}
	
	printk((resultado_Op == 0) ? "DMA driver initialized successfully\n": "Error initializing DMA driver module\n");
	return resultado_Op;
}

module_init(dma_init_module);

/**
 * dma_exit_module - Driver Exit Cleanup Routine
 *
 * dma_exit_module is called just before removing the driver from memory.
 **/
static void __exit
dma_exit_module(void)
{
	int32_t bufferIndex;
	printk(KERN_INFO "DMA: Removing module...\n");

	//We release any memory that has been reserved
	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		if(dataBuffer[bufferIndex].virt != 0)
			kfree(dataBuffer[bufferIndex].virt);
		if(descriptorBuffer[bufferIndex].virt != 0)
			kfree(descriptorBuffer[bufferIndex].virt);
		if(controlBuffer[bufferIndex].virt != 0)
			kfree(controlBuffer[bufferIndex].virt);
	}

	//Freeing the Major and Minor number
	unregister_chrdev_region(driverCB.dev, driverCB.max_minor);

	//Freeing device
	cdev_del(driverCB.cdev);

	printk(KERN_INFO "DMA module removed\n");
}

module_exit(dma_exit_module);

/**
 * dma_dev_open - Called when the device is opened.
 *
 * Returns 0 on success, negative value on failure
 **/
int32_t dma_dev_open(struct inode *inod, struct file *fil)
{
	return 0;
}

/**
 * dma_dev_read - Called when the device is read.
 *
 * Returns 0 on success, negative value on failure
 **/

ssize_t dma_dev_read(struct file *file, char *buff, size_t len, loff_t * offset)
{
	return 0;
}

/**
 * dma_dev_write - Called when the device is written.
 *
 * Returns 0 on success, negative value on failure
 **/

ssize_t dma_dev_write(struct file *file, const char *buff, size_t len, loff_t * offset)
{
	return 0;
}

/**
 * dma_dev_rls - Called when the device is released.
 *
 * Returns 0 on success, negative value on failure
 **/
int32_t dma_dev_rls(struct inode *inod, struct file *fil)
{
	return 0;
}

/**
 * dma_dev_ioctl - Ioctl's of the device.
 *
 * Function called when receiving an ioctl command
 **/
long dma_dev_ioctl(struct file *file, uint32_t ioctl_num, unsigned long ioctl_param)
{
	long result = 0;

	switch (ioctl_num)
	{
		case DMA_INIT:
			result = initDma_ioctl();
			break;
		case DMA_PRINT_BUFFER:
			result = printBuffer_ioctl();
			break;
		case DMA_ADDR_CONF:
			result = dmaAddrConfig_ioctl(ioctl_param);
			break;
		case DMA_READ_DATA_A:
			dmaReadPtr_ioctl(DATA_A, ioctl_param);
			break;
		case DMA_READ_DATA_B:
			dmaReadPtr_ioctl(DATA_B, ioctl_param);
			break;
		case DMA_READ_DATA_C:
			dmaReadPtr_ioctl(DATA_C, ioctl_param);
			break;
		case DMA_READ_DATA_D:
			dmaReadPtr_ioctl(DATA_D, ioctl_param);
			break;
		default:
			printk(KERN_ALERT "DMA: Unknown ioctl call received.");
			result = -1;
			break;
	}

	return result;
}

/////////////////////////
//// IOCTL FUNCTIONS ////
/////////////////////////

long initDma_ioctl(void){

	int32_t bufferIndex;

	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		memset(dataBuffer[bufferIndex].virt, 0, BYTES_4K);
		memset(descriptorBuffer[bufferIndex].virt, 0, NUM_DESCRIPTORS * FW_WORD_SIZE);
		memset(controlBuffer[bufferIndex].virt, 0, FW_WORD_SIZE);
	}
	printk(KERN_INFO "DMA Initialized to 0\n");

	return 0;
}

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

long dmaAddrConfig_ioctl(unsigned long ioctl_param){

	int32_t bufferIndex;

	DmaAddrConfig dmaAddrConf;
	DmaAddrConfig * dmaAddrConf_pointer = (DmaAddrConfig*) ioctl_param;
	printk(KERN_INFO "DMA: Request for physical addresses received \n");

	if (dmaAddrConf_pointer == NULL)
		return -EFAULT;

	for(bufferIndex = DATA_A; bufferIndex <= DATA_D; bufferIndex++)
	{
		dmaAddrConf.baseAddrData[bufferIndex] = (uint32_t) dataBuffer[bufferIndex].bus;
		dmaAddrConf.highAddrData[bufferIndex] = dataBuffer[bufferIndex].bus + BYTES_4K - 1;
		printk(KERN_INFO "REQUEST FOR PHYSICAL ADDRESSES: DATA BUFFER[%i]: %llx to %llx\n", 
				bufferIndex, (long long unsigned int)dmaAddrConf.baseAddrData[bufferIndex], (long long unsigned int)dmaAddrConf.highAddrData[bufferIndex]);

		dmaAddrConf.baseAddrDesc[bufferIndex] = (uint32_t) descriptorBuffer[bufferIndex].bus;
		dmaAddrConf.highAddrDesc[bufferIndex] = descriptorBuffer[bufferIndex].bus + (NUM_DESCRIPTORS * FW_WORD_SIZE) - 1;
		printk(KERN_INFO "REQUEST FOR PHYSICAL ADDRESSES: DESCRIPTOR BUFFER[%i]: %llx to %llx\n", 
				bufferIndex, (long long unsigned int)dmaAddrConf.baseAddrDesc[bufferIndex], (long long unsigned int)dmaAddrConf.highAddrDesc[bufferIndex]);

		dmaAddrConf.baseAddrCtl[bufferIndex] = (uint32_t) controlBuffer[bufferIndex].bus;
		printk(KERN_INFO "REQUEST FOR PHYSICAL ADDRESSES: CONTROL BUFFER[%i]: %llx\n\n", 
				bufferIndex, (long long unsigned int)dmaAddrConf.baseAddrCtl[bufferIndex]);
	}

	if(copy_to_user(dmaAddrConf_pointer, &dmaAddrConf, sizeof(DmaAddrConfig)))
		return -EFAULT;
	
	printk(KERN_INFO "DMA: Set physical addresses \n");

	return 0;
}

long dmaReadPtr_ioctl(int32_t data, unsigned long ioctl_param){

	long tmo = usecs_to_jiffies(5000000); //5 seconds (it is excessive, but we can test the synchronization with the simulator)

	long int size;
	uint64_t sizeaux;
	uint64_t dataPtr;
	uint64_t error;
	
	char *bufdata;
	
	DmaReadPtrInfo * pDmaPtrInfoSt;
	DmaReadPtrInfo dmaPtrInfoSt;
	
	//We wait for the corresponding reading semaphore
	if(down_timeout(&semaforoIO_DMA[data], tmo) == 0)
	{
		pDmaPtrInfoSt = (DmaReadPtrInfo*) ioctl_param;

		if (pDmaPtrInfoSt == NULL || copy_from_user(&dmaPtrInfoSt, pDmaPtrInfoSt, sizeof(DmaReadPtrInfo)))
			return -EFAULT;

		size = 0;

		if((*(uint32_t*)controlBuffer[data].virt) != 0)
		{
			//We check if something new has been written. If what we have to read is equal to the control buffer, 
			//it means we are going to read the buffer going to be written next, so we wait.
			if(*((uint64_t*)controlBuffer[data].virt) != *((uint64_t*)phys_to_virt(descriptor_a_leer[data])))
			{
				//The descriptor gives us the memory address to which we are going to read, and the size
				//We extract the information from it with the bitmasks

				dataPtr = *((uint64_t*)phys_to_virt(descriptor_a_leer[data])) & DATA_POINTER_MASK;
				size = ((*((uint64_t*)phys_to_virt(descriptor_a_leer[data])) & SIZE_MASK) >> 32); // Data size
				error = (*((uint64_t*)phys_to_virt(descriptor_a_leer[data])) & ERROR_MASK)  >> 56; // 1 OK, 0 DON'T READ

				printk(KERN_INFO "DMA: DATA DESCRIPTOR WAS READ %i: error=%02x size=%06x dataPtr=%08x \n", data, (uint16_t) error, (uint32_t) size, (uint32_t)  dataPtr);
				
				if(error > 0)
				{
					//We read the information described by this descriptor
					bufdata = phys_to_virt((uint32_t) dataPtr);
				
					//We calculate if we go out of the buffer, and read by turning it around if that's the case
					if(((uint64_t)(dataPtr + size)) > ((uint64_t)(dataBuffer[data].bus + BYTES_4K - 1)))
					{
						printk(KERN_INFO "DMA: READING EXCEEDED THE BUFFER LIMIT%i\n", data);
						sizeaux = (dataBuffer[data].bus + BYTES_4K - dataPtr);
						
						if(copy_to_user(dmaPtrInfoSt.buffer, bufdata, sizeaux))
							return -EFAULT;
						
						bufdata = phys_to_virt(dataBuffer[data].bus);
						
						if(copy_to_user(dmaPtrInfoSt.buffer + sizeaux, bufdata, (size - sizeaux)))
							return -EFAULT;

					}
					else
					{
						if(copy_to_user(dmaPtrInfoSt.buffer, bufdata, size))
							return -EFAULT;
					}

				}
				else
				{
					printk(KERN_INFO "DMA: EMPTY DESCRIPTOR ERROR %i\n", data);
					size = 0;
				}
				
				//Copy the read data to the user space memory
				if(copy_to_user(&(pDmaPtrInfoSt->bytes), &size, sizeof(long int)))
					return -EFAULT;
				
				//Once we finish reading, we point to the following descriptor
				descriptor_a_leer[data] = descriptor_a_leer[data] + FW_WORD_SIZE;

				//Return pointer to the first descriptor if the last descriptor has already been read
				if(descriptor_a_leer[data] == virt_to_phys(descriptorBuffer[data].virt) + (NUM_DESCRIPTORS * FW_WORD_SIZE))
					descriptor_a_leer[data] = virt_to_phys(descriptorBuffer[data].virt);
				

			}
		}
		//We release the corresponding semaphore
		up(&semaforoIO_DMA[data]);
	}
	
	return 0;
}

