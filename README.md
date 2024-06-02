# About

The objective of this project is to develop a kernel module, or driver, for the Linux operating system (preferably *CentOS 7*) that will enable communication between the firmware of an FPGA, and any application in user space.

To facilitate the driver testing in any enviroment, an **FPGA simulator** and **ioctl simulator** are also presented. The *FPGA simulator* counts with 4 test cases, each one following with more complexity. On the other hand, the *ioctl simulator* allows sending of ioctl commands to both DMA and FPGA sim kernel modules.

</br>

## DMA Driver
This driver will be able to access and read the information provided by the FPGA in an orderly fashion. The FPGA, ideally, will write a constant data stream. Therefore, the developed driver must be able to read this information in a loop as long as necessary.  In turn, the driver must be able to provide the information read to the software components in user space that request it. 

The driver currently counts with 4 independent datasets of configurable size (untested):

</br>

- DATASET A
  - Control buffer:  *FW_WORD_SIZE*
  - Descriptor buffer:  *NUM_DESCRIPTORS* * *FW_WORD_SIZE*
  - Data buffer:  *BYTES_4K*
- DATASET B
  - ...
- DATASET C
  - ...
- DATASET D
  - ...

The data from each of these datasets can be accessed by any application in user space through the ioctl call DMA_READ_DATA_[x]. This call will return a *DmaReadPtrInfo* struct to the user space program, cointaining a pointer to data and its size.

</br>


## FPGA Simulator
To facilitate the driver testing in any enviroment, a **FPGA simulator** and a **ioctl simulator** is facilitated. The *FPGA simulator* counts with 4 test cases, each one following with more complexity. On the other hand, the *ioctl simulator* allows sending of ioctl commands to both DMA and FPGA sim kernel modules.

### Test case A
Writes characters from 1 to 3 on DMA driver's data buffer A
### Test case B
Writes characters from 1 to 3 on DMA driver's data buffer B, blocking synchronization semaphore for 3 seconds
### Test case C
Writes characters from 1 to 16, cutting off at the end of the buffer in data buffer C
### Test case D
Writes for several seconds a stream of 16 blocks of data, occupying the entire data buffer D while simulating *writing* and *idle* times

</br>

## Ioctl Simulator
C++ written userpsace application that allows sending of ioctl commands to both DMA and FPGA sim kernel modules. 

Compile the binary and execute it while passing any of the following characters as the first command-line argument:
```
./ioctl_sim [character]
```
</br>

| Character  | Sent ioctl command | Destiny | Description |
| ---------- | ------------------ | ------- | ----------- |
|i|DMA_INIT|DMA Driver|Initializes all DMA driver buffers to 0 |
|p|DMA_PRINT_BUFFER|DMA Driver|Print by kernel traces the buffer contents seen by the DMA driver|
|r|DMA_ADDR_CONF|DMA Driver|Prints by kernel traces the physical start/end addresses of each DMA driver buffers.|
|a|DMA_READ_DATA_A|DMA Driver|Prints by kernel traces the size and contents of the data buffer A|
|b|DMA_READ_DATA_B|DMA Driver|Prints by kernel traces the size and contents of the data buffer B|
|c|DMA_READ_DATA_C|DMA Driver|Prints by kernel traces the size and contents of the data buffer C|
|d|DMA_READ_DATA_D|DMA Driver|Prints by kernel traces the size and contents of the data buffer D|
|0|FPGA_PRINT_BUFFER|FPGA Simulator|Print by kernel traces the buffer contents seen by the FPGA Simulator|
|1|FPGA_WRITE_DATA_A|FPGA Simulator|Writes characters from 1 to 3 on DMA driver's data buffer A|
|2|FPGA_WRITE_DATA_B|FPGA Simulator|Writes characters from 1 to 3 on DMA driver's data buffer B, blocking synchronization semaphore for 3 seconds|
|3|FPGA_WRITE_DATA_C|FPGA Simulator|Writes characters from 1 to 16, cutting off at the end of the buffer in data buffer C|
|4|FPGA_WRITE_DATA_D|FPGA Simulator|Writes for several seconds a stream of 16 blocks of data, occupying the entire data buffer D while simulating *writing* and *idle* times|
