/*
 *      Author: Daniel Pelikan
 *      Copyright 2016. All rights reserved
*/

    //some documentaton

	/* By default the full size of the DDR-RAM is assigned to the linux kernel.
	 * With an option in the u-boot command line a limit can be set how much memeory should be assigned to the kernel
	 * This can be done with the option mem=xxxx
	 *
	 * To be able to enter the u-boot console connect the serial cable to the serial port before booting
	 * 115200 is the baudrate
	 *
	 *
	 * https://www.altera.com/support/support-resources/knowledge-base/solutions/rd06132014_165.html
	 * setenv bootargs console=ttyS0,115200 mem=1000M
	 * saveenv
	 *
	 * It seesm that mmcboot is booted by default
	 *
	 * The default line in u-boot looks like
	 * mmcboot=setenv bootargs console=ttyS0,115200 root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 * which needs to be changed to:
	 * mmcboot=setenv bootargs console=ttyS0,115200 mem=800M root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 *by:
	 * setenv mmcboot 'setenv bootargs console=ttyS0,115200 mem=800M root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}'
	 * saveenv
	 * ----setenv bootargs console=ttyS0,115200 root=${mmcroot} rw rootwait;bootz ${loadaddr} - ${fdtaddr}
	 *
	 * More infos
	 *
	 * https://forum.rocketboards.org/t/how-to-reserve-ddr3-memory-region-for-fpga-direct-access/162/6
	 * https://rocketboards.org/foswiki/view/Documentation/GSRD131ProgrammingFPGA#GSRD_FPGA_Configuration
	 *
	 * cat /proc/meminfo
	 *
	 * cat /proc/iomem
	 *
	 * 00000000-31ffffff : System RAM
	 * rest to 1024 is not occupied
	 *
	 * 32000000-3fffffff should be available
	 *
	 * 32000000-35ffffff = 64 MB
	 *
	 */

//--------------------------------------------------------------------------------------------------------


//#define soc_cv_av

#define DEBUG

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "soc_cv_av/socal/socal.h"
#include "soc_cv_av/socal/hps.h"
#include "soc_cv_av/socal/alt_gpio.h"
#include "hps_0.h"



#include "sgdma.h"

#include "dma.h"



//DMA
//#include "alt_dma.h"
//#include "alt_globaltmr.h"



/*
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/inc/altera_avalon_sgdma_regs.h
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/HAL/inc/altera_avalon_sgdma.h
/home/imp/altera/16.0/ip/altera/sopc_builder_ip/altera_avalon_sgdma/HAL/inc/altera_avalon_sgdma_descriptor.h

ip/altera/nios2_ip/altera_nios2/HAL/inc/
*/

//corret in the Makre file the path to:
// /home/imp/altera/16.0/embedded/ip/altera/hps/altera_hps/hwlib/include

//https://www.altera.com/hps/en_us/cyclone-v/hps.html#topic/sfo1418687413697.html
//http://www.alteraforum.com/forum/showthread.php?t=42136

//settings for the lightweight HPS-to-FPGA bridge
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 ) //64 MB with 32 bit adress space this is 256 MB
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )


//setting for the HPS2FPGA AXI Bridge
#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )


//SDRAM 32000000-35ffffff //64 MB
#define SDRAM_64_BASE 0x32000000
#define SDRAM_64_SPAN 0x3FFFFFF

//SDRAM 36000000-36ffffff //16 MB
#define SDRAM_16_BASE 0x36000000
#define SDRAM_16_SPAN 0xFFFFFF





int main() {

	//pointer to the different address spaces

	void *virtual_base;
	void *axi_virtual_base;
	int fd;


	void *h2p_lw_reg1_addr;
	void *h2p_lw_reg2_addr;
	void *h2p_lw_reg3_addr;
	//void *h2p_lw_myBus_addr;


	void *h2p_led_addr; //led via AXI master
	void *h2p_rom_addr; //scratch space via ax master 64kb
	void *h2p_rom2_addr;

	void *sdram_64MB_add;
	void *sdram_16MB_add;


	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	//lightweight HPS-to-FPGA bridge
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	//HPS-to-FPGA bridge
	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd,ALT_AXI_FPGASLVS_OFST );

	if( axi_virtual_base == MAP_FAILED ) {
		printf( "ERROR: axi mmap() failed...\n" );
		close( fd );
		return( 1 );
	}


//-----------------------------------------------------------
	//configure the LEDs of the Golden Reference design
	printf( "\n\n\n-----------Set the LEDs on-------------\n\n" );

	//LED connected to the HPS-to-FPGA bridge
	h2p_led_addr=axi_virtual_base + ( ( unsigned long  )( 0x0 + PIO_LED_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );

	*(uint32_t *)h2p_led_addr = 0b10111100;

//-----------------------------------------------------------
	//Adder test: Two registers are connected to a adder and place the result in the third register
	printf( "\n\n\n-----------Add two numbers in the FPGA-------------\n\n" );

	//the address of the two input (reg1 and reg2) registers and the output register (reg3)
	h2p_lw_reg1_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_reg2_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_reg3_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_REG3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );


	//write into register to test the adder
	*(uint32_t *)h2p_lw_reg1_addr = 10;
	*(uint32_t *)h2p_lw_reg2_addr = 5;

	//read result of the adder from register 3
	printf( "Adder result:%d + %d = %d\n", *((uint32_t *)h2p_lw_reg1_addr), *((uint32_t *)h2p_lw_reg2_addr), *((uint32_t *)h2p_lw_reg3_addr) );


//-------------------------------------------------------------
	//prepare the on chip memory devices
	printf( "\n\n\n-----------write on chip RAM-------------\n\n" );

	//ONCHIP_MEMORY2_0_BASE connected via the HPS-to-FPGA bridge
	h2p_rom_addr=axi_virtual_base + ( ( unsigned long  )( ONCHIP_MEMORY2_0_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );

	h2p_rom2_addr=axi_virtual_base + ( ( unsigned long  )( ONCHIP_MEMORY2_1_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );


	//write some data to the scatch disk
	for (int i=0;i<16000;i++){
		*((uint32_t *)h2p_rom_addr+i)=i*1024;
		*((uint32_t *)h2p_rom2_addr+i)=i+3;
	}

	printf( "Print scratch disks:\n" );
	printf( "ROM1 \t ROM2\n");
	for (int i=0;i<10;i++){
		printf( "%d\t%d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}

//-----------------------------------------------
	//DMA

	printf( "\n\n\n-----------DMA RAM to RAM-------------\n\n" );


	//print the content of scratchdisk 1 and 2
	printf( "Print scratch disk 1 and 2:\n" );
	for (int i=0;i<10;i++){
		printf( "%d\t%d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}


	//create a pointer to the DMA controller base
	h2p_lw_dma_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DMA_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	//configure the DMA controller for transfer
	_DMA_REG_STATUS(h2p_lw_dma_addr)=0;
	_DMA_REG_READ_ADDR(h2p_lw_dma_addr)=ONCHIP_MEMORY2_0_BASE; //read from ROM1
	_DMA_REG_WRITE_ADDR(h2p_lw_dma_addr)=ONCHIP_MEMORY2_1_BASE; //write to ROM2
	_DMA_REG_LENGTH(h2p_lw_dma_addr)=4*16000;//write 100x 4bytes since we have a 32 bit system

	//start the transfer
	_DMA_REG_CONTROL(h2p_lw_dma_addr)=_DMA_CTR_WORD | _DMA_CTR_GO | _DMA_CTR_LEEN;


	debugPrintDMARegister();

	debugPrintDMAStatus();

	//wait for DMA to be finished
	waitDMAFinish();
	stopDMA();//stop the DMA controller


	//check if data was copied
	printf( "Print scratch disk 1 and 2:\n" );
	for (int i=0;i<10;i++){
		printf( "%d\t %d\n", *((uint32_t *)h2p_rom_addr+i),*((uint32_t *)h2p_rom2_addr+i) );
	}

//------------------------------------------------------

	printf( "\n\n\n-----------DMA RAM to PIO-------------\n\n" );


	//generate some test data were every byte is a different number
	//the least significant byte is the clocked out first to the PIO
	for (int i=0;i<16000;i++){
		*((uint32_t *)h2p_rom_addr+i)=(uint32_t)((i*4+3)%256<<24 | (i*4+2)%256<<16 | (i*4+1)%256<<8 | (i*4+0)%256<<0);
	}

	//configure the DMA

	_DMA_REG_STATUS(h2p_lw_dma_addr)=0;
	_DMA_REG_READ_ADDR(h2p_lw_dma_addr)=ONCHIP_MEMORY2_0_BASE;
	_DMA_REG_WRITE_ADDR(h2p_lw_dma_addr)=DMA_0_WRITE_MASTER_MYBUS_BASE;
	_DMA_REG_LENGTH(h2p_lw_dma_addr)=4*100;//write 100x 4bytes

	//start the transfer
	_DMA_REG_CONTROL(h2p_lw_dma_addr)=_DMA_CTR_BYTE | _DMA_CTR_GO | _DMA_CTR_LEEN | _DMA_CTR_WCON;//0b1010001100;


	//some debug info
	debugPrintDMARegister();
	debugPrintDMAStatus();

	//wait untill the whole transfer is finished
	waitDMAFinish();
	stopDMA();//stop the DMA controller



//-----------------------------------------------
	//Scatter Gather DMA controller

	printf( "\n\n------------- Scatter DMA---------------\n\n" );

	//space for data
	sdram_64MB_add=mmap( NULL, SDRAM_64_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_64_BASE );

	//space for the descriptor
	sdram_16MB_add=mmap( NULL, SDRAM_16_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_16_BASE );


	//create descriptors in the mapped memory
	struct alt_avalon_sgdma_packed  *sgdma_desc1=sdram_16MB_add;
	struct alt_avalon_sgdma_packed  *sgdma_desc2=sdram_16MB_add+sizeof(struct alt_avalon_sgdma_packed);
	struct alt_avalon_sgdma_packed  *sgdma_desc3=sdram_16MB_add+2*sizeof(struct alt_avalon_sgdma_packed);
	struct alt_avalon_sgdma_packed  *sgdma_desc_empty=sdram_16MB_add+3*sizeof(struct alt_avalon_sgdma_packed);

	//Address to the physical space
	void* sgdma_desc1_phys=(void*)SDRAM_16_BASE;
	void* sgdma_desc2_phys=(void*)SDRAM_16_BASE+sizeof(struct alt_avalon_sgdma_packed);
	void* sgdma_desc3_phys=(void*)SDRAM_16_BASE+2*sizeof(struct alt_avalon_sgdma_packed);
	void* sgdma_desc_empty_phys=(void*)SDRAM_16_BASE+3*sizeof(struct alt_avalon_sgdma_packed);



	//configure the descriptor
	initDescriptor(sgdma_desc1,(void*)SDRAM_64_BASE,(void*)MEMORYDMA_M_WRITE_MYBUS_BASE,
			sgdma_desc2_phys,4*10,
			(_SGDMA_DESC_CTRMAP_WRITE_FIXED_ADDRESS | _SGDMA_DESC_CTRMAP_OWNED_BY_HW));

	initDescriptor(sgdma_desc2,(void*)SDRAM_64_BASE,(void*)MEMORYDMA_M_WRITE_MYBUS_BASE,
			sgdma_desc3_phys,4*20,
			(_SGDMA_DESC_CTRMAP_WRITE_FIXED_ADDRESS | _SGDMA_DESC_CTRMAP_OWNED_BY_HW));

	initDescriptor(sgdma_desc3,(void*)SDRAM_64_BASE,(void*)MEMORYDMA_M_WRITE_MYBUS_BASE,
			sgdma_desc_empty_phys,4*30,
			(_SGDMA_DESC_CTRMAP_WRITE_FIXED_ADDRESS | _SGDMA_DESC_CTRMAP_OWNED_BY_HW));

	initDescriptor(sgdma_desc_empty,NULL,NULL,
			sgdma_desc_empty_phys,0,
			(_SGDMA_DESC_CTRMAP_WRITE_FIXED_ADDRESS));



	//map memory of the control register
	h2p_lw_sgdma_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MEMORYDMA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );


	//fill the data space
	for (long int i=0;i<100000/*SDRAM_64_SPAN*/;i++){
		*((uint32_t *)sdram_64MB_add+i)=i;
	}



	//init the SGDMA controller
	init_sgdma(_SGDMA_CTR_IE_CHAIN_COMPLETED);

	debugPrintRegister();

	//set the address of the descriptor
	setDescriptor(sgdma_desc1_phys);

	//start the transfer
	setControlReg(_SGDMA_CTR_IE_CHAIN_COMPLETED|_SGDMA_CTR_RUN);

	debugPrintRegister();

	//wait until transfer is complete
	waitFinish();

	debugPrintDescriptorStatus(sgdma_desc1);

	//stop the core by clearing the run register
	setControlReg(_SGDMA_CTR_IE_CHAIN_COMPLETED);

	debugPrintDescriptorStatus(sgdma_desc1);


	//-------------------------------------------------------------
	// clean up our memory mapping and exit



	if( munmap( sdram_64MB_add, SDRAM_64_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	if( munmap( sdram_16MB_add, SDRAM_16_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}


	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	if( munmap( axi_virtual_base, HW_FPGA_AXI_SPAN ) != 0 ) {
		printf( "ERROR: axi munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
