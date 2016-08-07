/*
 * sgdma.h
 *
 *  Created on: Jul 4, 2016
 *      Author: Daniel Pelikan
 *      Copyright 2016. All rights reserved
 *
 *      https://www.altera.com/en_US/pdfs/literature/ug/ug_embedded_ip.pdf
 */

#ifndef SGDMA_H_
#define SGDMA_H_



#include <stdint.h>
#include <time.h>




//define the structure for the descriptor
struct alt_avalon_sgdma_packed{
	uint32_t *read_addr;
	uint32_t read_addr_pad;
	uint32_t *write_addr;
	uint32_t write_addr_pad;
	uint32_t *next; //Specifies the address of the next descriptor in the linked list. It seesm the next needs to pint to a empty descriptor with the owned-by-hw cleard
	uint32_t next_pad;
	uint16_t bytes_to_transfer;
	uint8_t read_burst; // Reserved field. Set to 0.
	uint8_t write_burst;// Reserved field. Set to 0.
	uint16_t actual_bytes_transferred;
	uint8_t status;
	uint8_t control;
};

//Base address pointer
void *h2p_lw_sgdma_addr=NULL;


//Register Map

#define _SGDMA_REG_STATUS(BASE_ADDR) *((uint32_t *)BASE_ADDR+0)
#define _SGDMA_REG_VERSION(BASE_ADDR) *((uint32_t *)BASE_ADDR+1)
#define _SGDMA_REG_CONTROL(BASE_ADDR) *((uint32_t *)BASE_ADDR+4)
#define _SGDMA_REG_NEXT_DESC_POINTER(BASE_ADDR) *((uint32_t *)BASE_ADDR+8)


//control register
#define _SGDMA_CTR_IE_ERROR 					0x1
#define _SGDMA_CTR_IE_EOP_ENCOUNTERED			0x2
#define _SGDMA_CTR_IE_DESCRIPTOR_COMPLETED		0x4
#define _SGDMA_CTR_IE_CHAIN_COMPLETED			0x8
#define _SGDMA_CTR_IE_GLOBAL					0x10
#define _SGDMA_CTR_RUN							0x20
#define _SGDMA_CTR_STOP_DMA_ER					0x40
#define _SGDMA_CTR_IE_MAX_DESC_PROCESSED		0x80
#define _SGDMA_CTR_MAX_DESC_PROCESSED			0x100

#define _SGDMA_CTR_SW_RESET						0x10000
#define _SGDMA_CTR_PARK							0x20000
#define _SGDMA_CTR_DESC_POLL_EN					0x40000

#define _SGDMA_CTR_CLEAR_INTERRUPT				0x80000000


//status register map

#define _SGDMA_STAT_ERROR						0x1
#define _SGDMA_STAT_EOP_ENCOUNTERED				0x2
#define _SGDMA_STAT_DESCRIPTOR_COMPLETED		0x4
#define _SGDMA_STAT_CHAIN_COMPLETED				0x8
#define _SGDMA_STAT_BUSY						0x10


//Descriptor control bit map
#define _SGDMA_DESC_CTRMAP_GENERATE_EOP 		0x1
#define _SGDMA_DESC_CTRMAP_READ_FIXED_ADDRESS	0x2
#define _SGDMA_DESC_CTRMAP_WRITE_FIXED_ADDRESS	0x4

#define _SGDMA_DESC_CTRMAP_OWNED_BY_HW			0x80





void debugPrintRegister(){
#ifdef DEBUG
	printf( "status: %x\n", _SGDMA_REG_STATUS(h2p_lw_sgdma_addr) );
	printf( "control: %x\n", _SGDMA_REG_CONTROL(h2p_lw_sgdma_addr) );
	printf( "next_descriptor_pointer: %x\n", _SGDMA_REG_NEXT_DESC_POINTER(h2p_lw_sgdma_addr) );
#endif
}

void debugPrintDescriptorStatus(struct alt_avalon_sgdma_packed *sgdma){
#ifdef DEBUG
	printf( "desc bytes transfered: %d\n", sgdma->actual_bytes_transferred);
	printf( "desc status: %x\n", sgdma->status);
	printf( "desc control: %x\n", sgdma->control);
	printf( "desc read addr: %x\n", (uint32_t)sgdma->read_addr);
#endif
}

void init_sgdma(uint32_t initCTR){
	_SGDMA_REG_STATUS(h2p_lw_sgdma_addr)=(uint32_t)0;//reset the status register
	_SGDMA_REG_CONTROL(h2p_lw_sgdma_addr)=0;
	_SGDMA_REG_CONTROL(h2p_lw_sgdma_addr)=(initCTR);
}

void setDescriptor(void *desc){
	  //set the adress of the descriptor
	_SGDMA_REG_NEXT_DESC_POINTER(h2p_lw_sgdma_addr)=((uint32_t)desc);

	//*((uint32_t *)h2p_lw_sgdma_addr+8)=((uint32_t)0x36000000);
}

void setControlReg(int32_t ctr){
	_SGDMA_REG_CONTROL(h2p_lw_sgdma_addr)=ctr;
}

void waitFinish(){
	if((_SGDMA_REG_STATUS(h2p_lw_sgdma_addr)&_SGDMA_STAT_BUSY))
		printf("wait...");
	while( (_SGDMA_REG_STATUS(h2p_lw_sgdma_addr)&_SGDMA_STAT_BUSY)){
		//usleep(1000);//usleep is obsolete
		struct timespec s;
		s.tv_sec = 0;
		s.tv_nsec = 1000000L;
		nanosleep(&s, NULL);
		printf(".");
	}
	printf("\n");
}

void initDescriptor(struct alt_avalon_sgdma_packed *desc,
		void *read_addr, void* write_addr, void *next_addr, uint16_t length, uint8_t ctr){
	desc->read_addr =  (uint32_t*)read_addr;
	desc->read_addr_pad = 0;
	desc->write_addr = (uint32_t*) (write_addr);
	desc->write_addr_pad =0;
	desc->next=next_addr;
	desc->next_pad = 0;
	desc->bytes_to_transfer=length;
	desc->read_burst = 0;
	desc->write_burst = 0;
	desc->actual_bytes_transferred = 0;
	desc->status = 0;
	desc->control = ctr;
}


#endif /* SGDMA_H_ */
