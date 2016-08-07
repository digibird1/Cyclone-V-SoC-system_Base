/*
 * dma.h
 *
 *  Created on: Jul 10, 2016
 *      Author: Daniel Pelikan
 *      Copyright 2016. All rights reserved
 */

#ifndef DMA_H_
#define DMA_H_

#include <stdint.h>

#include <time.h>

//Base address pointer
void *h2p_lw_dma_addr=NULL;

//Register Map

#define _DMA_REG_STATUS(BASE_ADDR) *((uint32_t *)BASE_ADDR+0)
#define _DMA_REG_READ_ADDR(BASE_ADDR) *((uint32_t *)BASE_ADDR+1)
#define _DMA_REG_WRITE_ADDR(BASE_ADDR) *((uint32_t *)BASE_ADDR+2)
#define _DMA_REG_LENGTH(BASE_ADDR) *((uint32_t *)BASE_ADDR+3)
#define _DMA_REG_CONTROL(BASE_ADDR) *((uint32_t *)BASE_ADDR+6)

//status register map

#define _DMA_STAT_DONE				0x1
#define _DMA_STAT_BUSY				0x2
#define _DMA_STAT_REOP				0x4
#define _DMA_STAT_WEOP				0x8
#define _DMA_STAT_LEN				0x10

//control register
#define _DMA_CTR_BYTE				0x1
#define _DMA_CTR_HW					0x2
#define _DMA_CTR_WORD				0x4
#define _DMA_CTR_GO					0x8
#define _DMA_CTR_I_EN				0x10
#define _DMA_CTR_REEN				0x20
#define _DMA_CTR_WEEN				0x40
#define _DMA_CTR_LEEN				0x80
#define _DMA_CTR_RCON				0x100
#define _DMA_CTR_WCON				0x200
#define _DMA_CTR_DOUBLEWORD			0x400
#define _DMA_CTR_QUADWORD			0x800
#define _DMA_CTR_SOFTWARERESET		0x1000







void debugPrintDMARegister(){
#ifdef DEBUG
	printf("DMA Registers:\n");
	printf( "status: %x\n", _DMA_REG_STATUS(h2p_lw_dma_addr) );
	printf( "read: %x\n", _DMA_REG_READ_ADDR(h2p_lw_dma_addr) );
	printf( "write: %x\n", _DMA_REG_WRITE_ADDR(h2p_lw_dma_addr) );
	printf( "length: %x\n", _DMA_REG_LENGTH(h2p_lw_dma_addr) );
	printf( "control: %x\n", _DMA_REG_CONTROL(h2p_lw_dma_addr) );

#endif
}

void debugPrintDMAStatus(){
#ifdef DEBUG
	printf("DMA Status Registers:\n");
	if(*((uint32_t *)h2p_lw_dma_addr) & _DMA_STAT_DONE) printf( "Status: DONE\n");
	if(*((uint32_t *)h2p_lw_dma_addr) & _DMA_STAT_BUSY) printf( "Status: BUSY\n");
	if(*((uint32_t *)h2p_lw_dma_addr) & _DMA_STAT_REOP) printf( "Status: REOP\n");
	if(*((uint32_t *)h2p_lw_dma_addr) & _DMA_STAT_WEOP) printf( "Status: WEOP\n");
	if(*((uint32_t *)h2p_lw_dma_addr) & _DMA_STAT_LEN) printf( "Status: LEN\n");

#endif
}

void waitDMAFinish(){
	if((_DMA_REG_STATUS(h2p_lw_dma_addr)&_DMA_STAT_BUSY))
		printf("wait...");
	while( (_DMA_REG_STATUS(h2p_lw_dma_addr)&_DMA_STAT_BUSY)){
		struct timespec s;
		s.tv_sec = 0;
		s.tv_nsec = 1000000L;
		nanosleep(&s, NULL);
		//usleep(1000);//usleep is obsolete
		printf(".");
	}
	printf("\n");
}

void stopDMA(){
	*((uint32_t *)h2p_lw_dma_addr+6)=0;
}



#endif /* DMA_H_ */
