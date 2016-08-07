#
TARGET = my_first_hps-fpga

ALT_DEVICE_FAMILY = soc_cv_av
SOCEDS_ROOT = /home/imp/altera/16.0/embedded
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib



SOCEDS_DEST_ROOT=/home/imp/altera/16.0/embedded/ip/altera/hps/altera_hps/hwlib
IP_DEST_ROOT=/home/imp/altera/16.0

HWLIBS_SRC  := ${SOCEDS_DEST_ROOT}/src/hwmgr/alt_dma.o ${SOCEDS_DEST_ROOT}/src/hwmgr/alt_dma_program.o ${SOCEDS_DEST_ROOT}/src/hwmgr/alt_globaltmr.o


#
CROSS_COMPILE = arm-linux-gnueabihf-
#CFLAGS = -static -g -Wall -std=c99  -I${SOCEDS_DEST_ROOT}/include -I$(HWLIBS_ROOT)/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -I${SOCEDS_ROOT}/examples/hardware/cv_soc_devkit_ghrd/software/preloader/uboot-socfpga/arch/arm/include/ -D$(ALT_DEVICE_FAMILY) 
CFLAGS = -static -g -Wall -std=c99 -D_POSIX_C_SOURCE=199309L -I${SOCEDS_DEST_ROOT}/include -I$(HWLIBS_ROOT)/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -I${SOCEDS_ROOT}/examples/hardware/cv_soc_devkit_ghrd/software/preloader/uboot-socfpga/arch/arm/include/ -D$(ALT_DEVICE_FAMILY) 

LDFLAGS =  -g -Wall  
CC = $(CROSS_COMPILE)gcc
ARCH= arm





all: $(TARGET)
$(TARGET): main.o $(HWLIBS_SRC)
	$(CC) $(LDFLAGS)   $^ -o $@  
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 