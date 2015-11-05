include $(RTEMS_MAKEFILE_PATH)/Makefile.inc
include $(RTEMS_CUSTOM)
include $(PROJECT_ROOT)/make/leaf.cfg

#### CONFIG ####################################################################
#For debugging symbols add -DLWIP_DEBUG
# COMPILER/LINKER
CFLAGS+=-g -O0 -Wall

# OUTPUT
LWIP_EXEC=stm32-lwip

#### PATHS #####################################################################

BSP_PATH=/opt/rtems-4.11/arm-rtems4.11/stm32f7x/lib/include/bsp

# STM32F LWIP
STM32F_LWIP_PATH=.
STM32F_LWIP_SRC_PATH=$(STM32F_LWIP_PATH)/src
STM32F_LWIP_INCL_PATH=$(STM32F_LWIP_PATH)/include

#### SOURCES ###################################################################

## STM32F SRC
STM32F_LWIP_SRC=$(wildcard $(STM32F_LWIP_SRC_PATH)/*.c )

SOURCES =  $(STM32F_LWIP_SRC) 

#### HEADERS ###################################################################

## CORE
STM32F_LWIP_H=$(STM32F_LWIP_INCL_PATH)

# HEADERS
HEADERS=-I$(STM32F_LWIP_H) -I$(BSP_PATH)

################################################################################

BIN=${ARCH}/$(LWIP_EXEC).bin
LIB=${ARCH}/lib$(LWIP_EXEC).a

# optional managers required
MANAGERS=all

# C source names
CSRCS=$(filter %.c ,$(SOURCES))
COBJS=$(patsubst %.c,${ARCH}/%.o,$(notdir $(CSRCS)))

ASMSRCS=$(filter %.S , $(SOURCES))
ASMOBJS=$(patsubst %.S,${ARCH}/%.o,$(notdir $(ASMSRCS)))

OBJS=$(COBJS) $(ASMOBJS)

all:${ARCH} $(LIB)
	echo "Source path = " $(STM32F_LWIP_SRC_PATH)

$(LIB): $(OBJS)
	$(AR)  rcs  $@ $^

${ARCH}/%.o: $(STM32F_LWIP_SRC_PATH)/%.c
	${COMPILE.c} $(AM_CPPFLAGS) $(AM_CXXFLAGS) -o $@ $<

INSTALL_DIR=$(RTEMS_MAKEFILE_PATH)/stm32f_lwip

install:
	rm -rf $(INSTALL_DIR)
	mkdir -p $(INSTALL_DIR)/include
	mkdir -p $(INSTALL_DIR)/lib
	cp $(LIB) $(INSTALL_DIR)/lib
	cp -r $(STM32F_LWIP_H) $(INSTALL_DIR)

CPPFLAGS+=$(HEADERS)
