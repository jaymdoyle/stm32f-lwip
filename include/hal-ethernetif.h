/**
 * @file hal-ethernetif.h
 *
 * @ingroup ethernet
 *
 * @brief A set of utility functions used in conjunction with lwip
 *  TCPIP stack.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include <lwip/err.h>
#include <lwip/netif.h>
#include <cmsis_os.h>

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init( struct netif *netif );

/**
 * @brief  Ethernet IRQ Handler
 */
void ETHERNET_IRQHandler( void );

/**
 * @brief A function to return the number of messages received via
 *  the Ethernet interface since start up.
 *
 * @return The number of Ethernet messages received since start up.
 */
uint32_t stm32f_ethernet_get_num_rx_msg( void );

/**
 * @brief A function to return the number of messages sent via
 *  the Ethernet interface since start up.
 *
 * @return The number of Ethernet messages sent out since start up.
 */
uint32_t stm32f_ethernet_get_num_tx_msg( void );

#endif
