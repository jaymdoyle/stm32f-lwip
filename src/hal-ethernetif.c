/**
 ******************************************************************************
 * original file:    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/ethernetif.c
 * original author:  MCD Application Team
 * original version: V1.0.0
 * original date:    25-June-2015
 ******************************************************************************
 * @file    hal-ethernetif.c
 * @brief   This file implements RTEMS Ethernet network interface drivers for lwIP
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <bspopts.h>
#include <stm32f7xx_hal.h>
#include <netif/etharp.h>
#include <hal-ethernetif.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( 100 )
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 4 * 1024 )

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

#define LAN8742A_PHY_ADDRESS 0x00
#define DP83848_PHY_ADDRESS             0x01
#define TRACE_ENABLED 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if defined( __ICCARM__ )  /*!< IAR Compiler */

#pragma location=0x2000E000
__no_init ETH_DMADescTypeDef DMARxDscrTab[ ETH_RXBUFNB ];/* Ethernet Rx MA Descriptor */
#pragma location=0x2000E100
__no_init ETH_DMADescTypeDef DMATxDscrTab[ ETH_TXBUFNB ];/* Ethernet Tx DMA Descriptor */
#elif defined( __CC_ARM )
ETH_DMADescTypeDef DMARxDscrTab[ ETH_RXBUFNB ] __attribute__( ( at( 0x2000E000 ) ) );/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef DMATxDscrTab[ ETH_TXBUFNB ] __attribute__( ( at( 0x2000E100 ) ) );/* Ethernet Tx DMA Descriptor */
#elif defined( __GNUC__ )
ETH_DMADescTypeDef DMARxDscrTab[ ETH_RXBUFNB ] __attribute__( ( section(
                                                                  ".bsp_fast_data" ) ) );/* Ethernet Rx MA Descriptor */
ETH_DMADescTypeDef DMATxDscrTab[ ETH_TXBUFNB ] __attribute__( ( section(
                                                                  ".bsp_fast_data" ) ) );/* Ethernet Tx DMA Descriptor */

#endif
#if defined( __ICCARM__ )  /*!< IAR Compiler */
#pragma location=0x2000E200
__no_init uint8_t Rx_Buff[ ETH_RXBUFNB ][ ETH_RX_BUF_SIZE ]; /* Ethernet Receive Buffer */
#pragma location=0x2000FFC4
__no_init uint8_t Tx_Buff[ ETH_TXBUFNB ][ ETH_TX_BUF_SIZE ]; /* Ethernet Transmit Buffer */
#elif defined( __CC_ARM )
uint8_t Rx_Buff[ ETH_RXBUFNB ][ ETH_RX_BUF_SIZE ] __attribute__( ( at(
                                                                     0x2000E200 ) ) ); /* Ethernet Receive Buffer */
uint8_t Tx_Buff[ ETH_TXBUFNB ][ ETH_TX_BUF_SIZE ]  __attribute__( ( at(
                                                                      0x2000FFC4 ) ) ); /* Ethernet Transmit Buffer */
#elif defined( __GNUC__ )
uint8_t Rx_Buff[ ETH_RXBUFNB ][ ETH_RX_BUF_SIZE ] __attribute__( ( section(
                                                                     ".bsp_fast_data" ) ) );/* Ethernet Receive Buffer */
uint8_t Tx_Buff[ ETH_TXBUFNB ][ ETH_TX_BUF_SIZE ] __attribute__( ( section(
                                                                     ".bsp_fast_data" ) ) );/* Ethernet Transmit Buffer */

#endif

/* Semaphore to signal incoming packets */
osSemaphoreId s_xSemaphore = NULL;

static uint32_t num_ethernet_rx_msg = 0UL;
static uint32_t num_ethernet_errors = 0UL;
static uint32_t num_ethernet_tx_msg = 0UL;

/* Global Ethernet handle*/
ETH_HandleTypeDef EthHandle;

/* Private function prototypes -----------------------------------------------*/
static void ethernetif_input( void const *argument );

static void stm32f_ethernet_isr( void *argData )
{
  ETH_HandleTypeDef *pEth =
    (ETH_HandleTypeDef *) argData;

  uint32_t init_rx_count = num_ethernet_rx_msg;

  HAL_ETH_IRQHandler( pEth );

  // This != accounts for the roll over case.
  if(num_ethernet_rx_msg != init_rx_count) {
      osSemaphoreRelease( s_xSemaphore );
  }
}

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
 * @brief  Initializes the ETH MSP.
 * @param  heth: ETH handle
 * @retval None
 */
#ifdef STM32F7_DISCOVERY
void HAL_ETH_MspInit( ETH_HandleTypeDef *heth )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

/* Ethernet pins configuration ************************************************/
/*
      RMII_REF_CLK ----------------------> PA1
      RMII_MDIO -------------------------> PA2
      RMII_MDC --------------------------> PC1
      RMII_MII_CRS_DV -------------------> PA7
      RMII_MII_RXD0 ---------------------> PC4
      RMII_MII_RXD1 ---------------------> PC5
      RMII_MII_RXER ---------------------> PG2
      RMII_MII_TX_EN --------------------> uint32_t stm32f_ethernet_get_num_tx_msg(void)
   {
   return num_ethernet_tx_msg;
   }PG11
      RMII_MII_TXD0 ---------------------> PG13
      RMII_MII_TXD1 ---------------------> PG14
 */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );

  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init( GPIOC, &GPIO_InitStructure );

  /* Configure PG2, PG11, PG13 and PG14 */
  GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_11 | GPIO_PIN_13 |
                           GPIO_PIN_14;
  HAL_GPIO_Init( GPIOG, &GPIO_InitStructure );

  // Install HAL Ethernet ISR
  rtems_interrupt_handler_install(
    ETH_IRQn,
    NULL,
    0,
    stm32f_ethernet_isr,
    heth );

  /* Enable ETHERNET clock  */
  __HAL_RCC_ETH_CLK_ENABLE();
}

/* Initialization code for STM32F756_Eval2 board*/
#elif STM32F7_EVAL2
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  GPIO_InitTypeDef GPIO_InitStructure

  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

/* Ethernet pins configuration ************************************************/
  /*
        ETH_MDIO -------------------------> PA2
        ETH_MDC --------------------------> PC1
        ETH_PPS_OUT ----------------------> PB5
        ETH_MII_RXD2 ---------------------> PH6
        ETH_MII_RXD3 ---------------------> PH7
        ETH_MII_TX_CLK -------------------> PC3
        ETH_MII_TXD2 ---------------------> PC2
        ETH_MII_TXD3 ---------------------> PE2 (or PB8 if trace is enabled)
        ETH_MII_RX_CLK -------------------> PA1
        ETH_MII_RX_DV --------------------> PA7
        ETH_MII_RXD0 ---------------------> PC4
        ETH_MII_RXD1 ---------------------> PC5
        ETH_MII_TX_EN --------------------> PG11
        ETH_MII_TXD0 ---------------------> PG13
        ETH_MII_TXD1 ---------------------> PG14
        ETH_MII_RX_ER --------------------> PI10 (not configured)
        ETH_MII_CRS ----------------------> PA0  (not configured)
        ETH_MII_COL ----------------------> PH3  (not configured)
  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Note : ETH_MDIO is connected to PA2 which is shared with other signals like SAI2_SCKB.
     By default on STM32756G-EVAL board, PA2 is connected to SAI2_SCKB, so to connect PA2 to ETH_MDIO :
    - unsolder bridge SB24 (SAI2_CKB)
    - solder bridge SB5 (ETH_MDIO) */

  /* Configure PB5 */
  GPIO_InitStructure.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* If ETM interface is used then you can't use PE2 for
   * ETH_MII_TX3 because that must be used for TRACE_CLK
   */
#if TRACE_ENABLED
  /* Configure PB8 */
  GPIO_InitStructure.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
  /* Configure PE2 */
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

  /* Configure PC1, PC2, PC3, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Note : ETH_MDC is connected to PC1 which is shared with other signals like SAI1_SDA.
     By default on STM32756G-EVAL board, PC1 is connected to SAI1_SDA, so to connect PC1 to ETH_MDC :
    - unsolder bridge SB22 (SAI1_SDA)
    - solder bridge SB33 (ETH_MDC) */

  /* Configure PG11, PG14 and PG13 */
  GPIO_InitStructure.Pin =  GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Configure PH6, PH7 */
  GPIO_InitStructure.Pin =  GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Configure PA0
  GPIO_InitStructure.Pin =  GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  Note: Ethernet Full duplex mode works properly in the default setting
  (which MII_CRS is not connected to PA0 of STM32F756NGH6) because PA0 is shared
  with MC_ENA.
  If Half duplex mode is needed, uncomment PA0 configuration code source (above
  the note) and close the SB36 solder bridge of the STM32756G-EVAL board .
  */

  /* Configure PH3
  GPIO_InitStructure.Pin =  GPIO_PIN_3;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  Note: Ethernet Full duplex mode works properly in the default setting
  (which MII_COL is not connected to PH3 of STM32F756NGH6) because PH3 is shared
  with SDRAM chip select SDNE0.
  If Half duplex mode is needed, uncomment PH3 configuration code source (above
  the note) and close SB47 solder bridge of the STM32756G-EVAL board.
  */

  /* Configure PI10
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);

  Note: Ethernet works properly in the default setting (which RX_ER is not
  connected to PI10 of STM32F756NGH6) because PI10 is shared with data signal
  of SDRAM.
  If RX_ER signal is needed, uncomment PI10 configuration code source (above
  the note) then remove R248 and solder SB9 of the STM32756G-EVAL board.
  */

  // Install HAL Ethernet ISR
  rtems_interrupt_handler_install(
    ETH_IRQn,
    NULL,
    0,
    stm32f_ethernet_isr,
    heth );

  /* Enable ETHERNET clock  */
  __HAL_RCC_ETH_CLK_ENABLE();
}
#endif



/**
 * @brief  Ethernet Rx Transfer completed callback
 * @param  heth: ETH handle
 * @retval None
 */
void HAL_ETH_RxCpltCallback( ETH_HandleTypeDef *heth )
{
  num_ethernet_rx_msg++;
}

void HAL_ETH_ErrorCallback ( ETH_HandleTypeDef *heth )
{
  num_ethernet_errors++;
}

/**
 * @brief  Ethernet IRQ Handler
 * @param  None
 * @retval None
 */
void ETHERNET_IRQHandler( void )
{
  HAL_ETH_IRQHandler( &EthHandle );
}

uint32_t stm32f_ethernet_get_num_rx_msg( void )
{
  return num_ethernet_rx_msg;
}

uint32_t stm32f_ethernet_get_num_tx_msg( void )
{
  return num_ethernet_tx_msg;
}

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/

// This function should be overloaded in the application code to assign
// the correct ethernet address.  Unless this function is overloaded then
// this default MAC address will be used.
__weak void stm32f_set_mac_addr(uint8_t* macaddress){

  macaddress[0] = MAC_ADDR0;
  macaddress[1] = MAC_ADDR1;
  macaddress[2] = MAC_ADDR2;
  macaddress[3] = MAC_ADDR3;
  macaddress[4] = MAC_ADDR4;
  macaddress[5] = MAC_ADDR5;
}

/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
#ifdef STM32F7_DISCOVERY
static void low_level_init( struct netif *netif )
{
  uint8_t macaddress[ 6 ];

  // Get Ethernet MAC address
  stm32f_set_mac_addr((uint8_t*) macaddress);

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  EthHandle.Init.Speed = ETH_SPEED_100M;
  EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
  EthHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
  EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  EthHandle.Init.PhyAddress = LAN8742A_PHY_ADDRESS;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  if ( HAL_ETH_Init( &EthHandle ) == HAL_OK ) {
    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;
  }

  /* Initialize Tx Descriptors list: Chain Mode */
  HAL_ETH_DMATxDescListInit( &EthHandle,
    DMATxDscrTab,
    &Tx_Buff[ 0 ][ 0 ],
    ETH_TXBUFNB );

  /* Initialize Rx Descriptors list: Chain Mode  */
  HAL_ETH_DMARxDescListInit( &EthHandle,
    DMARxDscrTab,
    &Rx_Buff[ 0 ][ 0 ],
    ETH_RXBUFNB );

  /* set netif MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set netif MAC hardware address */
  stm32f_set_mac_addr((uint8_t*) netif->hwaddr);

  /* set netif maximum transfer unit */
  netif->mtu = 1500;

  /* Accept broadcast address and ARP traffic */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef( SEM , rtems_build_name( 'E', 'T', 'H', 'I' ));
  s_xSemaphore = osSemaphoreCreate( osSemaphore( SEM ), 0 );

  /* create the task that handles the ETH_MAC */
  osThreadDef( EthIf,
    ethernetif_input,
    osPriorityRealtime,
    1,
    INTERFACE_THREAD_STACK_SIZE,
    rtems_build_name( 'E', 'T', 'H', 'I' ));
  osThreadCreate( osThread( EthIf ), netif );

  /* Enable MAC and DMA transmission and reception */
  HAL_ETH_Start( &EthHandle );
}

#elif STM32F7_EVAL2

static void low_level_init(struct netif *netif)
{
  uint32_t regvalue = 0;
  uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  EthHandle.Init.Speed = ETH_SPEED_100M;
  EthHandle.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_MII;
  EthHandle.Init.RxMode = ETH_RXPOLLING_MODE;
  EthHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  EthHandle.Init.PhyAddress = DP83848_PHY_ADDRESS;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  if (HAL_ETH_Init(&EthHandle) == HAL_OK)
  {
    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;
  }

  /* Initialize Tx Descriptors list: Chain Mode */
  HAL_ETH_DMATxDescListInit(&EthHandle, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

  /* Initialize Rx Descriptors list: Chain Mode  */
  HAL_ETH_DMARxDescListInit(&EthHandle, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* Enable MAC and DMA transmission and reception */
  HAL_ETH_Start(&EthHandle);

  /**** Configure PHY to generate an interrupt when Eth Link state changes ****/
  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_MICR, &regvalue);

  regvalue |= (PHY_MICR_INT_EN | PHY_MICR_INT_OE);

  /* Enable Interrupts */
  HAL_ETH_WritePHYRegister(&EthHandle, PHY_MICR, regvalue );

  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&EthHandle, PHY_MISR, &regvalue);

  regvalue |= PHY_MISR_LINK_INT_EN;

  /* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef( SEM , rtems_build_name( 'E', 'T', 'H', 'I' ));
  s_xSemaphore = osSemaphoreCreate( osSemaphore( SEM ), 0 );

  /* create the task that handles the ETH_MAC */
  osThreadDef( EthIf,
    ethernetif_input,
    osPriorityRealtime,
    1,
    INTERFACE_THREAD_STACK_SIZE,
    rtems_build_name( 'E', 'T', 'H', 'I' ));
  osThreadCreate( osThread( EthIf ), netif );

  /* Enable Interrupt on change of link status */
  HAL_ETH_WritePHYRegister(&EthHandle, PHY_MISR, regvalue);
}


#endif



/**
 * @brief This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(
  struct netif *netif,
  struct pbuf  *p
)
{
  err_t        errval;
  struct pbuf *q;
  uint8_t     *buffer =
    (uint8_t *) ( EthHandle.TxDesc->Buffer1Addr );
  __IO ETH_DMADescTypeDef *DmaTxDesc;
  uint32_t                 framelength = 0;
  uint32_t                 bufferoffset = 0;
  uint32_t                 byteslefttocopy = 0;
  uint32_t                 payloadoffset = 0;

  DmaTxDesc = EthHandle.TxDesc;
  bufferoffset = 0;

  /* copy frame from pbufs to driver buffers */
  for ( q = p; q != NULL; q = q->next ) {
    /* Is this buffer available? If not, goto error */
    if ( ( DmaTxDesc->Status & ETH_DMATXDESC_OWN ) != (uint32_t) RESET ) {
      errval = ERR_USE;
      goto error;
    }

    /* Get bytes in current lwIP buffer */
    byteslefttocopy = q->len;
    payloadoffset = 0;

    /* Check if the length of data to copy is bigger than Tx buffer size*/
    while ( ( byteslefttocopy + bufferoffset ) > ETH_TX_BUF_SIZE ) {
      /* Copy data to Tx buffer*/
      memcpy( (uint8_t *) ( (uint8_t *) buffer + bufferoffset ),
        (uint8_t *) ( (uint8_t *) q->payload + payloadoffset ),
        ( ETH_TX_BUF_SIZE - bufferoffset ) );

      /* Point to next descriptor */
      DmaTxDesc = (ETH_DMADescTypeDef *) ( DmaTxDesc->Buffer2NextDescAddr );

      /* Check if the buffer is available */
      if ( ( DmaTxDesc->Status & ETH_DMATXDESC_OWN ) != (uint32_t) RESET ) {
        errval = ERR_USE;
        goto error;
      }

      buffer = (uint8_t *) ( DmaTxDesc->Buffer1Addr );

      byteslefttocopy = byteslefttocopy - ( ETH_TX_BUF_SIZE - bufferoffset );
      payloadoffset = payloadoffset + ( ETH_TX_BUF_SIZE - bufferoffset );
      framelength = framelength + ( ETH_TX_BUF_SIZE - bufferoffset );
      bufferoffset = 0;
    }

    /* Copy the remaining bytes */
    memcpy( (uint8_t *) ( (uint8_t *) buffer + bufferoffset ),
      (uint8_t *) ( (uint8_t *) q->payload + payloadoffset ),
      byteslefttocopy );
    bufferoffset = bufferoffset + byteslefttocopy;
    framelength = framelength + byteslefttocopy;
  }

  /* Prepare transmit descriptors to give to DMA */
  HAL_ETH_TransmitFrame( &EthHandle, framelength );
  num_ethernet_tx_msg++;

  errval = ERR_OK;

error:

  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
  if ( ( EthHandle.Instance->DMASR & ETH_DMASR_TUS ) != (uint32_t) RESET ) {
    /* Clear TUS ETHERNET DMA flag */
    EthHandle.Instance->DMASR = ETH_DMASR_TUS;

    /* Resume DMA transmission*/
    EthHandle.Instance->DMATPDR = 0;
  }

  return errval;
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input( struct netif *netif )
{
  struct pbuf             *p = NULL, *q = NULL;
  uint16_t                 len = 0;
  uint8_t                 *buffer;
  __IO ETH_DMADescTypeDef *dmarxdesc;
  uint32_t                 bufferoffset = 0;
  uint32_t                 payloadoffset = 0;
  uint32_t                 byteslefttocopy = 0;
  uint32_t                 i = 0;

  /* get received frame */
  if ( HAL_ETH_GetReceivedFrame_IT( &EthHandle ) != HAL_OK )
    return NULL;

  /* Obtain the size of the packet and put it into the "len" variable. */
  len = EthHandle.RxFrameInfos.length;
  buffer = (uint8_t *) EthHandle.RxFrameInfos.buffer;

  if ( len > 0 ) {
    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    p = pbuf_alloc( PBUF_RAW, len, PBUF_POOL );
  }

  if ( p != NULL ) {
    dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;
    bufferoffset = 0;

    for ( q = p; q != NULL; q = q->next ) {
      byteslefttocopy = q->len;
      payloadoffset = 0;

      /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size */
      while ( ( byteslefttocopy + bufferoffset ) > ETH_RX_BUF_SIZE ) {
        /* Copy data to pbuf */
        memcpy( (uint8_t *) ( (uint8_t *) q->payload + payloadoffset ),
          (uint8_t *) ( (uint8_t *) buffer + bufferoffset ),
          ( ETH_RX_BUF_SIZE - bufferoffset ) );

        /* Point to next descriptor */
        dmarxdesc = (ETH_DMADescTypeDef *) ( dmarxdesc->Buffer2NextDescAddr );
        buffer = (uint8_t *) ( dmarxdesc->Buffer1Addr );

        byteslefttocopy = byteslefttocopy - ( ETH_RX_BUF_SIZE - bufferoffset );
        payloadoffset = payloadoffset + ( ETH_RX_BUF_SIZE - bufferoffset );
        bufferoffset = 0;
      }

      /* Copy remaining data in pbuf */
      memcpy( (uint8_t *) ( (uint8_t *) q->payload + payloadoffset ),
        (uint8_t *) ( (uint8_t *) buffer + bufferoffset ), byteslefttocopy );
      bufferoffset = bufferoffset + byteslefttocopy;
    }
  }

  /* Release descriptors to DMA */
  /* Point to first descriptor */
  dmarxdesc = EthHandle.RxFrameInfos.FSRxDesc;

  /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
  for ( i = 0; i < EthHandle.RxFrameInfos.SegCount; i++ ) {
    dmarxdesc->Status |= ETH_DMARXDESC_OWN;
    dmarxdesc = (ETH_DMADescTypeDef *) ( dmarxdesc->Buffer2NextDescAddr );
  }

  /* Clear Segment_Count */
  EthHandle.RxFrameInfos.SegCount = 0;

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ( ( EthHandle.Instance->DMASR & ETH_DMASR_RBUS ) != (uint32_t) RESET ) {
    /* Clear RBUS ETHERNET DMA flag */
    EthHandle.Instance->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    EthHandle.Instance->DMARPDR = 0;
  }

  return p;
}

/**
 * @brief This function is the ethernetif_input task, it is processed when a packet
 * is ready to be read from the interface. It uses the function low_level_input()
 * that should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input( void const *argument )
{
  struct pbuf  *p;
  struct netif *netif = (struct netif *) argument;

  for (;; ) {
    if ( osSemaphoreWait( s_xSemaphore, TIME_WAITING_FOR_INPUT ) == osOK ) {
      do {
        p = low_level_input( netif );

        if ( p != NULL ) {
          if ( netif->input( p, netif ) != ERR_OK ) {
            pbuf_free( p );
          }
        }
      } while ( p != NULL );
    }
  }
}

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
 *         ERR_ARG if the input argument is NULL
 */
err_t ethernetif_init( struct netif *netif )
{
  LWIP_ASSERT( "netif != NULL", ( netif != NULL ) );

  if ( netif == NULL ) {
    return ERR_ARG;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[ 0 ] = IFNAME0;
  netif->name[ 1 ] = IFNAME1;

  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init( netif );

  return ERR_OK;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
