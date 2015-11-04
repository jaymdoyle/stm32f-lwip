/**
 ******************************************************************************
 * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Src/httpserver-socket.c
 * @author  MCD Application Team
 * @version V1.2.2
 * @date    25-May-2015
 * @brief   Basic http server implementation using LwIP socket API
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

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "string.h"
#include "cmsis_os.h"
#include <httpserver-socket.h>
#include <hal-ethernetif.h>
#include <cmsis_os.h>
#include <hal-utils.h>
#include <rtems/stackchk.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO ( tskIDLE_PRIORITY + 4 )
#define WEBSERVER_STACK_SIZE ( 10 * 1024 )
#define WEBSERVER_RX_BUFFER_SIZE 1500

u32_t nPageHits = 0;

/* Private function prototypes -----------------------------------------------*/
static void http_server_socket_thread( void *arg );

/**
 * @brief generates an HTML page with platform diagnostic information
 *
 * @retval A pointer to string containing the entire HTML content for
 *   a platform diagnostic page.
 */
static portCHAR *http_generate_platform_stats_page( void );

/**
 * @brief generates a HTML page with a generic main
 *  page for the http server
 *
 * @retval A pointer to string containing the entire HTML content for
 *   a main page
 */
static portCHAR *http_generate_main_page( void );

/**
 * @brief generates an HTML page with a generic 404 error
 *
 * @retval A pointer to string containing the entire HTML content for
 *   the 404 error page
 */
static portCHAR *http_generate_error_page( void );

/* Private Data ---------------------------------------------------------*/
static portCHAR DYNAMIC_PAGE_CONTENT[ 10 * 1024 ];
static portCHAR PAGE_HEADER_DYNAMIC[ 512 ] =
  "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN"
  "http://www.w3.org/TR/html4/strict.dtd\"><html><head><title>Next Gen Control Platform Task List</title><meta http-equiv=\"Content-Type\"content=\"text/html; charset=windows-1252\"><meta http-equiv=\"refresh\" content=\"0.5\"><style =\"font-weight: normal; font-family: Verdana;\"></style></head><body><h1>List of NextGen Tasks</h1><a href=\"/\">Main Page</a><p>";
static portCHAR main_page[ 4096 ] =
  "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN"
  "http://www.w3.org/TR/html4/strict.dtd\"><html><head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"><title>Next Gen Main Page</title></head><body lang=\"en-US\" dir=\"ltr\" style=\"background: transparent\"><h1><font face=\"Courier 10 Pitch\">Vecna NextGen Platform Web Server</font></h1><p><a href=\"/list_of_tasks.html\">List of tasks</a></body></html>";
static portCHAR error_page[ 4096 ] =
  "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN"
  "http://www.w3.org/TR/html4/strict.dtd\"><html><head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"><title></title></head><body lang=\"en-US\" dir=\"ltr\" style=\"background: transparent\"><h1><font face=\"Courier 10 Pitch\">404: Requested page does not exist</font></h1><p><br><br></p><a href=\"/\">Main page</a></body></html>";
static osThreadDef_t web_server_task;

__attribute__( ( weak ) ) bool http_server_server_app_specific(
  char      *recv_buffer,
  portCHAR **output_buffer
)
{
  // override this function in the application to process any non-default

  // pages.
  return false;
}

/**
 * @brief  Initialize the HTTP server (start its thread)
 * @param  none
 * @retval None
 */
void http_server_socket_init(rtems_id barrier_id)
{
  web_server_task.pthread = http_server_socket_thread;
  web_server_task.tpriority = osPriorityNormal;
  web_server_task.instances = 1;
  web_server_task.stacksize = WEBSERVER_STACK_SIZE;
  web_server_task.thread_name = rtems_build_name( 'W', 'E', 'B', 'S' );

  osThreadCreate( &web_server_task, &barrier_id );
}

/**
 * @brief serve tcp connection
 * @param conn: connection sockestm32f_ethernet_get_num_rx_msgt
 * @retval None
 */
__attribute__( ( weak ) )  void http_server_serve( int conn )
{
  int       ret;
  char      recv_buffer[ WEBSERVER_RX_BUFFER_SIZE ];
  portCHAR *pageContent;

  // Read in the request
  ret = read( conn, recv_buffer, COUNTOF( recv_buffer ) );

  if ( ret < 0 )
    return;

  if ( strncmp( recv_buffer, "GET /list_of_tasks.html", 23 ) == 0 ) {
    // load dynamic page with list RTEMS tasks
    pageContent = http_generate_platform_stats_page();
  } else if ( strncmp( recv_buffer, "GET / ", 6 ) == 0 ) {
    // load main page
    pageContent = http_generate_main_page();
  } else if ( http_server_server_app_specific( recv_buffer,
                &pageContent ) == false ) {
    pageContent = http_generate_error_page();
  }

  // write page content to socket
  write( conn, pageContent, strlen( pageContent ) );

  // close connection socket
  close( conn );
}

/* Private Functions ---------------------------------------------------------*/
/**
 * @brief  http server thread
 * @param arg: pointer on argument(not used here)
 * @retval None
 */
static void http_server_socket_thread( void *arg )
{
  int                sock, newconn, size;
  struct sockaddr_in address, remotehost;
  rtems_id           tcpip_barrier_id = *((rtems_id*) arg);

  rtems_barrier_wait(tcpip_barrier_id, RTEMS_NO_TIMEOUT);

  /* create a TCP socket */
  if ( ( sock = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
    return;
  }

  /* bind to port 80 at any interface */
  address.sin_family = AF_INET;
  address.sin_port = htons( 80 );
  address.sin_addr.s_addr = INADDR_ANY;

  if ( bind( sock, (struct sockaddr *) &address, sizeof( address ) ) < 0 ) {
    return;
  }

  /* listen for incoming connections (TCP listen backlog = 5) */
  listen( sock, 5 );

  size = sizeof( remotehost );

  while ( 1 ) {
    newconn = accept( sock,
      (struct sockaddr *) &remotehost,
      (socklen_t *) &size );

    if(newconn >= 0) {
      http_server_serve( newconn );
    }
  }
}

static portCHAR *http_generate_platform_stats_page( void )
{
  portCHAR dynamic_text[ 128 ] = { 0 };
  portCHAR headerRow[ 128 ];
  rtems_interval elapsed_time =  rtems_clock_get_ticks_since_boot() / rtems_clock_get_ticks_per_second();
  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;

  hours = elapsed_time / 3600UL;
  elapsed_time -= hours * 3600UL;
  minutes = elapsed_time / 60;
  elapsed_time -= minutes * 60;
  seconds = elapsed_time;

  memset( DYNAMIC_PAGE_CONTENT, 0, sizeof( DYNAMIC_PAGE_CONTENT ) );

  /* Update the hit count */
  nPageHits++;
  strncat( DYNAMIC_PAGE_CONTENT, PAGE_HEADER_DYNAMIC,
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );
  snprintf( dynamic_text,
    sizeof( dynamic_text ),
    "Page refresh count %d",
    (int) nPageHits );
  strncat( DYNAMIC_PAGE_CONTENT,
    dynamic_text,
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );

  snprintf( dynamic_text,
    sizeof( dynamic_text ),
    "<p>Number of Ethernet Packets (RX: %lu, TX %lu)",
    stm32f_ethernet_get_num_rx_msg(),
    stm32f_ethernet_get_num_tx_msg() );
  strncat( DYNAMIC_PAGE_CONTENT,
    dynamic_text,
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );

  snprintf( dynamic_text,
    sizeof( dynamic_text ),
    "<p>Uptime: %lu hours, %lu minutes, %lu second",  hours, minutes, seconds);
  strncat( DYNAMIC_PAGE_CONTENT,
    dynamic_text,
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );

#ifdef REPORT_STACK_USAGE
  static uint32_t last_stack_report = 0;

  if(last_stack_report != minutes) {
    rtems_stack_checker_report_usage();
    last_stack_report = minutes;
  }
#endif

  snprintf( headerRow,
    sizeof( headerRow ),
    "<pre><br>%4s\t%16s\t%8s\t%10s\t%8s",
    "Name",
    "Task State",
    "Stk Addr",
    "Stk Sz",
    "rtems_id" );
  strncat( (char *) DYNAMIC_PAGE_CONTENT, headerRow,
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );
  strncat( (char *) DYNAMIC_PAGE_CONTENT,
    "<br>------------------------------------------------------------------------------<br>",
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );

  /* The list of tasks and their status */
  osThreadList( (unsigned char *) ( DYNAMIC_PAGE_CONTENT +
                                    strlen( DYNAMIC_PAGE_CONTENT ) ) );
  strncat( (char *) DYNAMIC_PAGE_CONTENT,
    "<br>--------------------------------------------------------------------------</pre></body></html>",
    sizeof( DYNAMIC_PAGE_CONTENT ) - strlen( DYNAMIC_PAGE_CONTENT ) );

  return (portCHAR *) DYNAMIC_PAGE_CONTENT;
}

static portCHAR *http_generate_main_page( void )
{
  return (portCHAR *) main_page;
}

static portCHAR *http_generate_error_page( void )
{
  return (portCHAR *) error_page;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
