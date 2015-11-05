/**
 ******************************************************************************
 * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Inc/httpserver-socket.h
 * @author  MCD Application Team
 * @version V1.2.2
 * @date    25-May-2015
 * @brief   header file for httpserver-socket.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTTPSERVER_SOCKET_H__
#define __HTTPSERVER_SOCKET_H__

#include <cmsis_os.h>

/**
 * @brief  Initialize the HTTP server (start its thread)
 * @param  barrier_id The RTEMS id of the barrier that is released
 *         when the TCP/IP stack startup is complete.
 * @retval None
 */
void http_server_socket_init( void );

/**
 * @brief serve tcp connection
 *
 * This function has an implementation of a basic http server but
 * is defined as a weak symbol which allows applications to define their
 * own application-specific implementation.
 *
 * @param conn: connection socket
 * @retval None
 */
void http_server_serve( int conn );

#endif /* __HTTPSERVER_SOCKET_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
