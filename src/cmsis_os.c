/*----------------------------------------------------------------------------
*      CMSIS-RTOS  -  RTX
*----------------------------------------------------------------------------
*      Name:    rt_CMSIS.c
*      Purpose: CMSIS RTOS API
*      Rev.:    V4.78
*----------------------------------------------------------------------------
*
* Copyright (c) 1999-2009 KEIL, 2009-2015 ARM Germany GmbH
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of ARM  nor the names of its contributors may be used
*    to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*---------------------------------------------------------------------------*/

#define __CMSIS_GENERIC

#define os_thread_cb rtems_id
#define NUM_CMSIS_TASK_PRIORITIES 7

#include <cmsis_os.h>
#include <sched.h>
#include <rtems/score/statesimpl.h>
#include <stdio.h>
#include <hal-error.h>

#if ( osFeature_Semaphore > 65535 )
#error Invalid "osFeature_Semaphore" value!
#endif

// ==== Enumeration, structures, defines ====

// Service Calls defines

#define __NO_RETURN __attribute__( ( noreturn ) )

typedef uint32_t __attribute__( ( vector_size( 8 ) ) )  ret64;
typedef uint32_t __attribute__( ( vector_size( 16 ) ) ) ret128;

// Callback structure
typedef struct {
  void *fp;             // Function pointer
  void *arg;            // Function argument
} osCallback;

// OS Section definitions
#ifdef OS_SECTIONS_LINK_INFO
extern const uint32_t os_section_id$$Base;
extern const uint32_t os_section_id$$Limit;
#endif

// OS Stack Memory for Threads definitions
extern uint64_t       os_stack_mem[];
extern const uint32_t os_stack_sz;

// OS Timers external resources
extern const osThreadDef_t   os_thread_def_osTimerThread;
extern osThreadId            osThreadId_osTimerThread;
extern const osMessageQDef_t os_messageQ_def_osTimerMessageQ;
extern osMessageQId          osMessageQId_osTimerMessageQ;

static uint8_t         *pThreadListBuffer = NULL;
static uint32_t         num_tasks = 0UL;
static rtems_id         target_thread_id = 0UL;
static Priority_Control target_thread_real_priority = 0UL;
static rtems_id         utils_mutex_id;
static bool             utils_init = false;

static osStatus rtemsConvertReturnCode( const rtems_status_code rtems_ret )
{
  osStatus ret;

  switch ( rtems_ret ) {
    case RTEMS_SUCCESSFUL:
      ret = osOK;
      break;

    case RTEMS_INVALID_ID:
      ret = osErrorParameter;
      break;

    case RTEMS_RESOURCE_IN_USE:
      ret = osErrorResource;
      break;

    case RTEMS_ILLEGAL_ON_REMOTE_OBJECT:
      ret = osErrorParameter;
      break;

    case RTEMS_TIMEOUT:
      ret = osEventTimeout;
      break;

    case RTEMS_TASK_EXITTED:
    case RTEMS_MP_NOT_CONFIGURED:
    case RTEMS_INVALID_NAME:
    case RTEMS_TOO_MANY:
    case RTEMS_OBJECT_WAS_DELETED:
    case RTEMS_INVALID_SIZE:
    case RTEMS_INVALID_ADDRESS:
    case RTEMS_INVALID_NUMBER:
    case RTEMS_NOT_DEFINED:
    case RTEMS_UNSATISFIED:
    case RTEMS_INCORRECT_STATE:
    case RTEMS_ALREADY_SUSPENDED:
    case RTEMS_ILLEGAL_ON_SELF:
    case RTEMS_CALLED_FROM_ISR:
    case RTEMS_INVALID_PRIORITY:
    case RTEMS_INVALID_CLOCK:
    case RTEMS_INVALID_NODE:
    case RTEMS_NOT_CONFIGURED:
    case RTEMS_NOT_OWNER_OF_RESOURCE:
    case RTEMS_NOT_IMPLEMENTED:
    case RTEMS_INTERNAL_ERROR:
    case RTEMS_NO_MEMORY:
    case RTEMS_IO_ERROR:
    case RTEMS_PROXY_BLOCKING:
      ret = osErrorOS;
      break;
  }

  return ret;
}

// ==== Kernel Control ====

uint8_t os_initialized = 1;                         // Kernel Initialized flag
uint8_t os_running     = 1;                         // Kernel Running flag

// Kernel Control Public API

/// Initialize the RTOS Kernel for creating objects
osStatus osKernelInitialize( void )
{
  // Not implemented yet
  return osErrorOS;
}

/// Start the RTOS Kernel
osStatus osKernelStart( void )
{
  // Not implemented yet
  return osErrorOS;
}

/// Check if the RTOS kernel is already started
int32_t osKernelRunning( void )
{
  return os_running;
}

/// Get the RTOS kernel system timer counter
uint32_t osKernelSysTick( void )
{
  return (uint32_t) rtems_clock_get_ticks_since_boot();
}

__NO_RETURN void osThreadExit( void );

// Thread Public API

static rtems_task_priority convertTaskPriorityToRTEMS(
  const osPriority priority )
{
  rtems_task_priority ret;
  uint8_t             task_priority_step_size =
    ( PRIORITY_MAXIMUM - PRIORITY_MINIMUM ) /
    (uint8_t) ( NUM_CMSIS_TASK_PRIORITIES - 1 );

  // In RTEMS the higher the numerical value the lower the
  // priority.  RTEMS provide macros for highest and lowest
  // priority level.
  if ( priority != osPriorityError ) {
    ret =
      (rtems_task_priority) ( PRIORITY_MAXIMUM -
                              ( (uint8_t) ( priority - osPriorityIdle ) *
                                task_priority_step_size ) );
  } else {
    stm32f_error_handler_with_reason_conditional(
      "convertTaskPriority: Invalid task priority\n",
      false );
    ret = PRIORITY_DEFAULT_MAXIMUM + 1;
  }

  return ret;
}

static osPriority convertTaskPriorityToCMSIS(
  const rtems_task_priority priority )
{
  osPriority ret;
  uint8_t    task_priority_step_size =
    ( PRIORITY_MAXIMUM - PRIORITY_MINIMUM ) /
    (uint8_t) ( NUM_CMSIS_TASK_PRIORITIES - 1 );

  if ( ( priority <= PRIORITY_MAXIMUM ) && ( priority >= PRIORITY_MINIMUM ) ) {
    ret =
      ( ( ( PRIORITY_MAXIMUM - priority ) / task_priority_step_size ) +
        osPriorityIdle );
  } else {
    ret = osPriorityError;
  }

  return ret;
}

/// Create a thread and add it to Active Threads and set it to state READY
osThreadId osThreadCreate(
  const osThreadDef_t *thread_def,
  void                *argument
)
{
  rtems_status_code   ret;
  rtems_task_priority task_priority = convertTaskPriorityToRTEMS(
    thread_def->tpriority );
  rtems_id new_task_id = 0;

  if ( ( thread_def->pthread != NULL ) &&
       ( task_priority <= PRIORITY_MAXIMUM ) &&
       ( thread_def->instances > 0 ) &&
       ( thread_def->stacksize > 0 ) ) {

    // Create the rtems task.
    ret = rtems_task_create(
      thread_def->thread_name,
      task_priority,
      (size_t) thread_def->stacksize,
      RTEMS_DEFAULT_MODES,
      RTEMS_DEFAULT_ATTRIBUTES,
      &new_task_id
          );

    // Start the task if the create was successful
    if ( ret == RTEMS_SUCCESSFUL ) {

      ret = rtems_task_start(
        new_task_id,
        (rtems_task_entry) thread_def->pthread,
        (rtems_task_argument) argument
            );
    }
  }

  return (osThreadId) new_task_id;
}

/// Return the thread ID of the current running thread
osThreadId osThreadGetId( void )
{
  return (osThreadId) rtems_task_self();
}

/// Terminate execution of a thread and remove it from ActiveThreads
osStatus osThreadTerminate( osThreadId thread_id )
{
  osStatus ret = osOK;

  if ( rtems_task_delete( (rtems_id) thread_id ) != RTEMS_SUCCESSFUL ) {
    ret = osErrorParameter;
  }

  return ret;
}

/// Pass control to next thread that is in state READYosThreadGetPriority
osStatus osThreadYield( void )
{
  sched_yield();

  return osOK;
}

/// Change priority of an active thread
osStatus osThreadSetPriority(
  osThreadId thread_id,
  osPriority priority
)
{
  rtems_task_priority task_priority = convertTaskPriorityToRTEMS( priority );
  rtems_task_priority old_task_priority;
  rtems_status_code   rtems_ret;

  rtems_ret = rtems_task_set_priority(
    (rtems_id) thread_id,
    task_priority,
    &old_task_priority
              );

  return rtemsConvertReturnCode( rtems_ret );
}

/// Get current priority of an active thread
osPriority osThreadGetPriority( osThreadId thread_id )
{
  return convertTaskPriorityToCMSIS( tasks_get_priority( (rtems_id) thread_id ) );
}

static void osThreadStateName(
  const States_Control state,
  char                *state_name
)
{
  if ( _States_Is_ready( state ) == true ) {
    sprintf( state_name, "READY" );
  } else if ( _States_Is_suspended( state ) == true ) {
    sprintf( state_name, "SUSPENDED" );
  } else if ( _States_Is_delaying( state ) == true ) {
    sprintf( state_name, "DELAYING" );
  } else if ( _States_Is_waiting_for_buffer( state ) == true ) {
    sprintf( state_name, "WAIT (BUF)" );
  } else if ( _States_Is_waiting_for_segment( state ) == true ) {
    sprintf( state_name, "WAIT (SEG)" );
  } else if ( _States_Is_waiting_for_message( state ) == true ) {
    sprintf( state_name, "WAIT (MSG)" );
  } else if ( _States_Is_waiting_for_event( state ) == true ) {
    sprintf( state_name, "WAIT (EVT)" );
  } else if ( _States_Is_waiting_for_system_event( state ) == true ) {
    sprintf( state_name, "WAIT (SEVT)" );
  } else if ( _States_Is_waiting_for_mutex( state ) == true ) {
    sprintf( state_name, "WAIT (MUX)" );
  } else if ( _States_Is_waiting_for_semaphore( state ) == true ) {
    sprintf( state_name, "WAIT (SEM)" );
  } else if ( _States_Is_waiting_for_time( state ) == true ) {
    sprintf( state_name, "WAIT (TIME)" );
  } else if ( _States_Is_waiting_for_rpc_reply( state ) == true ) {
    sprintf( state_name, "WAIT (RPC)" );
  } else if ( _States_Is_waiting_for_period( state ) == true ) {
    sprintf( state_name, "WAIT (PER)" );
  } else if ( _States_Is_interruptible_by_signal( state ) == true ) {
    sprintf( state_name, "INT (SIG)" );
  } else {
    sprintf( state_name, "???" );
  }
}

static void utils_init_mutex( void )
{
  rtems_attribute   mutex_attributes;
  rtems_status_code ret;

  if ( utils_init == false ) {
    mutex_attributes = RTEMS_PRIORITY | RTEMS_LOCAL | RTEMS_INHERIT_PRIORITY |
                       RTEMS_BINARY_SEMAPHORE |
                       RTEMS_NO_PRIORITY_CEILING |
                       RTEMS_NO_MULTIPROCESSOR_RESOURCE_SHARING;

    ret = rtems_semaphore_create(
      rtems_build_name( 'C', 'M', 'X', 'T' ),
      1,
      mutex_attributes,
      PRIORITY_DEFAULT_MAXIMUM,
      &utils_mutex_id
          );

    if ( ret == RTEMS_SUCCESSFUL ) {
      utils_init = true;
    }
  }
}

/**
 *  @brief This function is called by an RTEMS task iterator that
 *  calls this function for each task control block.  Each invocation
 *  check to see to see if the task control block is valid (i.e., non-null
 *  pointer) and then increment the task count. A mutex in the calling function prevents other tasks
 *  from interrupting the current search across the task control block list.
 *
 *  @param the_thread The next task control block to check
 */
static void task_counter_increment_check( Thread_Control *the_thread )
{
  if ( the_thread != NULL) {
    num_tasks++;
  }
}


/**
 *  @brief This function is called by an RTEMS task iterator that
 *  calls this function for each task control block.  Each invocation
 *  check to see if the the current task control block refers to the
 *  task for which we wish to know the priority.  If so, then save the value
 *  to a local static value.  A mutex in the calling function prevents other tasks
 *  from interrupting the current search across the task control block list.
 *
 *  @param the_thread The next task control block to check
 */
static void task_priority_request_check( Thread_Control *the_thread )
{
  if ( the_thread ) {
    if ( the_thread->Object.id == target_thread_id ) {
      target_thread_real_priority = the_thread->real_priority;
    }
  }
}

uint32_t tasks_get_num( void )
{
  uint32_t ret = 0UL;

  if ( utils_init == false ) {
    utils_init_mutex();
  }

  rtems_semaphore_obtain( utils_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );

  num_tasks = 0UL;
  rtems_iterate_over_all_threads( task_counter_increment_check );
  ret = num_tasks;

  rtems_semaphore_release( utils_mutex_id );

  return ret;
}

Priority_Control tasks_get_priority( const rtems_id target_task )
{
  Priority_Control ret;

  if ( utils_init == false ) {
    utils_init_mutex();
  }

  rtems_semaphore_obtain( utils_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );

  target_thread_id = target_task;
  target_thread_real_priority = PRIORITY_MAXIMUM;
  rtems_iterate_over_all_threads( task_priority_request_check );
  ret = target_thread_real_priority;

  rtems_semaphore_release( utils_mutex_id );

  return ret;
}

static void osThreadListIterator( Thread_Control *the_thread )
{
  char NextThreadInfo[ 128 ];
  char thread_name[ 5 ];
  char thread_state[ 32 ];

  if ( the_thread ) {
    osThreadStateName( the_thread->current_state, (char *) thread_state );
    rtems_object_get_name( the_thread->Object.id,
      sizeof( thread_name ),
      thread_name );

    snprintf( (char *) NextThreadInfo, sizeof( NextThreadInfo ) - 1,
      "%4s\t%16s\t0x%.08X\t%6luKB\t0x%.08X \n\r",
      thread_name,
      thread_state,
      (unsigned int) the_thread->Start.stack,
      (uint32_t) ( the_thread->Start.Initial_stack.size / 1024 ),
      (unsigned int) the_thread->Object.id
    );
  }

  strcat( (char *) pThreadListBuffer, (char *) NextThreadInfo );
}

/**
 * @brief   Lists all the current threads, along with their current state
 *          and stack usage high water mark.
 * @param   buffer   A buffer into which the above mentioned details
 *          will be written
 * @retval  status code that indicates the execution status of the function.
 */
osStatus osThreadList( uint8_t *buffer )
{
  if ( utils_init == false ) {
    utils_init_mutex();
  }

  rtems_semaphore_obtain( utils_mutex_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );

  pThreadListBuffer = buffer;
  rtems_iterate_over_all_threads( osThreadListIterator );

  rtems_semaphore_release( utils_mutex_id );

  return osOK;
}

/// INTERNAL - Not Public
/// Auto Terminate Thread on exit (used implicitly when thread exists)
__NO_RETURN void osThreadExit( void )
{
  rtems_id my_task_id;

  if ( rtems_task_ident( RTEMS_SELF, RTEMS_SEARCH_ALL_NODES,
         &my_task_id ) == RTEMS_SUCCESSFUL ) {
    rtems_task_delete( my_task_id );
  }

  for (;; ) {
    // should never get here
  }
}

// Generic Wait API

/// Wait for Timeout (Time Delay)
osStatus osDelay( uint32_t millisec )
{
  rtems_interval ticks_per_millisec;

  ticks_per_millisec = rtems_clock_get_ticks_per_second() / 1000;

  (void) rtems_task_wake_after( millisec * ticks_per_millisec );

  return osOK;
}

/// Wait for Signal, Message, Mail, or Timeout
os_InRegs osEvent osWait( uint32_t millisec )
{
  // Not implemented yet
  osEvent ret = { osErrorOS, { 0 }, { 0 } };

  return ret;
}

// ==== Timer Management ====

// Timer definitions
#define osTimerInvalid 0
#define osTimerStopped 1
#define osTimerRunning 2

// Timer structures

typedef struct os_timer_cb_ {                   // Timer Control Block
  struct os_timer_cb_ *next;                    // Pointer to next active Timer
  uint8_t state;                                // Timer State
  uint8_t type;                                 // Timer Type (Periodic/One-shot)
  uint16_t reserved;                            // Reserved
  uint32_t tcnt;                                // Timer Delay Count
  uint32_t icnt;                                // Timer Initial Count
  void *arg;                                    // Timer Function Argument
  const osTimerDef_t *timer;                    // Pointer to Timer definition
} os_timer_cb;

// Timer variables
os_timer_cb *os_timer_head;                     // Pointer to first active Timer

// Timer Management Public API

/// Create timer
osTimerId osTimerCreate(
  const osTimerDef_t *timer_def,
  os_timer_type       type,
  void               *argument
)
{
  return (osTimerId) 0;
}

/// Start or restart timer
osStatus osTimerStart(
  osTimerId timer_id,
  uint32_t  millisec
)
{
  // Not implemented yet
  return osErrorOS;
}

/// Stop timer
osStatus osTimerStop( osTimerId timer_id )
{
  // Not implemented yet
  return osErrorOS;
}

/// Delete timer
osStatus osTimerDelete( osTimerId timer_id )
{
  // Not implemented yet
  return osErrorOS;
}

// Signal Public API

/// Set the specified Signal Flags of an active thread
int32_t osSignalSet(
  osThreadId thread_id,
  int32_t    signals
)
{
  // To prevent unused argument compiler warnings ...
  (void) thread_id;
  (void) signals;

  // Not implemented yet
  return 0L;
}

/// Clear the specified Signal Flags of an active thread
int32_t osSignalClear(
  osThreadId thread_id,
  int32_t    signals
)
{
  // To prevent unused argument compiler warnings ...
  (void) thread_id;
  (void) signals;

  // Not implemented yet
  return 0L;
}

/// Wait for one or more Signal Flags to become signaled for the current RUNNING thread
os_InRegs osEvent osSignalWait(
  int32_t  signals,
  uint32_t millisec
)
{
  // To prevent unused argument compiler warnings ...
  (void) signals;
  (void) millisec;

  // Not implemented yet
  osEvent ret = { osErrorOS, { 0 }, { 0 } };

  return ret;
}

// Mutex Public API

/// Create and Initialize a Mutex object
osMutexId osMutexCreate( const osMutexDef_t *mutex_def )
{

  rtems_attribute   mutex_attributes;
  rtems_status_code ret;
  rtems_id          mutex_id;

  mutex_attributes = RTEMS_PRIORITY | RTEMS_LOCAL | RTEMS_INHERIT_PRIORITY |
                     RTEMS_BINARY_SEMAPHORE |
                     RTEMS_NO_PRIORITY_CEILING |
                     RTEMS_NO_MULTIPROCESSOR_RESOURCE_SHARING;

  ret = rtems_semaphore_create(
    mutex_def->mutex_name,
    1,
    mutex_attributes,
    PRIORITY_DEFAULT_MAXIMUM,
    &mutex_id
        );

  if ( ret == RTEMS_SUCCESSFUL ) {

    return (osMutexId) mutex_id;
  } else {
    return 0;
  }
}

/// Wait until a Mutex becomes available
osStatus osMutexWait(
  osMutexId mutex_id,
  uint32_t  millisec
)
{
  return osSemaphoreWait( (osSemaphoreId) mutex_id, millisec );
}

/// Release a Mutex that was obtained with osMutexWait
osStatus osMutexRelease( osMutexId mutex_id )
{
  return osSemaphoreRelease( (osSemaphoreId) mutex_id );
}

/// Delete a Mutex that was created by osMutexCreate
osStatus osMutexDelete( osMutexId mutex_id )
{
  return osSemaphoreDelete( (osSemaphoreId) mutex_id );
}

// Semaphore Public API

/// Create and Initialize a Semaphore object
osSemaphoreId osSemaphoreCreate(
  const osSemaphoreDef_t *semaphore_def,
  int32_t                 count
)
{
  rtems_attribute   sema_attributes;
  rtems_status_code ret;
  rtems_id          semaphore_id;

  if ( count > 1 ) {
    sema_attributes = RTEMS_DEFAULT_ATTRIBUTES;
  } else {
    sema_attributes = RTEMS_SIMPLE_BINARY_SEMAPHORE;
  }

  ret = rtems_semaphore_create(
    semaphore_def->sema_name,
    count,
    sema_attributes,
    RTEMS_NO_PRIORITY,
    &semaphore_id
        );

  if ( ret == RTEMS_SUCCESSFUL ) {
    return (osSemaphoreId) semaphore_id;
  } else {
    return 0;
  }
}

/// Wait until a Semaphore becomes available
int32_t osSemaphoreWait(
  osSemaphoreId semaphore_id,
  uint32_t      millisec
)
{
  rtems_interval  ticks_per_millisec = rtems_clock_get_ticks_per_second() /
                                         1000UL;

  // wait for specified semaphore for the maximum amount of time
  return rtemsConvertReturnCode( rtems_semaphore_obtain( (rtems_id)
      semaphore_id, RTEMS_WAIT, ( ticks_per_millisec * millisec ) ) );
}

/// Release a Semaphore
osStatus osSemaphoreRelease( osSemaphoreId semaphore_id )
{
  return rtemsConvertReturnCode( rtems_semaphore_release( (rtems_id)
      semaphore_id ) );
}

/// Delete a Semaphore that was created by osSemaphoreCreate
osStatus osSemaphoreDelete( osSemaphoreId semaphore_id )
{
  return rtemsConvertReturnCode( rtems_semaphore_delete( (rtems_id)
      semaphore_id ) );
}

// ==== Memory Management Functions ====

// Memory Management Public API

/// Create and Initialize memory pool
osPoolId osPoolCreate( const osPoolDef_t *pool_def )
{
  (void) pool_def;

  // Not implemented yet
  return (osPoolId) 0;
}

/// Allocate a memory block from a memory pool
void *osPoolAlloc( osPoolId pool_id )
{
  (void) pool_id;

  // Not implemented yet
  return NULL;
}

/// Allocate a memory block from a memory pool and set memory block to zero
void *osPoolCAlloc( osPoolId pool_id )
{
  (void) pool_id;

  // Not implemented yet
  return NULL;
}

/// Return an allocated memory block back to a specific memory pool
osStatus osPoolFree(
  osPoolId pool_id,
  void    *block
)
{
  (void) pool_id;
  (void) block;

  // Not implemented yet
  return osErrorOS;
}

// ==== Message Queue Management Functions ====

// Message Queue Management Public API

/// Create and Initialize Message Queue
osMessageQId osMessageCreate(
  const osMessageQDef_t *queue_def,
  osThreadId             thread_id
)
{
  (void) queue_def;
  (void) thread_id;

  // Not implemented yet
  return (osMessageQId) 0;
}

/// Put a Message to a Queue
osStatus osMessagePut(
  osMessageQId queue_id,
  uint32_t     info,
  uint32_t     millisec
)
{
  (void) queue_id;
  (void) info;
  (void) millisec;

  // Not implemented yet
  return osErrorOS;
}

/// Get a Message or Wait for a Message from a Queue
os_InRegs osEvent osMessageGet(
  osMessageQId queue_id,
  uint32_t     millisec
)
{
  (void) queue_id;
  (void) millisec;

  // Not implemented yet
  osEvent ret = { osErrorOS, { 0 }, { 0 } };

  return ret;
}

// ==== Mail Queue Management Functions ====

// Mail Queue Management Public API

/// Create and Initialize mail queue
osMailQId osMailCreate(
  const osMailQDef_t *queue_def,
  osThreadId          thread_id
)
{
  (void) queue_def;
  (void) thread_id;

  // Not implemented yet
  return (osMailQId) 0;
}

/// Allocate a memory block from a mail
void *osMailAlloc(
  osMailQId queue_id,
  uint32_t  millisec
)
{
  (void) queue_id;
  (void) millisec;

  // Not implemented yet
  return NULL;
}

/// Allocate a memory block from a mail and set memory block to zero
void *osMailCAlloc(
  osMailQId queue_id,
  uint32_t  millisec
)
{
  (void) queue_id;
  (void) millisec;

  // Not implemented yet
  return NULL;
}

/// Free a memory block from a mail
osStatus osMailFree(
  osMailQId queue_id,
  void     *mail
)
{
  (void) queue_id;
  (void) mail;

  // Not implemented yet
  return osErrorOS;
}

/// Put a mail to a queue
osStatus osMailPut(
  osMailQId queue_id,
  void     *mail
)
{
  (void) queue_id;
  (void) mail;

  // Not implemented yet
  return osErrorOS;
}

/// Get a mail from a queue
os_InRegs osEvent osMailGet(
  osMailQId queue_id,
  uint32_t  millisec
)
{
  (void) queue_id;
  (void) millisec;

  // Not implemented yet
  osEvent ret = { osErrorOS, { 0 }, { 0 } };

  return ret;
}

