
#ifndef KARSH_H
#define KARSH_H

/* For documentation see file drvTemplate/CO_driver.h
 *
 *
 * */

/**From documentation
 *
 *
 * @defgroup CO_driver Driver
 * @ingroup CO_CANopen
 * @{
 *
 * Microcontroller specific code for CANopenNode.
 *
 * This file contains type definitions, functions and macros for:
 *  - Basic data types.
 *  - Receive and transmit buffers for CANopen messages.
 *  - Interaction with CAN module on the microcontroller.
 *  - CAN receive and transmit interrupts.
 *
 * This file is not only a CAN driver. There are no classic CAN queues for CAN
 * messages. This file provides direct connection with other CANopen
 * objects. It tries to provide fast responses and tries to avoid unnecessary
 * calculations and memory consumptions.
 *
 * CO_CANmodule_t contains an array of _Received message objects_ (of type
 * CO_CANrx_t) and an array of _Transmit message objects_ (of type CO_CANtx_t).
 * Each CANopen communication object owns one member in one of the arrays.
 * For example Heartbeat producer generates one CANopen transmitting object,
 * so it has reserved one member in CO_CANtx_t array.
 * SYNC module may produce sync or consume sync, so it has reserved one member
 * in CO_CANtx_t and one member in CO_CANrx_t array.
 *
 * ###Reception of CAN messages.
 * Before CAN messages can be received, each member in CO_CANrx_t must be
 * initialized. CO_CANrxBufferInit() is called by CANopen module, which
 * uses specific member. For example @ref CO_HBconsumer uses multiple members
 * in CO_CANrx_t array. (It monitors multiple heartbeat messages from remote
 * nodes.) It must call CO_CANrxBufferInit() multiple times.
 *
 * Main arguments to the CO_CANrxBufferInit() function are CAN identifier
 * and a pointer to callback function. Those two arguments (and some others)
 * are copied to the member of the CO_CANrx_t array.
 *
 * Callback function is a function, specified by specific CANopen module
 * (for example by @ref CO_HBconsumer). Each CANopen module defines own
 * callback function. Callback function will process the received CAN message.
 * It will copy the necessary data from CAN message to proper place. It may
 * also trigger additional task, which will further process the received message.
 * Callback function must be fast and must only make the necessary calculations
 * and copying.
 *
 * Received CAN messages are processed by CAN receive interrupt function.
 * After CAN message is received, function first tries to find matching CAN
 * identifier from CO_CANrx_t array. If found, then a corresponding callback
 * function is called.
 *
 * Callback function accepts two parameters:
 *  - object is pointer to object registered by CO_CANrxBufferInit().
 *  - msg  is pointer to CAN message of type CO_CANrxMsg_t.
 *
 * Callback function must return #CO_ReturnError_t: CO_ERROR_NO,
 * CO_ERROR_RX_OVERFLOW, CO_ERROR_RX_PDO_OVERFLOW, CO_ERROR_RX_MSG_LENGTH or
 * CO_ERROR_RX_PDO_LENGTH.
 *
 *
 * ###Transmission of CAN messages.
 * Before CAN messages can be transmitted, each member in CO_CANtx_t must be
 * initialized. CO_CANtxBufferInit() is called by CANopen module, which
 * uses specific member. For example Heartbeat producer must initialize it's
 * member in CO_CANtx_t array.
 *
 * CO_CANtxBufferInit() returns a pointer of type CO_CANtx_t, which contains buffer
 * where CAN message data can be written. CAN message is send with calling
 * CO_CANsend() function. If at that moment CAN transmit buffer inside
 * microcontroller's CAN module is free, message is copied directly to CAN module.
 * Otherwise CO_CANsend() function sets _bufferFull_ flag to true. Message will be
 * then sent by CAN TX interrupt as soon as CAN module is freed. Until message is
 * not copied to CAN module, its contents must not change. There may be multiple
 * _bufferFull_ flags in CO_CANtx_t array set to true. In that case messages with
 * lower index inside array will be sent first.
 */


/**
 * @name Critical sections
 * CANopenNode is designed to run in different threads, as described in README.
 * Threads are implemented differently in different systems. In microcontrollers
 * threads are interrupts with different priorities, for example.
 * It is necessary to protect sections, where different threads access to the
 * same resource. In simple systems interrupts or scheduler may be temporary
 * disabled between access to the shared resource. Otherwise mutexes or
 * semaphores can be used.
 *
 * ####Reentrant functions.
 * Functions CO_CANsend() from C_driver.h, CO_errorReport() from CO_Emergency.h
 * and CO_errorReset() from CO_Emergency.h may be called from different threads.
 * Critical sections must be protected. Eather by disabling scheduler or
 * interrupts or by mutexes or semaphores.
 *
 * ####Object Dictionary variables.
 * In general, there are two threads, which accesses OD variables: mainline and
 * timer. CANopenNode initialization and SDO server runs in mainline. PDOs runs
 * in faster timer thread. Processing of PDOs must not be interrupted by
 * mainline. Mainline thread must protect sections, which accesses the same OD
 * variables as timer thread. This care must also take the application. Note
 * that not all variables are allowed to be mapped to PDOs, so they may not need
 * to be protected. SDO server protects sections with access to OD variables.
 *
 * ####CAN receive thread.
 * It partially processes received CAN data and puts them into appropriate
 * objects. Objects are later processed. It does not need protection of
 * critical sections. There is one circumstance, where CANrx should be disabled:
 * After presence of SYNC message on CANopen bus, CANrx should be temporary
 * disabled until all receive PDOs are processed. See also CO_SYNC.h file and
 * CO_SYNC_initCallback() function.
 * @{
 */

#include <stdc-predef.h>         /* for 'NULL' */
#include <stdint.h>         /* for 'int8_t' to 'uint64_t' */
#include "stdbool.h"        /* for 'true', 'false' */
#include <unistd.h>
#include <endian.h>
#include "Logger.h"

#ifndef CO_SINGLE_THREAD
#include <pthread.h>
#endif
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
/*#include "CO_Linux_tasks.h"*/

/* general configuration */
 //   #define CO_LOG_CAN_MESSAGES   /* Call external function for each received or transmitted CAN message. */
#define CO_SDO_BUFFER_SIZE   889    /* Override default SDO buffer size. */



/* Critical sections */


#ifdef CO_SINGLE_THREAD

    #define CO_LOCK_CAN_SEND()
    #define CO_UNLOCK_CAN_SEND()

    #define CO_LOCK_EMCY()
    #define CO_UNLOCK_EMCY()

    #define CO_LOCK_OD()
    #define CO_UNLOCK_OD()

#else


    #define CO_LOCK_CAN_SEND()      /* not needed */
    #define CO_UNLOCK_CAN_SEND()

    extern pthread_mutex_t CO_EMCY_mtx;
    #define CO_LOCK_EMCY()          {if(pthread_mutex_lock(&CO_EMCY_mtx) != 0) CO_errExit("Mutex lock CO_EMCY_mtx failed");}
    #define CO_UNLOCK_EMCY()        {if(pthread_mutex_unlock(&CO_EMCY_mtx) != 0) CO_errExit("Mutex unlock CO_EMCY_mtx failed");}

    extern pthread_mutex_t CO_OD_mtx;
    #define CO_LOCK_OD()            {if(pthread_mutex_lock(&CO_OD_mtx) != 0) CO_errExit("Mutex lock CO_OD_mtx failed");}
    #define CO_UNLOCK_OD()          {if(pthread_mutex_unlock(&CO_OD_mtx) != 0) CO_errExit("Mutex unlock CO_OD_mtx failed");}

 #endif


    /* Data types */
    /* int8_t to uint64_t are defined in stdint.h */
    typedef _Bool                   bool_t;
    typedef float                   float32_t;
    typedef double                  float64_t;
    typedef char                    char_t;
    typedef unsigned char           oChar_t;
    typedef unsigned char           domain_t;


/* Return values */
typedef enum{
    CO_ERROR_NO                 = 0,
    CO_ERROR_ILLEGAL_ARGUMENT   = -1,
    CO_ERROR_OUT_OF_MEMORY      = -2,
    CO_ERROR_TIMEOUT            = -3,
    CO_ERROR_ILLEGAL_BAUDRATE   = -4,
    CO_ERROR_RX_OVERFLOW        = -5,
    CO_ERROR_RX_PDO_OVERFLOW    = -6,
    CO_ERROR_RX_MSG_LENGTH      = -7,
    CO_ERROR_RX_PDO_LENGTH      = -8,
    CO_ERROR_TX_OVERFLOW        = -9,
    CO_ERROR_TX_PDO_WINDOW      = -10,
    CO_ERROR_TX_UNCONFIGURED    = -11,
    CO_ERROR_PARAMETERS         = -12,
    CO_ERROR_DATA_CORRUPT       = -13,
    CO_ERROR_CRC                = -14
}CO_ReturnError_t;


/* CAN receive message structure as aligned in CAN module. */
typedef struct{
    uint32_t        ident;
    uint8_t         DLC;
    uint8_t         data[8] __attribute__((aligned(8)));
}CO_CANrxMsg_t;


/* Received message object */
typedef struct{
    uint32_t            ident;
    uint32_t            mask;
    void               *object;
    void              (*pFunct)(void *object, const CO_CANrxMsg_t *message);
}CO_CANrx_t;



 //karthik
// Transmit message object as aligned in CAN module.
typedef struct{
    uint32_t            ident;
    uint8_t             DLC;
    uint8_t             data[8] __attribute__((aligned(8)));
    volatile bool_t     bufferFull;
    volatile bool_t     syncFlag;
}CO_CANtx_t;



/*
 * karthik
 *
 CAN module object.
 */
typedef struct{
    int32_t             CANbaseAddress;
#ifdef CO_LOG_CAN_MESSAGES
    CO_CANtx_t          txRecord;
#endif
    CO_CANrx_t         *rxArray;
    uint16_t            rxSize;
    CO_CANtx_t         *txArray;
    uint16_t            txSize;
    uint16_t            wasConfigured; //Zero only on first run of CO_CANmodule_init
    int                 fd;          //CAN_RAW socket file descriptor
    struct can_filter  *filter;      //array of CAN filters of size rxSize
    volatile bool_t     CANnormal;
    volatile bool_t     useCANrxFilters;
    volatile bool_t     bufferInhibitFlag;
    volatile bool_t     firstCANtxMessage;
    volatile uint8_t    error;
    volatile uint16_t   CANtxCount;
    uint32_t            errOld;
    void               *em;
}CO_CANmodule_t;



/* Endianes */
#ifdef BYTE_ORDER
#if BYTE_ORDER == LITTLE_ENDIAN
    #define CO_LITTLE_ENDIAN
#else
    #define CO_BIG_ENDIAN
#endif
#endif


/* Helper function, must be defined externally. */
void CO_errExit(char* msg);


/* Request CAN configuration or normal mode */
void CO_CANsetConfigurationMode(int32_t fdSocket);
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule);


/* Initialize CAN module object. */
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate); /* not used */


/* Switch off CANmodule. */
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule);


/* Read CAN identifier */
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg);


/* Configure CAN message receive buffer. */
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message));


/* Configure CAN message transmit buffer. */
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag);


/* Send CAN message. */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);


/* Clear all synchronous TPDOs from CAN module transmit buffers. */
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule);


/* Verify all errors of CAN module. */
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule);


/* Functions receives CAN messages. It is blocking.
 *
 * @param CANmodule This object.
 */
void CO_CANrxWait(CO_CANmodule_t *CANmodule);

#endif
