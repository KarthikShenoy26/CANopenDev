/*
 * CANopen SYNC object.
 *
 * @file        CO_SYNC.c
 * @ingroup     CO_SYNC
 * @author      Janez Paternoster
 * @copyright   2004 - 2013 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */


#include "CO_driver.h"
#include "CO_SDO.h"
#include "CO_Emergency.h"
#include "CO_NMT_Heartbeat.h"
#include "CO_SYNC.h"
#include "Logger.h"
/*
 * Read received message from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN
 * message with correct identifier will be received. For more information and
 * description of parameters see file CO_driver.h.
 */
static void CO_SYNC_receive(void *object, const CO_CANrxMsg_t *msg){



    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_SYNC.C||"
    					 "Call: CO_SYNC_receive"
    					 "\n, msg:start");
    			 logPrint(LOG,logLine);}


    CO_SYNC_t *SYNC;
    uint8_t operState;

    SYNC = (CO_SYNC_t*)object;   /* this is the correct pointer type of the first argument */
    operState = *SYNC->operatingState;


    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_SYNC.C||"
    					 "Call: CO_SYNC_receive"
    					 "\n, msg:begin check for operating state");
    			 logPrint(LOG,logLine);}


    if((operState == CO_NMT_OPERATIONAL) || (operState == CO_NMT_PRE_OPERATIONAL)){


        if(LEVEL_1){
        			 sprintf(logLine,"FILE:CO_SYNC.C||"
        					 "Call: CO_SYNC_receive"
        					 "\n, msg:check for the counterOverflowValue");
        			 logPrint(LOG,logLine);}

        if(SYNC->counterOverflowValue == 0){
        	if(LEVEL_1){
        	 sprintf(logLine,"FILE:CO_SYNC.C||"
						 "Call: CO_SYNC_receive"
						 "\n, msg:check for msg->DLC=0");
				 logPrint(LOG,logLine);}

            if(msg->DLC == 0U){
                SYNC->CANrxNew = true;
            }
            else{
                SYNC->receiveError = (uint16_t)msg->DLC | 0x0100U;
            }
        }
        if(LEVEL_1){
	 sprintf(logLine,"FILE:CO_SYNC.C||"
					 "Call: CO_SYNC_receive"
					 "\n, msg:when NMT state is different than OPE an PRE_OPE");
			 logPrint(LOG,logLine);}
        else{
            if(msg->DLC == 1U){
                SYNC->counter = msg->data[0];
                SYNC->CANrxNew = true;
            }
            else{
                SYNC->receiveError = (uint16_t)msg->DLC | 0x0200U;
            }
        }
        if(SYNC->CANrxNew) {
            SYNC->CANrxToggle = SYNC->CANrxToggle ? false : true;
        }
    }
}


/*
 * Function for accessing _COB ID SYNC Message_ (index 0x1005) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1005(CO_ODF_arg_t *ODF_arg){

	if(LEVEL_1){
		 sprintf(logLine,"FILE:CO_SYNC.C||"
						 "Call: CO_ODF_1005"
						 "\n, msg:start");
				 logPrint(LOG,logLine);}

    CO_SYNC_t *SYNC;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(LEVEL_1){
    		 sprintf(logLine,"FILE:CO_SYNC.C||"
    						 "Call: CO_ODF_1005"
    						 "\n, msg:check if SDO mode is UPLOAD");
    				 logPrint(LOG,logLine);}


    if(!ODF_arg->reading){
        uint8_t configureSyncProducer = 0;

        /* only 11-bit CAN identifier is supported */
        if(LEVEL_1){
        		 sprintf(logLine,"FILE:CO_SYNC.C||"
        						 "Call: CO_ODF_1005"
        						 "\n, msg:check CAN frame type");
        				 logPrint(LOG,logLine);}
        if(value & 0x20000000UL){

        	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_SYNC.C||"
								 "Call: CO_ODF_1005"
								 "\n, ERROR:Wrong frame format");
						 logPrint(ERROR,logLine);}


            ret = CO_SDO_AB_INVALID_VALUE;
        }
        else{

        	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_SYNC.C||"
								 "Call: CO_ODF_1005"
								 "\n, msg:check generate Sync messge' bit set");
						 logPrint(LOG,logLine);}
            /* is 'generate Sync messge' bit set? */
            if(value & 0x40000000UL){
                /* if bit was set before, value can not be changed */
            	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_SYNC.C||"
										 "Call: CO_ODF_1005"
										 "\n, msg:check if Sync is a producer");
								 logPrint(LOG,logLine);}

                if(SYNC->isProducer){

                	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_SYNC.C||"
										 "Call: CO_ODF_1005"
										 "\n, ERROR:Value cannot be changed as sync is a producer");
								 logPrint(ERROR,logLine);}
                    ret = CO_SDO_AB_DATA_DEV_STATE;
                }
                else{
                	if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_SYNC.C||"
									 "Call: CO_ODF_1005"
									 "\n, ERROR:Sync bit is not set");
							 logPrint(ERROR,logLine);}

                    configureSyncProducer = 1;
                }
            }
        }
        if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_SYNC.C||"
							 "Call: CO_ODF_1005"
							 "\n, msg:configure sync producer and consumer");
					 logPrint(LOG,logLine);}


        /* configure sync producer and consumer */
        if(ret == CO_SDO_AB_NONE){
            SYNC->COB_ID = (uint16_t)(value & 0x7FFU);

            if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_SYNC.C||"
							 "Call: CO_ODF_1005"
							 "\n, msg:check if configureSyncProducer is true or not, if true then sync is producer if false otherwise");
					 logPrint(LOG,logLine);}

            if(configureSyncProducer){
                uint8_t len = 0U;
                if(SYNC->counterOverflowValue != 0U){
                    len = 1U;
                    SYNC->counter = 0U;
                    SYNC->timer = 0U;
                }
                SYNC->CANtxBuff = CO_CANtxBufferInit(
                        SYNC->CANdevTx,         /* CAN device */
                        SYNC->CANdevTxIdx,      /* index of specific buffer inside CAN module */
                        SYNC->COB_ID,           /* CAN identifier */
                        0,                      /* rtr */
                        len,                    /* number of data bytes */
                        0);                     /* synchronous message flag bit */
                SYNC->isProducer = true;
            }
            else{
                SYNC->isProducer = false;
            }

            CO_CANrxBufferInit(
                    SYNC->CANdevRx,         /* CAN device */
                    SYNC->CANdevRxIdx,      /* rx buffer index */
                    SYNC->COB_ID,           /* CAN identifier */
                    0x7FF,                  /* mask */
                    0,                      /* rtr */
                    (void*)SYNC,            /* object passed to receive function */
                    CO_SYNC_receive);       /* this function will process received message */
        }
    }

    return ret;
}


/*
 * Function for accessing _Communication cycle period_ (index 0x1006) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1006(CO_ODF_arg_t *ODF_arg){

    if(LEVEL_1){
		sprintf(logLine,"FILE:CO_SYNC.C||"
					 "Call: CO_ODF_1006"
					 "\n, msg:start");
			 logPrint(LOG,logLine);}

    CO_SYNC_t *SYNC;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading){
        /* period transition from 0 to something */
        if((SYNC->periodTime == 0) && (value != 0)){
            SYNC->counter = 0;
        }


        if(LEVEL_1){
    		sprintf(logLine,"FILE:CO_SYNC.C||"
    					 "Call: CO_ODF_1006"
    					 "\n, msg:check for overflow of the value");
    			 logPrint(LOG,logLine);}

        SYNC->periodTime = value;
        SYNC->periodTimeoutTime = (value / 2U) * 3U;
        /* overflow? */
        if(SYNC->periodTimeoutTime < value){
            SYNC->periodTimeoutTime = 0xFFFFFFFFUL;
        }

        SYNC->timer = 0;
    }

    return ret;
}


/**
 * Function for accessing _Synchronous counter overflow value_ (index 0x1019) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1019(CO_ODF_arg_t *ODF_arg){

    if(LEVEL_1){
		sprintf(logLine,"FILE:CO_SYNC.C||"
					 "Call: CO_ODF_1019"
					 "\n, msg:start");
			 logPrint(LOG,logLine);}

    CO_SYNC_t *SYNC;
    uint8_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    SYNC = (CO_SYNC_t*) ODF_arg->object;
    value = ODF_arg->data[0];

    if(LEVEL_1){
    		sprintf(logLine,"FILE:CO_SYNC.C||"
    					 "Call: CO_ODF_1019"
    					 "\n, msg:check if the SDO state is download");
    		logPrint(LOG,logLine);}
    if(!ODF_arg->reading){
        uint8_t len = 0U;

        if(LEVEL_1){
           		sprintf(logLine,"FILE:CO_SYNC.C||"
           					 "Call: CO_ODF_1019"
           					 "\n, msg:check if the period time is not zero");
           		logPrint(LOG,logLine);}

        if(SYNC->periodTime){

            if(LEVEL_1){
               		sprintf(logLine,"FILE:CO_SYNC.C||"
               					 "Call: CO_ODF_1019"
               					 "\n, ERROR:period time is zero");
               		logPrint(ERROR,logLine);}
            ret = CO_SDO_AB_DATA_DEV_STATE;
        }
        else{
            SYNC->counterOverflowValue = value;
            if(value != 0){
                len = 1U;
            }
            if(LEVEL_1){
                     		sprintf(logLine,"FILE:CO_SYNC.C||"
                     					 "Call: CO_ODF_1019"
                     					 "\n, msg:call CO_CANtxBufferInit for initializing the CANtx buffer");
                     		logPrint(LOG,logLine);}

            SYNC->CANtxBuff = CO_CANtxBufferInit(
                    SYNC->CANdevTx,         /* CAN device */
                    SYNC->CANdevTxIdx,      /* index of specific buffer inside CAN module */
                    SYNC->COB_ID,           /* CAN identifier */
                    0,                      /* rtr */
                    len,                    /* number of data bytes */
                    0);                     /* synchronous message flag bit */
        }
    }

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_SYNC_init(
        CO_SYNC_t              *SYNC,
        CO_EM_t                *em,
        CO_SDO_t               *SDO,
        uint8_t                *operatingState,
        uint32_t                COB_ID_SYNCMessage,
        uint32_t                communicationCyclePeriod,
        uint8_t                 synchronousCounterOverflowValue,
        CO_CANmodule_t         *CANdevRx,
        uint16_t                CANdevRxIdx,
        CO_CANmodule_t         *CANdevTx,
        uint16_t                CANdevTxIdx)
{

    if(LEVEL_1){
			sprintf(logLine,"FILE:CO_SYNC.C||"
						 "Call: CO_SYNC_init"
						 "\n, msg:start");
			logPrint(LOG,logLine);}
    uint8_t len = 0;

    if(LEVEL_1){
    			sprintf(logLine,"FILE:CO_SYNC.C||"
    						 "Call: CO_SYNC_init"
    						 "\n, msg:Verify arguments");
    			logPrint(LOG,logLine);}

    /* verify arguments */
    if(SYNC==NULL || em==NULL || SDO==NULL || operatingState==NULL ||
        CANdevRx==NULL || CANdevTx==NULL){
    	 if(LEVEL_1){
    	    			sprintf(logLine,"FILE:CO_SYNC.C||"
    	    						 "Call: CO_SYNC_init"
    	    						 "\n, ERROR:Illegal argument");
    	    			logPrint(ERROR,logLine);}

        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    if(LEVEL_1){
       			sprintf(logLine,"FILE:CO_SYNC.C||"
       						 "Call: CO_SYNC_init"
       						 "\n, msg:begin configuring object variables");
       			logPrint(LOG,logLine);}


    /* Configure object variables */
    SYNC->isProducer = (COB_ID_SYNCMessage&0x40000000L) ? true : false;
    SYNC->COB_ID = COB_ID_SYNCMessage&0x7FF;

    SYNC->periodTime = communicationCyclePeriod;
    SYNC->periodTimeoutTime = communicationCyclePeriod / 2 * 3;
    /* overflow? */
    if(SYNC->periodTimeoutTime < communicationCyclePeriod) SYNC->periodTimeoutTime = 0xFFFFFFFFL;

    SYNC->counterOverflowValue = synchronousCounterOverflowValue;
    if(synchronousCounterOverflowValue) len = 1;

    SYNC->curentSyncTimeIsInsideWindow = true;

    SYNC->CANrxNew = false;
    SYNC->CANrxToggle = false;
    SYNC->timer = 0;
    SYNC->counter = 0;
    SYNC->receiveError = 0U;

    SYNC->em = em;
    SYNC->operatingState = operatingState;

    SYNC->CANdevRx = CANdevRx;
    SYNC->CANdevRxIdx = CANdevRxIdx;


    if(LEVEL_1){
       			sprintf(logLine,"FILE:CO_SYNC.C||"
       						 "Call: CO_SYNC_init"
       						 "\n, msg:begin configuring object dictionary entries, call CO_OD_configure ");
       			logPrint(LOG,logLine);}

    /* Configure Object dictionary entry at index 0x1005, 0x1006 and 0x1019 */
    CO_OD_configure(SDO, OD_H1005_COBID_SYNC,        CO_ODF_1005, (void*)SYNC, 0, 0);
    CO_OD_configure(SDO, OD_H1006_COMM_CYCL_PERIOD,  CO_ODF_1006, (void*)SYNC, 0, 0);
    CO_OD_configure(SDO, OD_H1019_SYNC_CNT_OVERFLOW, CO_ODF_1019, (void*)SYNC, 0, 0);


    if(LEVEL_1){
          			sprintf(logLine,"FILE:CO_SYNC.C||"
          						 "Call: CO_SYNC_init"
          						 "\n, msg:begin configuring onfigure SYNC CAN reception, call CO_CANrxBufferInit");
          			logPrint(LOG,logLine);}



    /* configure SYNC CAN reception */
    CO_CANrxBufferInit(
            CANdevRx,               /* CAN device */
            CANdevRxIdx,            /* rx buffer index */
            SYNC->COB_ID,           /* CAN identifier */
            0x7FF,                  /* mask */
            0,                      /* rtr */
            (void*)SYNC,            /* object passed to receive function */
            CO_SYNC_receive);       /* this function will process received message */

    if(LEVEL_1){
          			sprintf(logLine,"FILE:CO_SYNC.C||"
          						 "Call: CO_SYNC_init"
          						 "\n, msg:begin configure SYNC CAN transmission");
          			logPrint(LOG,logLine);}



    /* configure SYNC CAN transmission */
    SYNC->CANdevTx = CANdevTx;
    SYNC->CANdevTxIdx = CANdevTxIdx;
    SYNC->CANtxBuff = CO_CANtxBufferInit(
            CANdevTx,               /* CAN device */
            CANdevTxIdx,            /* index of specific buffer inside CAN module */
            SYNC->COB_ID,           /* CAN identifier */
            0,                      /* rtr */
            len,                    /* number of data bytes */
            0);                     /* synchronous message flag bit */

    return CO_ERROR_NO;
}


/******************************************************************************/
uint8_t CO_SYNC_process(
        CO_SYNC_t              *SYNC,
        uint32_t                timeDifference_us,
        uint32_t                ObjDict_synchronousWindowLength)
{
	  if(LEVEL_1){
	          			sprintf(logLine,"FILE:CO_SYNC.C||"
	          						 "Call: CO_SYNC_process"
	          						 "\n, msg:start");
	          			logPrint(LOG,logLine);}



    uint8_t ret = 0;
    uint32_t timerNew;
    if(LEVEL_1){
   	          			sprintf(logLine,"FILE:CO_SYNC.C||"
   	          						 "Call: CO_SYNC_process"
   	          						 "\n, msg:check the operating state");
   	          			logPrint(LOG,logLine);}


    if(*SYNC->operatingState == CO_NMT_OPERATIONAL || *SYNC->operatingState == CO_NMT_PRE_OPERATIONAL){
        /* update sync timer, no overflow */
        timerNew = SYNC->timer + timeDifference_us;
        if(timerNew > SYNC->timer) SYNC->timer = timerNew;

        if(LEVEL_1){
          	          			sprintf(logLine,"FILE:CO_SYNC.C||"
          	          						 "Call: CO_SYNC_process"
          	          						 "\n, msg:check if the SYNC was just received");
          	          			logPrint(LOG,logLine);}

        /* was SYNC just received */
        if(SYNC->CANrxNew){
            SYNC->timer = 0;
            ret = 1;
            SYNC->CANrxNew = false;
        }


        if(LEVEL_1){
          	          			sprintf(logLine,"FILE:CO_SYNC.C||"
          	          						 "Call: CO_SYNC_process"
          	          						 "\n, msg: sync  producer begin");
          	          			logPrint(LOG,logLine);}
        /* SYNC producer */
        if(SYNC->isProducer && SYNC->periodTime){
            if(SYNC->timer >= SYNC->periodTime){
                if(++SYNC->counter > SYNC->counterOverflowValue) SYNC->counter = 1;
                SYNC->timer = 0;
                ret = 1;
                SYNC->CANrxToggle = SYNC->CANrxToggle ? false : true;
                SYNC->CANtxBuff->data[0] = SYNC->counter;
                CO_CANsend(SYNC->CANdevTx, SYNC->CANtxBuff);
            }
        }

        /* Synchronous PDOs are allowed only inside time window */
        if(ObjDict_synchronousWindowLength){


            if(LEVEL_1){
              	          			sprintf(logLine,"FILE:CO_SYNC.C||"
              	          						 "Call: CO_SYNC_process"
              	          						 "\n, msg: check if SYNC->timer > ObjDict_synchronousWindowLength");
              	          			logPrint(LOG,logLine);}
            if(SYNC->timer > ObjDict_synchronousWindowLength){
                if(SYNC->curentSyncTimeIsInsideWindow){
                    ret = 2;
                }
                SYNC->curentSyncTimeIsInsideWindow = false;
            }
            else{
                SYNC->curentSyncTimeIsInsideWindow = true;
            }
        }
        else{
            SYNC->curentSyncTimeIsInsideWindow = true;
        }
        if(LEVEL_1){
				sprintf(logLine,"FILE:CO_SYNC.C||"
							 "Call: CO_SYNC_process"
							 "\n, msg: begin Verify timeout of SYNC");
				logPrint(LOG,logLine);}
        /* Verify timeout of SYNC */
        if(SYNC->periodTime && SYNC->timer > SYNC->periodTimeoutTime && *SYNC->operatingState == CO_NMT_OPERATIONAL)
            CO_errorReport(SYNC->em, CO_EM_SYNC_TIME_OUT, CO_EMC_COMMUNICATION, SYNC->timer);
    }
    else {
        SYNC->CANrxNew = false;
    }
    if(LEVEL_1){
				sprintf(logLine,"FILE:CO_SYNC.C||"
							 "Call: CO_SYNC_process"
							 "\n, msg: begin verify error from receive function");
				logPrint(LOG,logLine);}
    /* verify error from receive function */
    if(SYNC->receiveError != 0U){
        CO_errorReport(SYNC->em, CO_EM_SYNC_LENGTH, CO_EMC_SYNC_DATA_LENGTH, (uint32_t)SYNC->receiveError);
        SYNC->receiveError = 0U;
    }

    return ret;
}
