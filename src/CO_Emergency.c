/*
 * CANopen Emergency object.
 *
 * @file        CO_Emergency.c
 * @ingroup     CO_Emergency
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



#include "CO_SDO.h"
#include "CO_Emergency.h"
#include "Logger.h"
/*#include "CANopen.h"*/


/*
 * Function for accessing _Pre-Defined Error Field_ (index 0x1003) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1003(CO_ODF_arg_t *ODF_arg);
static CO_SDO_abortCode_t CO_ODF_1003(CO_ODF_arg_t *ODF_arg){


    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_Emergency.C||"
    					 "Call: CO_ODF_1003"
    					 "\n, msg:start");
    			 logPrint(LOG,logLine);}

    CO_EMpr_t *emPr;
    uint8_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    emPr = (CO_EMpr_t*) ODF_arg->object;
    value = ODF_arg->data[0];


    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_Emergency.C||"
    					 "Call: CO_ODF_1003"
    					 "\n, msg:checks if the SDO state is upload");
    			 logPrint(LOG,logLine);}

    if(ODF_arg->reading){
        uint8_t noOfErrors;
        noOfErrors = emPr->preDefErrNoOfErrors;
        if(LEVEL_1){
           			 sprintf(logLine,"FILE:CO_Emergency.C||"
           					 "Call: CO_ODF_1003"
           					 "\n, msg:checks subindexes in ODF_arg");
           			 logPrint(LOG,logLine);}

        if(ODF_arg->subIndex == 0U){
            ODF_arg->data[0] = noOfErrors;
        }

        else if(ODF_arg->subIndex > noOfErrors){

        	 if(LEVEL_1){
        	    			 sprintf(logLine,"FILE:CO_Emergency.C||"
        	    					 "Call: CO_ODF_1003"
        	    					 "\n, ERROR:NUMBER OF SUBINDES MORE THAN THE NUMBER OF ERRORS");
        	    			 logPrint(ERROR,logLine);}
            ret = CO_SDO_AB_NO_DATA;
        }
        else{

        	 if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_Emergency.C||"
							 "Call: CO_ODF_1003"
							 "\n, ERROR:false subindex information");
					 logPrint(ERROR,logLine);}
            ret = CO_SDO_AB_NONE;
        }
    }
    else{
    	 if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_Emergency.C||"
							 "Call: CO_ODF_1003"
							 "\n, msg:check is subindex is 0");
					 logPrint(LOG,logLine);}
        /* only '0' may be written to subIndex 0 */
        if(ODF_arg->subIndex == 0U){

        	if(LEVEL_1){
        						 sprintf(logLine,"FILE:CO_Emergency.C||"
        								 "Call: CO_ODF_1003"
        								 "\n, msg:check is value 0");
        						 logPrint(LOG,logLine);}
            if(value == 0U){
                emPr->preDefErrNoOfErrors = 0U;
            }
            else{
            	if(LEVEL_1){
            						 sprintf(logLine,"FILE:CO_Emergency.C||"
            								 "Call: CO_ODF_1003"
            								 "\n, ERROR:Invalid value");
            						 logPrint(ERROR,logLine);}
                ret = CO_SDO_AB_INVALID_VALUE;
            }
        }
        else{

        	if(LEVEL_1){
        	            						 sprintf(logLine,"FILE:CO_Emergency.C||"
        	            								 "Call: CO_ODF_1003"
        	            								 "\n, ERROR:Attempt tp write a read only field");
        	            						 logPrint(ERROR,logLine);}
            ret = CO_SDO_AB_READONLY;
        }
    }

    return ret;
}


/*
 * Function for accessing _COB ID EMCY_ (index 0x1014) from SDO server.
 *
 * For more information see file CO_SDO.h.
 */
static CO_SDO_abortCode_t CO_ODF_1014(CO_ODF_arg_t *ODF_arg);
static CO_SDO_abortCode_t CO_ODF_1014(CO_ODF_arg_t *ODF_arg){

	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_Emergency.C||"
						 "Call: CO_ODF_1014"
						 "\n, msg:start");
				 logPrint(LOG,logLine);}
    uint8_t *nodeId;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    nodeId = (uint8_t*) ODF_arg->object;
    value = CO_getUint32(ODF_arg->data);
    if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_Emergency.C||"
								 "Call: CO_ODF_1003"
								 "\n, msg:add node-ID to the value");
						 logPrint(LOG,logLine);}
    /* add nodeId to the value */
    if(ODF_arg->reading){
        CO_setUint32(ODF_arg->data, value + *nodeId);
    }

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_EM_init(
        CO_EM_t                *em,
        CO_EMpr_t              *emPr,
        CO_SDO_t               *SDO,
        uint8_t                *errorStatusBits,
        uint8_t                 errorStatusBitsSize,
        uint8_t                *errorRegister,
        uint32_t               *preDefErr,
        uint8_t                 preDefErrSize,
        CO_CANmodule_t         *CANdev,
        uint16_t                CANdevTxIdx,
        uint16_t                CANidTxEM)
{

	if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_Emergency.C||"
					 "Call: CO_EM_init"
					 "\n, msg:start");
			 logPrint(LOG,logLine);}

    uint8_t i;
	if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_Emergency.C||"
					 "Call: CO_EM_init"
					 "\n, msg:verify function arguments begins");
			 logPrint(LOG,logLine);}
    /* verify arguments */
    if(em==NULL || emPr==NULL || SDO==NULL || errorStatusBits==NULL ||
        errorStatusBitsSize<6U || errorRegister==NULL || preDefErr==NULL || CANdev==NULL){

    	if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_Emergency.C||"
    					 "Call: CO_EM_init"
    					 "\n, ERROR:Illegal arguments to the function");
    			 logPrint(ERROR,logLine);}

        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
	if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_Emergency.C||"
					 "Call: CO_EM_init"
					 "\n, msg:Configure object variables");
			 logPrint(LOG,logLine);}


    /* Configure object variables */
    em->errorStatusBits         = errorStatusBits;
    em->errorStatusBitsSize     = errorStatusBitsSize;
    em->bufEnd                  = em->buf + (CO_EM_INTERNAL_BUFFER_SIZE * 8);
    em->bufWritePtr             = em->buf;
    em->bufReadPtr              = em->buf;
    em->bufFull                 = 0U;
    em->wrongErrorReport        = 0U;
    em->pFunctSignal            = NULL;
    emPr->em                    = em;
    emPr->errorRegister         = errorRegister;
    emPr->preDefErr             = preDefErr;
    emPr->preDefErrSize         = preDefErrSize;
    emPr->preDefErrNoOfErrors   = 0U;
    emPr->inhibitEmTimer        = 0U;

    /* clear error status bits */
    for(i=0U; i<errorStatusBitsSize; i++){
        em->errorStatusBits[i] = 0U;
    }

    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_Emergency.C||"
    					 "Call: CO_EM_init"
    					 "\n, msg:Configure object dictionary entries, call CO_OD_configure");
    			 logPrint(LOG,logLine);}

    /* Configure Object dictionary entry at index 0x1003 and 0x1014 */
    CO_OD_configure(SDO, OD_H1003_PREDEF_ERR_FIELD, CO_ODF_1003, (void*)emPr, 0, 0U);
    CO_OD_configure(SDO, OD_H1014_COBID_EMERGENCY, CO_ODF_1014, (void*)&SDO->nodeId, 0, 0U);


    if(LEVEL_1){
      			 sprintf(logLine,"FILE:CO_Emergency.C||"
      					 "Call: CO_EM_init"
      					 "\n, msg:Cconfigure emergency message CAN transmission, call CO_CANtxBufferInit");
      			 logPrint(LOG,logLine);}

    /* configure emergency message CAN transmission */
    emPr->CANdev = CANdev;
    emPr->CANdev->em = (void*)em; /* update pointer inside CAN device. */
    emPr->CANtxBuff = CO_CANtxBufferInit(
            CANdev,             /* CAN device */
            CANdevTxIdx,        /* index of specific buffer inside CAN module */
            CANidTxEM,          /* CAN identifier */
            0,                  /* rtr */
            8U,                  /* number of data bytes */
            0);                 /* synchronous message flag bit */

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_EM_initCallback(
        CO_EM_t                *em,
        void                  (*pFunctSignal)(void))
{
    if(LEVEL_1){
      			 sprintf(logLine,"FILE:CO_Emergency.C||"
      					 "Call: CO_EM_initCallback"
      					 "\n, msg:start");
      			 logPrint(LOG,logLine);}

    if(em != NULL){
        em->pFunctSignal = pFunctSignal;
    }
}


/******************************************************************************/
void CO_EM_process(
        CO_EMpr_t              *emPr,
        bool_t                  NMTisPreOrOperational,
        uint16_t                timeDifference_100us,
        uint16_t                emInhTime)
{

	 if(LEVEL_1){
	      			 sprintf(logLine,"FILE:CO_Emergency.C||"
	      					 "Call: CO_EM_process"
	      					 "\n, msg:start");
	      			 logPrint(LOG,logLine);}

    CO_EM_t *em = emPr->em;
    uint8_t errorRegister;

    if(LEVEL_1){
   	      			 sprintf(logLine,"FILE:CO_Emergency.C||"
   	      					 "Call: CO_EM_process"
   	      					 "\n, msg:verify errors from driver and other");
   	      			 logPrint(LOG,logLine);}


    /* verify errors from driver and other */
    CO_CANverifyErrors(emPr->CANdev);
    if(em->wrongErrorReport != 0U){
        CO_errorReport(em, CO_EM_WRONG_ERROR_REPORT, CO_EMC_SOFTWARE_INTERNAL, (uint32_t)em->wrongErrorReport);
        em->wrongErrorReport = 0U;
    }


    if(LEVEL_1){
   	      			 sprintf(logLine,"FILE:CO_Emergency.C||"
   	      					 "Call: CO_EM_process"
   	      					 "\n, msg:calculate Error register");
   	      			 logPrint(LOG,logLine);}

    /* calculate Error register */
    errorRegister = 0U;
    /* generic error */
    if(em->errorStatusBits[5]){
        errorRegister |= CO_ERR_REG_GENERIC_ERR;
    }
    /* communication error (overrun, error state) */
    if(em->errorStatusBits[2] || em->errorStatusBits[3]){
        errorRegister |= CO_ERR_REG_COMM_ERR;
    }
    *emPr->errorRegister = (*emPr->errorRegister & 0xEEU) | errorRegister;

    /* inhibit time */
    if(emPr->inhibitEmTimer < emInhTime){
        emPr->inhibitEmTimer += timeDifference_100us;
    }

    /* send Emergency message. */
    if(     NMTisPreOrOperational &&
            !emPr->CANtxBuff->bufferFull &&
            emPr->inhibitEmTimer >= emInhTime &&
            (em->bufReadPtr != em->bufWritePtr || em->bufFull))
    {
        uint32_t preDEF;    /* preDefinedErrorField */
        
        /* add error register */
        em->bufReadPtr[2] = *emPr->errorRegister;

        /* copy data to CAN emergency message */
        CO_memcpy(emPr->CANtxBuff->data, em->bufReadPtr, 8U);
        CO_memcpy((uint8_t*)&preDEF, em->bufReadPtr, 4U);
        em->bufReadPtr += 8;

        /* Update read buffer pointer and reset inhibit timer */
        if(em->bufReadPtr == em->bufEnd){
            em->bufReadPtr = em->buf;
        }
        emPr->inhibitEmTimer = 0U;

        if(LEVEL_1){if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_Emergency.C||"
					 "Call: CO_isError"
					 "\n, msg:start");
			 logPrint(LOG,logLine);}
          	      			 sprintf(logLine,"FILE:CO_Emergency.C||"
          	      					 "Call: CO_EM_process"
          	      					 "\n, msg:verify message buffer overflow, then clear full flag begin");
          	      			 logPrint(LOG,logLine);}


        /* verify message buffer overflow, then clear full flag */
        if(em->bufFull == 2U){
            em->bufFull = 0U;    /* will be updated below */
            CO_errorReport(em, CO_EM_EMERGENCY_BUFFER_FULL, CO_EMC_GENERIC, 0U);
        }
        else{
        	 if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_Emergency.C||"
								 "Call: CO_EM_process"
								 "\n, msg:buffer not full");
						 logPrint(LOG,logLine);}
            em->bufFull = 0;
        }

        if(LEVEL_1){
       						 sprintf(logLine,"FILE:CO_Emergency.C||"
       								 "Call: CO_EM_process"
       								 "\n, msg:begin to write to 'pre-defined error field' (object dictionary, index 0x1003)");
       						 logPrint(LOG,logLine);}

        /* write to 'pre-defined error field' (object dictionary, index 0x1003) */
        if(emPr->preDefErr){
            uint8_t i;

            if(emPr->preDefErrNoOfErrors < emPr->preDefErrSize)
                emPr->preDefErrNoOfErrors++;
            for(i=emPr->preDefErrNoOfErrors-1; i>0; i--)
                emPr->preDefErr[i] = emPr->preDefErr[i-1];
            emPr->preDefErr[0] = preDEF;
        }


        if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_Emergency.C||"
					 "Call: CO_EM_process"
					 "\n, msg:send CAN message, call CO_CANsend");
			 logPrint(LOG,logLine);}
        /* send CAN message */
        CO_CANsend(emPr->CANdev, emPr->CANtxBuff);
    }

    return;
}


/******************************************************************************/
void CO_errorReport(CO_EM_t *em, const uint8_t errorBit, const uint16_t errorCode, const uint32_t infoCode){

	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_Emergency.C||"
						 "Call: CO_errorReport"
						 "\n, msg:start");
				 logPrint(LOG,logLine);}

    uint8_t index = errorBit >> 3;
    uint8_t bitmask = 1 << (errorBit & 0x7);
    uint8_t *errorStatusBits = 0;
    bool_t sendEmergency = true;

	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_Emergency.C||"
						 "Call: CO_errorReport"
						 "\n, msg:check if em is Null");
				 logPrint(LOG,logLine);}

    if(em == NULL){
    	if(LEVEL_1){
    				 sprintf(logLine,"FILE:CO_Emergency.C||"
    						 "Call: CO_errorReport"
    						 "\n, ERROR:Em is null");
    				 logPrint(ERROR,logLine);}
        sendEmergency = false;
    }
	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_Emergency.C||"
						 "Call: CO_errorReport"
						 "\n, msg:check if Index > errorStatusBitSize");
				 logPrint(LOG,logLine);}

    else if(index >= em->errorStatusBitsSize){
        /* if errorBit value not supported, send emergency 'CO_EM_WRONG_ERROR_REPORT' */

    	if(LEVEL_1){
    				 sprintf(logLine,"FILE:CO_Emergency.C||"
    						 "Call: CO_errorReport"
    						 "\n, ERROR:errorbit value not supported, wrong error report");
    				 logPrint(LOG,logLine);}

        em->wrongErrorReport = errorBit;
        sendEmergency = false;
    }
    else{
        errorStatusBits = &em->errorStatusBits[index];
        /* if error was already reported, do nothing */
        if((*errorStatusBits & bitmask) != 0){
            sendEmergency = false;
        }
    }

    if(sendEmergency){
        /* set error bit */
        if(errorBit){
            /* any error except NO_ERROR */
            *errorStatusBits |= bitmask;
        }

        /* verify buffer full, set overflow */
        if(em->bufFull){
            em->bufFull = 2;
        }
        else{
            uint8_t bufCopy[8];

            /* prepare data for emergency message */
            CO_memcpySwap2(&bufCopy[0], &errorCode);
            bufCopy[2] = 0; /* error register will be set later */
            bufCopy[3] = errorBit;
            CO_memcpySwap4(&bufCopy[4], &infoCode);

            /* copy data to the buffer, increment writePtr and verify buffer full */
            CO_LOCK_EMCY();
            CO_memcpy(em->bufWritePtr, &bufCopy[0], 8);
            em->bufWritePtr += 8;

            if(em->bufWritePtr == em->bufEnd) em->bufWritePtr = em->buf;
            if(em->bufWritePtr == em->bufReadPtr) em->bufFull = 1;
            CO_UNLOCK_EMCY();

            /* Optional signal to RTOS, which can resume task, which handles CO_EM_process */
            if(em->pFunctSignal != NULL) {
                em->pFunctSignal();
            }
        }
    }
}


/******************************************************************************/
void CO_errorReset(CO_EM_t *em, const uint8_t errorBit, const uint32_t infoCode){

	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_Emergency.C||"
						 "Call: CO_errorReset"
						 "\n, msg:start");

				 logPrint(LOG,logLine);}

    uint8_t index = errorBit >> 3;
    uint8_t bitmask = 1 << (errorBit & 0x7);
    uint8_t *errorStatusBits = 0;
    bool_t sendEmergency = true;

    if(LEVEL_1){
    				 sprintf(logLine,"FILE:CO_Emergency.C||"
    						 "Call: CO_errorReset"
    						 "\n, msg:check if em is Null");
    				 logPrint(LOG,logLine);}



    if(em == NULL){

    	if(LEVEL_1){
    	    				 sprintf(logLine,"FILE:CO_Emergency.C||"
    	    						 "Call: CO_errorReset"
    	    						 "\n, ERROR:Em is null");
    	    				 logPrint(ERROR,logLine);}
        sendEmergency = false;
    }
    else if(index >= em->errorStatusBitsSize){
        /* if errorBit value not supported, send emergency 'CO_EM_WRONG_ERROR_REPORT' */
        em->wrongErrorReport = errorBit;
        sendEmergency = false;
    }
    else{
        errorStatusBits = &em->errorStatusBits[index];
        /* if error was allready cleared, do nothing */
        if((*errorStatusBits & bitmask) == 0){
            sendEmergency = false;
        }
    }

    if(sendEmergency){
        /* erase error bit */
        *errorStatusBits &= ~bitmask;
        if(LEVEL_1){
            				 sprintf(logLine,"FILE:CO_Emergency.C||"
            						 "Call: CO_errorReport"
            						 "\n, msg:verify if the buffer is full");
            				 logPrint(LOG,logLine);}
        /* verify buffer full */
        if(em->bufFull){
            em->bufFull = 2;
        }
        else{
            uint8_t bufCopy[8];

            /* prepare data for emergency message */
            bufCopy[0] = 0;
            bufCopy[1] = 0;
            bufCopy[2] = 0; /* error register will be set later */
            bufCopy[3] = errorBit;
            CO_memcpySwap4(&bufCopy[4], &infoCode);

            /* copy data to the buffer, increment writePtr and verify buffer full */
            CO_LOCK_EMCY();
            CO_memcpy(em->bufWritePtr, &bufCopy[0], 8);
            em->bufWritePtr += 8;

            if(em->bufWritePtr == em->bufEnd) em->bufWritePtr = em->buf;
            if(em->bufWritePtr == em->bufReadPtr) em->bufFull = 1;
            CO_UNLOCK_EMCY();

            /* Optional signal to RTOS, which can resume task, which handles CO_EM_process */
            if(em->pFunctSignal != NULL) {
                em->pFunctSignal();
            }
        }
    }
}


/******************************************************************************/
bool_t CO_isError(CO_EM_t *em, const uint8_t errorBit){

	if(LEVEL_1){
	    				 sprintf(logLine,"FILE:CO_Emergency.C||"
	    						 "Call: CO_isError"
	    						 "\n, msg:start");
	    				 logPrint(LOG,logLine);}
    uint8_t index = errorBit >> 3;
    uint8_t bitmask = 1 << (errorBit & 0x7);
    bool_t ret = false;


    if(LEVEL_1){
    	    				 sprintf(logLine,"FILE:CO_Emergency.C||"
    	    						 "Call: CO_isError"
    	    						 "\n, msg:check if em is not null and if index < than the errorStatusBitsSize");
    	    				 logPrint(LOG,logLine);}


    if(em != NULL && index < em->errorStatusBitsSize){
        if((em->errorStatusBits[index] & bitmask) != 0){
            ret = true;
        }
    }

    return ret;
}
