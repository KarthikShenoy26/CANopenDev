/*
 * CAN module object for Linux SocketCAN.
 *
 * @file        CO_driver.c
 * @author      Janez Paternoster
 * @copyright   2015 Janez Paternoster
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
 */


#include "CO_driver.h"
#include "CO_Emergency.h"
#include <string.h> /* for memcpy */
#include <stdlib.h> /* for malloc, free */
#include <errno.h>
#include <sys/socket.h>


/******************************************************************************/
#ifndef CO_SINGLE_THREAD
    pthread_mutex_t CO_EMCY_mtx = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t CO_OD_mtx = PTHREAD_MUTEX_INITIALIZER;
#endif


/** Set socketCAN filters *****************************************************/
    /*
     * This function is used to set socketCAN filters.
     *In the canModule we have #filter pointer pointing to the array of can_filter types.
      if we set useCANrxFilter means it will use socketCAN filter. if Thus this function checks for filter in
      filter array of canModule with nonzero canid and tries to set socketCAN filter for the mentioned settings.
     */
static CO_ReturnError_t setFilters(CO_CANmodule_t *CANmodule){
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: setFilters"
			"\nMSG: started"); logPrint(LOG,logLine);}
    CO_ReturnError_t ret = CO_ERROR_NO;

    if(CANmodule->useCANrxFilters)
    {
    	 if(LEVEL_1){sprintf(logLine,
    			 "FILE: CO_driver.c"
    			 "||CALL: setFilters"
    			 "\nMSG:Use of receive filter is enabled"); logPrint(LOG,logLine);}

        int nFiltersIn, nFiltersOut;
        struct can_filter *filtersOut;

        nFiltersIn = CANmodule->rxSize;
        nFiltersOut = 0;

        if(LEVEL_1){sprintf(logLine,
            		 "FILE: CO_driver.c"
            		 "||CALL: setFilters"
            		 "\nMSG: Allocating memory to filter"); logPrint(LOG,logLine);}

        filtersOut = (struct can_filter *) calloc(nFiltersIn, sizeof(struct can_filter));

        if(filtersOut == NULL){
        	 if(LEVEL_1){sprintf(logLine,
        			 "FILE: CO_driver.c"
        			 "||CALL: setFilters"
        			 "\nMSG: Allocating memory to filter failed. out of memory"); logPrint(ERROR,logLine);}
            ret = CO_ERROR_OUT_OF_MEMORY;
        }else{
            int i;
            int idZeroCnt = 0;

            /* Copy filterIn to filtersOut. Accept only first filter with
             * can_id=0, omit others. */
            for(i=0; i<nFiltersIn; i++)
            {
                struct can_filter *fin;

                fin = &CANmodule->filter[i];

                /*incrementing idZeroCnt makes sure atleast element with canid=0
                  in the array gets configured.
                 */
                if(fin->can_id == 0)
                {
                    idZeroCnt++;
                }
                if(fin->can_id != 0 || idZeroCnt == 1)
                {
                    struct can_filter *fout;

                    fout = &filtersOut[nFiltersOut++];
                    fout->can_id = fin->can_id;
                    fout->can_mask = fin->can_mask;
                }
            }

            if(LEVEL_1){sprintf(logLine,
            		"FILE: CO_driver.c"
            		"||CALL: setFilters"
            		"\nMSG: Setting socket option.For filter settings"); logPrint(LOG,logLine);}

            //setting filter configuration in socketCAN.
            if(setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                          filtersOut, sizeof(struct can_filter) * nFiltersOut) != 0)
            {
                if(LEVEL_1){sprintf(logLine,
                		"FILE: CO_driver.c"
                		"||CALL: setFilters"
                		"\nMSG: Setting socket option failed. Illegal argument "); logPrint(ERROR,logLine);}

                ret = CO_ERROR_ILLEGAL_ARGUMENT;
            }

            free(filtersOut);
        }
    }else{
    	if(LEVEL_1){sprintf(logLine,
    			"FILE: CO_driver.c"
    			"||CALL: setFilters"
    			"\nMSG:Use of receive filter is disabled"); logPrint(LOG,logLine);}
        /* Use one socketCAN filter, match any CAN address, including extended and rtr. */
        CANmodule->filter[0].can_id = 0;
        CANmodule->filter[0].can_mask = 0;

        if(LEVEL_1){sprintf(logLine,
        		"FILE: CO_driver.c"
        		"||CALL: setFilters"
        		"\nMSG: Setting socket option. No filtering. All can address, "
        		"extended, rtr allowed"); logPrint(LOG,logLine);}

        if(setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER,
            &CANmodule->filter[0], sizeof(struct can_filter)) != 0)
        {
            if(LEVEL_1){sprintf(logLine,
            		"FILE: CO_driver.c"
            		"||CALL: setFilters"
            		"\nMSG: Setting socket option failed. Illegal argument "); logPrint(ERROR,logLine);}

            ret = CO_ERROR_ILLEGAL_ARGUMENT;
        }
    }

    return ret;
}


/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress){
	 if(LEVEL_1){sprintf(logLine,
			 "FILE: CO_driver.c"
			 "||CALL: CO_CANsetConfigurationMode"
			 "\nMSG: Stared"); logPrint(LOG,logLine);}

}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
	 if(LEVEL_1){sprintf(logLine,
			 "FILE: CO_driver.c"
			 "||CALL: CO_CANsetNormalMode"
			 "\nMSG: Stared"); logPrint(LOG,logLine);}

    /* set CAN filters */
    if(CANmodule == NULL || setFilters(CANmodule) != CO_ERROR_NO){
    	if(LEVEL_1){sprintf(logLine,
    			"FILE: CO_driver.c"
    			"||CALL: CO_CANsetNormalMode"
    			"\nMSG: CO_CANsetNormalMode failed"); logPrint(ERROR,logLine);}
        CO_errExit("CO_CANsetNormalMode failed");
    }
    CANmodule->CANnormal = true;
}


/******************************************************************************/
/*
 * Before we call canmodule init. We need to create rxArray,txArray, also get interface index of the can harDware interface.
 */
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,//can_ifindex info here.
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{

    if(LEVEL_1){sprintf(logLine,
    		"FILE: CO_driver.c"
    		"||CALL: CO_CANmodule_init"
    		"\nMSG: Init CAN module"); logPrint(LOG,logLine);}
    CO_ReturnError_t ret = CO_ERROR_NO;
    uint16_t i;

    /* verify arguments */
    /*KAR:
     * if there exists no { CAN module or CAN hardware interface or receive array or transmit array } then throw an error
     */
    if(CANmodule==NULL || CANbaseAddress==0 || rxArray==NULL || txArray==NULL){
        if(LEVEL_1){sprintf(logLine,
        		"FILE: CO_driver.c"
        		"||CALL: CO_CANmodule_init"
        		"\nMSG: In valid parameters"); logPrint(ERROR,logLine);}
    	ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */


    if(ret == CO_ERROR_NO){
    	 if(LEVEL_1){sprintf(logLine,
    	           		"FILE: CO_driver.c"
    	           		"||CALL: CO_CANmodule_init"
    	           		"\nMSG: Configure object variables"); logPrint(LOG,logLine);}
        CANmodule->CANbaseAddress = CANbaseAddress;//interface info in copied
        CANmodule->rxArray = rxArray;//address of the receive array is copied.
        CANmodule->rxSize = rxSize;//size of the receive array is copied
        CANmodule->txArray = txArray;//address of the transmit array is copied
        CANmodule->txSize = txSize;//size of the transmit array is copied
        CANmodule->CANnormal = false;// setting canmodule to configuration mode
        CANmodule->useCANrxFilters = true;//use hardware filters for receiving the can messages
        CANmodule->bufferInhibitFlag = false;//any sync PDO in the transmit buffer, since NO set to false.
        CANmodule->firstCANtxMessage = true;// do transmit buffer contain boot up message. can module is starting so bootup mode.
        CANmodule->error = 0;//no error exists
        CANmodule->CANtxCount = 0U;// no data in transmit buffer waiting for canmodule.
        CANmodule->errOld = 0U;//no old error
        CANmodule->em = NULL;//no emergency object

#ifdef CO_LOG_CAN_MESSAGES
        CANmodule->useCANrxFilters = false;
#endif
        if(LEVEL_1){sprintf(logLine,
               		"FILE: CO_driver.c"
               		"||CALL: CO_CANmodule_init"
               		"\nMSG: Init rxArray and txArray of the CAN module"); logPrint(LOG,logLine);}
//init for rxArray
        for(i=0U; i<rxSize; i++){
            rxArray[i].ident = 0U;
            rxArray[i].pFunct = NULL;
        }
//init for txArray
        for(i=0U; i<txSize; i++){
            txArray[i].bufferFull = false;
        }
    }

    /* First time only configuration */
    /*
     * create a socket and bind it.
     */
    if(ret == CO_ERROR_NO && CANmodule->wasConfigured == 0){
        if(LEVEL_1){sprintf(logLine,
               		"FILE: CO_driver.c"
               		"||CALL: CO_CANmodule_init"
               		"\nMSG: create and bind socket"); logPrint(LOG,logLine);}
        struct sockaddr_can sockAddr;

        CANmodule->wasConfigured = 1;

        /* Create and bind socket */
        CANmodule->fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if(CANmodule->fd < 0){
            if(LEVEL_1){sprintf(logLine,
                   		"FILE: CO_driver.c"
                   		"||CALL: CO_CANmodule_init"
                   		"\nMSG: socket creation failed"); logPrint(ERROR,logLine);}
            ret = CO_ERROR_ILLEGAL_ARGUMENT;
        }else{
        	//karthik started
        		struct ifreq ifr;
				//char* canInterfaceName="can0";
				//copy the can interface to the interface request (ifreq) object name field.
				strcpy(ifr.ifr_name, "can0");

				//get interface index from the interface name mapping.
				ioctl(CANmodule->fd, SIOCGIFINDEX, &ifr);
				CANbaseAddress=ifr.ifr_ifindex;

        	//karthik ended
            sockAddr.can_family = AF_CAN;
            sockAddr.can_ifindex = CANbaseAddress;
            if(bind(CANmodule->fd, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) != 0){
                if(LEVEL_1){sprintf(logLine,
                       		"FILE: CO_driver.c"
                       		"||CALL: CO_CANmodule_init"
                       		"\nMSG: socket binding failed"); logPrint(ERROR,logLine);}
                ret = CO_ERROR_ILLEGAL_ARGUMENT;
            }
        }

        /* allocate memory for filter array */
        /*
         * create a receive filter of size equal to rxArray. Each element of this array is of #can_filter type.
         * can_filter contains #canid and #canmask.
         */
        if(ret == CO_ERROR_NO){
            if(LEVEL_1){sprintf(logLine,
                   		"FILE: CO_driver.c"
                   		"||CALL: CO_CANmodule_init"
                   		"\nMSG: Allocating filter array"); logPrint(LOG,logLine);}
            CANmodule->filter = (struct can_filter *) calloc(rxSize, sizeof(struct can_filter));
            if(CANmodule->filter == NULL){
               if(LEVEL_1){sprintf(logLine,
                       		"FILE: CO_driver.c"
                       		"||CALL: CO_CANmodule_init"
                       		"\nMSG: Allocating filter array"); logPrint(ERROR,logLine);}
                ret = CO_ERROR_OUT_OF_MEMORY;
            }
        }
    }

    /* Additional check. */
    if(ret == CO_ERROR_NO && CANmodule->filter == NULL){
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure CAN module hardware filters */
    if(ret == CO_ERROR_NO && CANmodule->useCANrxFilters){
        if(LEVEL_1){sprintf(logLine,
               		"FILE: CO_driver.c"
               		"||CALL: CO_CANmodule_init"
               		"\nMSG: configure CAN module filters"); logPrint(LOG,logLine);}
        /* Match filter, standard 11 bit CAN address only, no rtr */

    	/*Init each filter.   <received_can_id> & mask == can_id & mask
    	 * allows only standard 11 bit CAN ID.
    	 */
        for(i=0U; i<rxSize; i++){
            CANmodule->filter[i].can_id = 0;

            //It matters whether it is SFF or EFF or rtr. we can identify all 3 of them using this setting
            CANmodule->filter[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
        }
    }

    /* close CAN module filters for now. */
    if(ret == CO_ERROR_NO){
        if(LEVEL_1){sprintf(logLine,
               		"FILE: CO_driver.c"
               		"||CALL: CO_CANmodule_init"
               		"\nMSG: setting socket option. For filter settings"); logPrint(LOG,logLine);}
        setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);// applying the filter setting done earlier.
    }

    return ret;
}


/******************************************************************************/
//disables canmodule.
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    if(LEVEL_1){sprintf(logLine,
           		"FILE: CO_driver.c"
           		"||CALL: CO_CANmodule_disable"
           		"\nMSG: started"); logPrint(LOG,logLine);}
    close(CANmodule->fd);
    free(CANmodule->filter);
    CANmodule->filter = NULL;
}


/******************************************************************************/
//read can id from the received message.
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg){
    if(LEVEL_1){sprintf(logLine,
           		"FILE: CO_driver.c"
           		"||CALL: CO_CANrxMsg_readIdent"
           		"\nMSG: started"); logPrint(LOG,logLine);}
    return (uint16_t) rxMsg->ident;
}


/******************************************************************************/
//init for rxBuffer elements. Each element in the rxArray need to be initialized using this call.
/*
 * rxArray belongs to a certain CANmodule.
 * rxArray each elements contains index, canid, mask
 * rtr If true, 'Remote Transmit Request' messages will be accepted..0
 */
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
	 if(LEVEL_1){sprintf(logLine,
	           		"FILE: CO_driver.c"
	           		"||CALL: CO_CANrxBufferInit"
	           		"\nMSG: started"); logPrint(LOG,logLine);}

    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) &&
       (CANmodule->filter!=NULL) && (index < CANmodule->rxSize))
    {

    	/* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

     	 if(LEVEL_1){sprintf(logLine,
     	           		"FILE: CO_driver.c"
     	           		"||CALL: CO_CANrxBufferInit"
     	           		"\nMSG: configure call back function "); logPrint(LOG,logLine);}

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* Configure CAN identifier and CAN mask, bit aligned with CAN module. */
    	 if(LEVEL_1){sprintf(logLine,
    	           		"FILE: CO_driver.c"
    	           		"||CALL: CO_CANrxBufferInit"
    	           		"\nMSG:  Configure CAN identifier and CAN mask"); logPrint(LOG,logLine);}
        //extract only 11 bit CAN ID contents
        buffer->ident = ident & CAN_SFF_MASK;

        //if rtr the sent rtr bit
        if(rtr){
       	 if(LEVEL_1){sprintf(logLine,
       	           		"FILE: CO_driver.c"
       	           		"||CALL: CO_CANrxBufferInit"
       	           		"\nMSG:  Configure for RTR"); logPrint(LOG,logLine);}
            buffer->ident |= CAN_RTR_FLAG;
        }

        //It matters whether it is (SFF & mask) or EFF or rtr. we can identify all 3 of them using this setting
        //(SFF & mask) means subset of SFF.
        buffer->mask = (mask & CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){
            CANmodule->filter[index].can_id = buffer->ident;
            CANmodule->filter[index].can_mask = buffer->mask;

            //set filters only if canmodule is in normal module.That is canmodule is up and running.
            if(CANmodule->CANnormal){
              	 if(LEVEL_1){sprintf(logLine,
              	           		"FILE: CO_driver.c"
              	           		"||CALL: CO_CANrxBufferInit"
              	           		"\nMSG:  Setting filter. CAN module in normal mode"); logPrint(LOG,logLine);}
                ret = setFilters(CANmodule);
            }
        }
    }
    else{
      	 if(LEVEL_1){sprintf(logLine,
      	           		"FILE: CO_driver.c"
      	           		"||CALL: CO_CANrxBufferInit"
      	           		"\nMSG: verify argument. Failed. Illegal arguments"); logPrint(ERROR,logLine);}
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
/*
 * This function puts the data into the buffer.
 */
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
	 if(LEVEL_1){sprintf(logLine,
	           		"FILE: CO_driver.c"
	           		"||CALL: CO_CANtxBufferInit"
	           		"\nMSG: started"); logPrint(LOG,logLine);}

    CO_CANtx_t *buffer = NULL;

    // check if canmodule exists or not and index is less than the size of the txArray.
    if((CANmodule != NULL) && (index < CANmodule->txSize)){
   	 if(LEVEL_1){sprintf(logLine,
   	           		"FILE: CO_driver.c"
   	           		"||CALL: CO_CANtxBufferInit"
   	           		"\nMSG: Configure element of txArray. txArray[%d]",index); logPrint(LOG,logLine);}
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, bit aligned with CAN module registers */
        buffer->ident = ident & CAN_SFF_MASK;

        //if rtr request set its flag bit.
        if(rtr){
            buffer->ident |= CAN_RTR_FLAG;
        }

        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;// not an sync PDO.
        buffer->syncFlag = syncFlag;//mention is it a sync message?
    }
    else
    {
    	if(LEVEL_1){sprintf(logLine,
    			"FILE: CO_driver.c"
    			"||CALL: CO_CANtxBufferInit"
    			"\nMSG: txArray cannot be configured"); logPrint(LOG,logLine);}
    }

    return buffer;
}


/******************************************************************************/
/*
 * This function writes the data from buffer to the socket.
 */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: CO_CANsend"
			"\nMSG: started"); logPrint(LOG,logLine);}

    CO_ReturnError_t err = CO_ERROR_NO;
    ssize_t n;
    size_t count = sizeof(struct can_frame);

//write the data in the buffer to the socket
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: CO_CANsend"
			"\nMSG: write message to the buffer"); logPrint(LOG,logLine);}
    n = write(CANmodule->fd, buffer, count);
#ifdef CO_LOG_CAN_MESSAGES
    void CO_logMessage(const CanMsg *msg);
    CO_logMessage((const CanMsg*) buffer);
#endif

//if error in sending. if checked using the number bytes transferred (n).
    if(n != count){
    	if(LEVEL_1){sprintf(logLine,
    			"FILE: CO_driver.c"
    			"||CALL: CO_CANsend"
    			"\nMSG: Failed to write message into the socket"); logPrint(ERROR,logLine);}
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, n);
        err = CO_ERROR_TX_OVERFLOW;
    }

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: CO_CANclearPendingSyncPDOs"
			"\nMSG: started"); logPrint(LOG,logLine);}
    /* Messages can not be cleared, because they are allready in kernel */
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
#if 0
    unsigned rxErrors, txErrors;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;

    canGetErrorCounters(CANmodule->CANbaseAddress, &rxErrors, &txErrors);
    if(txErrors > 0xFFFF) txErrors = 0xFFFF;
    if(rxErrors > 0xFF) rxErrors = 0xFF;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | CANmodule->error;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(txErrors >= 256U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                bool_t isError = CO_isError(em, CO_EM_CAN_BUS_WARNING);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }
        }

        if(CANmodule->error & 0x02){                       /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
#endif
}


/******************************************************************************/
/*
 * This functions read a message from the socket and matches it with the rxArray canid.
 * If received message canid matches with the rxArray canid then callback function pointed by the rxArray is called
 * if not matching then exists silently.
 * */
void CO_CANrxWait(CO_CANmodule_t *CANmodule){
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: CO_CANrxWait"
			"\nMSG: started"); logPrint(LOG,logLine);}

    struct can_frame msg;
    int n, size;

    if(CANmodule == NULL){
        errno = EFAULT;
    	if(LEVEL_1){sprintf(logLine,
    			"FILE: CO_driver.c"
    			"||CALL: CO_CANrxWait"
    			"\nMSG: CAN module does not exist"); logPrint(ERROR,logLine);}
        CO_errExit("CO_CANreceive - CANmodule not configured.");
    }

    /* Read socket and pre-process message */
	if(LEVEL_1){sprintf(logLine,
			"FILE: CO_driver.c"
			"||CALL: CO_CANrxWait"
			"\nMSG: read from socket"); logPrint(LOG,logLine);}

    size = sizeof(struct can_frame);
    n = read(CANmodule->fd, &msg, size);

    if(CANmodule->CANnormal){
        if(n != size){
        	if(LEVEL_1){sprintf(logLine,
        			"FILE: CO_driver.c"
        			"||CALL: CO_CANrxWait"
        			"\nMSG: error while reading socket"); logPrint(ERROR,logLine);}
            /* This happens only once after error occurred (network down or something). */
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_COMMUNICATION, n);
        }
        else{

            CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
            uint32_t rcvMsgIdent;       /* identifier of the received message */
            CO_CANrx_t *buffer;         /* receive message buffer from CO_CANmodule_t object. */
            int i;
            bool_t msgMatched = false;

            rcvMsg = (CO_CANrxMsg_t *) &msg;  //typecast for received message type
            rcvMsgIdent = rcvMsg->ident;

            /* Search rxArray form CANmodule for the matching CAN-ID. */
            if(LEVEL_1){sprintf(logLine,
            		"FILE: CO_driver.c"
            		"||CALL: CO_CANrxWait"
            		"\nMSG: Searching rxArray from canModule for matching CAN-ID"); logPrint(LOG,logLine);}

            buffer = &CANmodule->rxArray[0];
            for(i = CANmodule->rxSize; i > 0U; i--)
            {
                if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                    if(LEVEL_1){sprintf(logLine,
                    		"FILE: CO_driver.c"
                    		"||CALL: CO_CANrxWait"
                    		"\nMSG: CAN ID matched with rxArray element"); logPrint(LOG,logLine);}
                    msgMatched = true;
                    break;
                }
                buffer++;
            }

            if(msgMatched==false)
            {
                if(LEVEL_1){sprintf(logLine,
                		"FILE: CO_driver.c"
                		"||CALL: CO_CANrxWait"
                		"\nMSG: CAN ID did not match with rxArray element"); logPrint(LOG,logLine);}
            }

            /* Call specific function, which will process the message */
            if(msgMatched && (buffer->pFunct != NULL)){

                if(LEVEL_1){sprintf(logLine,
                		"FILE: CO_driver.c"
                		"||CALL: CO_CANrxWait"
                		"\nMSG: Calling function registered to the received message CANID"); logPrint(LOG,logLine);}
                buffer->pFunct(buffer->object, rcvMsg);
            }

#ifdef CO_LOG_CAN_MESSAGES
            void CO_logMessage(const CanMsg *msg);
            CO_logMessage((CanMsg*)&rcvMsg);
#endif
        }
    }
}




