
#include "CO_LSS.h"

static void CO_LSS_receive(void *object, const CO_CANrxMsg_t *msg);
static void CO_LSS_receive(void *object, const CO_CANrxMsg_t *msg) {
	if (LEVEL_1) {
		sprintf(logLine, "FILE: CO_SDO.c"
				"||CALL: CO_SDO_receive"
				"\nMSG: started");
		logPrint(LOG, logLine);
	}

	CO_LSS_t *LSS;

	LSS = (CO_LSS_t*) object; /* this is the correct pointer type of the first argument */

	/* verify message length and message overflow (previous message was not processed yet) */
	if ((msg->DLC == 8U) && (!LSS->LSSRespData.CANrxNew)) {
		if (LEVEL_1) {
			sprintf(logLine, "FILE: CO_SDO.c"
					"||CALL: CO_SDO_receive"
					"\nMSG: Valid SDO message and Prev message is processed.");
			logPrint(LOG, logLine);
		}
	}
	__u8 temp[4];
	switch (LSS->state) {
	case CO_LSS_ST_SWITCH_MODE_SELECTIVE_RESP:
		if (msg->data[0] == SWITCH_MODE_SELECTIVE_RESPONSE_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			LSS->LSSRespData.mode = msg->data[1];
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_CONFIGURE_NODE_ID_RESP:
		if (msg->data[0] == CONFIGURE_NODE_ID_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			LSS->LSSRespData.errorCode = msg->data[1];
			LSS->LSSRespData.specificCode = msg->data[2];
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_RESP:
		if (msg->data[0] == CONFIGURE_BIT_TIMING_PARAMETERS_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			LSS->LSSRespData.errorCode = msg->data[1];
			LSS->LSSRespData.specificCode = msg->data[2];
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_STORE_CONFIGURATION_RESP:
		if (msg->data[0] == STORE_CONFIGURATION_SETTINGS_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			LSS->LSSRespData.errorCode = msg->data[1];
			LSS->LSSRespData.specificCode = msg->data[2];
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_INQUIRE_VENDOR_ID_RESP:
		if (msg->data[0] == INQUIRE_VENDOR_ID_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			//fill temp buffer with array of data from vendor id.
			temp[0] = msg->data[1];
			temp[1] = msg->data[2];
			temp[2] = msg->data[3];
			temp[3] = msg->data[4];
			//get 32bit vendor from 4x8 bit array.
			LSS->LSSRespData.vendorID = CO_getUint32(temp);
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_INQUIRE_PRODUCT_ID_RESP:
		if (msg->data[0] == INQUIRE_PRODUCT_CODE_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			//fill temp buffer with array of data from vendor id.
			temp[0] = msg->data[1];
			temp[1] = msg->data[2];
			temp[2] = msg->data[3];
			temp[3] = msg->data[4];
			//get 32bit vendor from 4x8 bit array.
			LSS->LSSRespData.productID = CO_getUint32(temp);
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_INQUIRE_REVISION_ID_RESP:
		if (msg->data[0] == INQUIRE_REVISION_NUMBER_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			//fill temp buffer with array of data from vendor id.
			temp[0] = msg->data[1];
			temp[1] = msg->data[2];
			temp[2] = msg->data[3];
			temp[3] = msg->data[4];
			//get 32bit vendor from 4x8 bit array.
			LSS->LSSRespData.revisionNumber = CO_getUint32(temp);
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_INQUIRE_SERIAL_NUM_RESP:
		if (msg->data[0] == INQUIRE_SERIAL_NUMBER_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			//fill temp buffer with array of data from vendor id.
			temp[0] = msg->data[1];
			temp[1] = msg->data[2];
			temp[2] = msg->data[3];
			temp[3] = msg->data[4];
			//get 32bit vendor from 4x8 bit array.
			LSS->LSSRespData.serialNumber = CO_getUint32(temp);
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	case CO_LSS_ST_IDENTIFY_SLAVE_RESP:
		if (msg->data[0] == IDENTIFY_SLAVE_CSC) {
			LSS->LSSRespData.slaveCommandSpecifier = msg->data[0];
			LSS->LSSRespData.CANrxNew = true;
		}
		break;
	default:
		LSS->LSSRespData.CANrxNew = false;
		break;
	}

}

CO_LSS_ReturnError_t CO_LSS_init(CO_LSS_t *LSS, CO_CANmodule_t *CANdevRx,
		uint16_t CANdevRxIdx, CO_CANmodule_t *CANdevTx, uint16_t CANdevTxIdx) {
	if (LEVEL_1) {
		sprintf(logLine, "FILE: CO_LSS.c"
				"||CALL: CO_LSS_init"
				"\nMSG: started");
		logPrint(LOG, logLine);
	}
	/* verify arguments */
	/*
	 * LSS master object should exist.
	 * CANmodule for sdo server to receive should exist
	 * CAN module for sdo server to transmit should exist
	 */
	if (LSS == NULL || CANdevRx == NULL || CANdevTx == NULL) {

		if (LEVEL_1) {
			sprintf(logLine, "FILE: CO_LSS.c"
					"||CALL: CO_LSS_init"
					"\nMSG: Illegal argument");
			logPrint(ERROR, logLine);
		}

		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	//Configure
	LSS->LSSTxdata.nodeId = 0;
	LSS->LSSTxdata.vendorID = 0;
	LSS->LSSTxdata.productCode = 0;
	LSS->LSSTxdata.revisionNumber = 0;
	LSS->LSSTxdata.serialNumber = 0;
	LSS->LSSTxdata.switchDelay = 0;
	LSS->LSSTxdata.tableIndex = 0;
	LSS->LSSTxdata.tableSelector = 0;

	uint32_t COB_IDMasterToSlave = 0X7E5;
	uint32_t COB_IDSlaveToMaster = 0X7E4;

	/* configure SDO server CAN reception */
	CO_CANrxBufferInit(CANdevRx, /* CAN device */
	CANdevRxIdx, /* rx buffer index */
	COB_IDSlaveToMaster, /* CAN identifier */
	0x7FF, /* mask */
	0, /* rtr */
	(void*) LSS, /* object passed to receive function */
	CO_LSS_receive); /* this function will process received message */

	LSS->CANdevTx = CANdevTx;
	LSS->CANtxBuff = CO_CANtxBufferInit(CANdevTx, /* CAN device */
	CANdevTxIdx, /* index of specific buffer inside CAN module */
	COB_IDMasterToSlave, /* CAN identifier */
	0, /* rtr */
	8, /* number of data bytes */
	0); /* synchronous message flag bit */
	if (LEVEL_1) {
		sprintf(logLine, "FILE: CO_LSS.c"
				"||CALL: CO_LSS_init"
				"\nMSG: Init successful");
		logPrint(ERROR, logLine);
	}
	return CO_ERROR_NO;
}

int8_t CO_LSS_process(CO_LSS_t *LSS, bool_t NMTisStopped,
		uint16_t timeDifference_ms, uint16_t LSStimeoutTime,
		uint16_t *timerNext_ms) {
	if (LEVEL_1) {
		sprintf(logLine, "FILE: CO_LSS.c"
				"||CALL: CO_LSS_process"
				"\nMSG: started");
		logPrint(LOG, logLine);
	}



	/* return if idle */
	if (LSS->state == CO_LSS_ST_IDLE) {
		if (LEVEL_1) {
			sprintf(logLine, "FILE: CO_LSS.c"
					"||CALL: CO_LSS_process"
					"\nMSG: No new msg recved, and LSS Master in Idle state");
			logPrint(LOG, logLine);
		}
		return NO_RESPONSE;
	}

	//if its not NMT stopped then LSS does not work.
	//LSS works on in stopped mode.
	if (!NMTisStopped) {
		if (LEVEL_1) {
			sprintf(logLine,
					"FILE: CO_LSS.c"
							"||CALL: CO_LSS_process"
							"\nMSG: This node is Not in stopped mode. LSS Communication is not support in this mode.");
			logPrint(LOG, logLine);
		}
		LSS->state = CO_LSS_ST_IDLE;
		return NO_RESPONSE;
	}

	/* verify SDO timeout */
	if (LSS->timeoutTimer < LSStimeoutTime) {
		LSS->timeoutTimer += timeDifference_ms;
		LSS->LSSRespData.isRespTimeout = false;
	} else {
		LSS->LSSRespData.isRespTimeout = true;
		return RESPONSE_TIMEOUT;
	}

	uint8_t temp[4];
	switch (LSS->state) {
	case CO_LSS_ST_IDLE:
		LSS->timeoutTimer = 0;
		break;

	case CO_LSS_ST_SWITCH_MODE_GLOBAL_TO_CONFIGURATION_MODE_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_GLOBAL_CSC;
		//mode switching
		LSS->CANtxBuff->data[1] = CONFIGURATION_MODE;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_IDLE;
		break;

	case CO_LSS_ST_SWITCH_MODE_GLOBAL_TO_OPERATION_MODE_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_GLOBAL_CSC;
		//mode switching
		LSS->CANtxBuff->data[1] = OPERATION_MODE;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_IDLE;
		break;

	case CO_LSS_ST_SWITCH_MODE_SELECTIVE_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//VENDOR ID
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_SELECTIVE_VENDOR_ID_CSC;

		//Vendor ID added
		CO_setUint32(temp, LSS->LSSTxdata.vendorID);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];

		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//PRODUCT CODE
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_SELECTIVE_PRODUCT_CODE_CSC;

		//product code added
		CO_setUint32(temp, LSS->LSSTxdata.productCode);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];

		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//REVISION NUMBER
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_SELECTIVE_REVISION_NUM_CSC;

		//revision number added
		CO_setUint32(temp, LSS->LSSTxdata.revisionNumber);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];

		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//SERIAL NUMBER
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = SWITCH_MODE_SELECTIVE_SERIAL_NUM_CSC;

		//serial number added
		CO_setUint32(temp, LSS->LSSTxdata.serialNumber);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];

		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//No response to be received so switch to get response.
		LSS->state = CO_LSS_ST_SWITCH_MODE_SELECTIVE_RESP;
		break;

	case CO_LSS_ST_SWITCH_MODE_SELECTIVE_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_CONFIGURE_NODE_ID_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = CONFIGURE_NODE_ID_CSC;
		//Node ID added
		LSS->CANtxBuff->data[1] = LSS->LSSTxdata.nodeId;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_CONFIGURE_NODE_ID_RESP;
		break;

	case CO_LSS_ST_CONFIGURE_NODE_ID_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = CONFIGURE_BIT_TIMING_PARAMETERS_CSC;
		//table index and selector added
		LSS->CANtxBuff->data[1] = LSS->LSSTxdata.tableSelector;
		LSS->CANtxBuff->data[2] = LSS->LSSTxdata.tableIndex;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_RESP;
		break;
//--TO DO
	case CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_ACTIVATE_BIT_TIMING_PARAM_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = ACTIVATE_BIT_TIMING_PARAMETERS_CSC;
		//switch delay added
		CO_setUint16(temp, LSS->LSSTxdata.switchDelay);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_IDLE;
		break;

	case CO_LSS_ST_STORE_CONFIGURATION_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = STORE_CONFIGURATION_SETTINGS_CSC;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[1] = 0;
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_STORE_CONFIGURATION_RESP;
		break;

//--TO DO
	case CO_LSS_ST_STORE_CONFIGURATION_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_INQUIRE_VENDOR_ID_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = INQUIRE_VENDOR_ID_CSC;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[1] = 0;
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_INQUIRE_VENDOR_ID_RESP;
		break;

//--TO DO
	case CO_LSS_ST_INQUIRE_VENDOR_ID_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_INQUIRE_PRODUCT_ID_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = INQUIRE_PRODUCT_CODE_CSC;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[1] = 0;
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_INQUIRE_PRODUCT_ID_RESP;
		break;

//--TO DO
	case CO_LSS_ST_INQUIRE_PRODUCT_ID_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_INQUIRE_REVISION_ID_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = INQUIRE_REVISION_NUMBER_CSC;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[1] = 0;
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_INQUIRE_REVISION_ID_RESP;
		break;

//--TO DO
	case CO_LSS_ST_INQUIRE_REVISION_ID_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_INQUIRE_SERIAL_NUM_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = INQUIRE_SERIAL_NUMBER_CSC;
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[1] = 0;
		LSS->CANtxBuff->data[2] = 0;
		LSS->CANtxBuff->data[3] = 0;
		LSS->CANtxBuff->data[4] = 0;
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);
		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_INQUIRE_SERIAL_NUM_RESP;
		break;

//--TO DO
	case CO_LSS_ST_INQUIRE_SERIAL_NUM_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	case CO_LSS_ST_IDENTITY_REMOTE_SLAVE_REQ:
		//clear response data structure before any request
		CO_LSS_ClearRespData(LSS);

		//VENDOR ID
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_VENDOR_ID_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.vendorID);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//PRODUCT CODE
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_PRODUCT_CODE_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.productCode);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//REVISION NUMBER LOW
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_REVISION_NUM_LOW_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.revisionNumberLow);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//REVISION NUMBER HIGH
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_REVISION_NUM_HIGH_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.revisionNumberHigh);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//SERIAL NUMBER LOW
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_SERIAL_NUM_LOW_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.serialNumberLow);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//SERIAL NUMBER HIGH
		//fill data into the CANtxBuffer
		//Command specifiers
		LSS->CANtxBuff->data[0] = IDENTIFY_REMOTE_SLAVE_SERIAL_NUM_HIGH_CSC;
		CO_setUint32(temp, LSS->LSSTxdata.serialNumberHigh);
		LSS->CANtxBuff->data[1] = temp[0];
		LSS->CANtxBuff->data[2] = temp[1];
		LSS->CANtxBuff->data[3] = temp[2];
		LSS->CANtxBuff->data[4] = temp[3];
		//Other data bytes filled with zeros
		LSS->CANtxBuff->data[5] = 0;
		LSS->CANtxBuff->data[6] = 0;
		LSS->CANtxBuff->data[7] = 0;
		//send the can data
		CO_CANsend(LSS->CANdevTx, LSS->CANtxBuff);

		//No response to be received so switch to idle state.
		LSS->state = CO_LSS_ST_IDENTIFY_SLAVE_RESP;
		break;

	case CO_LSS_ST_IDENTIFY_SLAVE_RESP:
		if ((LSS->LSSRespData.isRespTimeout == false)
				&& (LSS->LSSRespData.CANrxNew == true)) {
			LSS->state = CO_LSS_ST_IDLE;
			LSS->LSSRespData.CANrxNew = false;
			return RESPONSE_RECV;
		}
		break;

	default:
		LSS->state = CO_LSS_ST_IDLE;
		LSS->LSSRespData.CANrxNew = false;
		break;

	}
	return 0;
}

void CO_LSS_ClearRespData(CO_LSS_t *LSS) {
	LSS->LSSRespData.CANrxNew = false;
	LSS->LSSRespData.errorCode = 0;
	LSS->LSSRespData.mode = 0;
	LSS->LSSRespData.productID = 0;
	LSS->LSSRespData.revisionNumber = 0;
	LSS->LSSRespData.serialNumber = 0;
	LSS->LSSRespData.slaveCommandSpecifier = 0;
	LSS->LSSRespData.specificCode = 0;
	LSS->LSSRespData.vendorID = 0;

}

