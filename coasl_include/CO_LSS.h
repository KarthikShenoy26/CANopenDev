/*
 * CO_LSS.h
 *
 *  Created on: Jul 30, 2017
 *      Author: karthik
 */

#ifndef COASL_INCLUDE_CO_LSS_H_
#define COASL_INCLUDE_CO_LSS_H_


				#include "CO_SDO.h"
				#include "CO_OD.h"

				#define NO_RESPONSE 0
				#define RESPONSE_RECV 1
				#define RESPONSE_TIMEOUT 3

				#define CONFIGURATION_MODE 0X01
				#define OPERATION_MODE 0X00

				enum lss_commands
				{
				  SWITCH_MODE_GLOBAL_CSC = 4,      /**<CANOpen LSS switch mode global command */

				  CONFIGURE_NODE_ID_CSC = 17,

				  CONFIGURE_BIT_TIMING_PARAMETERS_CSC = 19,

				  ACTIVATE_BIT_TIMING_PARAMETERS_CSC=21,

				  STORE_CONFIGURATION_SETTINGS_CSC = 23,    /**<CANOpen LSS store configuration settings command */

				  SWITCH_MODE_SELECTIVE_VENDOR_ID_CSC=64,
				  SWITCH_MODE_SELECTIVE_PRODUCT_CODE_CSC=65,
				  SWITCH_MODE_SELECTIVE_REVISION_NUM_CSC=66,
				  SWITCH_MODE_SELECTIVE_SERIAL_NUM_CSC=67,
				  SWITCH_MODE_SELECTIVE_RESPONSE_CSC=68,

				  INQUIRE_VENDOR_ID_CSC = 90,               /**<CANOpen LSS inquire vendor id command */
				  INQUIRE_PRODUCT_CODE_CSC = 91,            /**<CANOpen LSS inquire product code command */
				  INQUIRE_REVISION_NUMBER_CSC = 92,         /**<CANOpen LSS inquire revision number command */
				  INQUIRE_SERIAL_NUMBER_CSC = 93,           /**<CANOpen LSS inquire serial number command */

				  IDENTITY_REMOTE_SLAVE_VENDOR_ID_CSC =70,
				  IDENTITY_REMOTE_SLAVE_PRODUCT_CODE_CSC =71,
				  IDENTITY_REMOTE_SLAVE_REVISION_NUM_LOW_CSC =72,
				  IDENTITY_REMOTE_SLAVE_REVISION_NUM_HIGH_CSC =73,
				  IDENTITY_REMOTE_SLAVE_SERIAL_NUM_LOW_CSC =74,
				  IDENTITY_REMOTE_SLAVE_SERIAL_NUM_HIGH_CSC =75,

				  IDENTITY_SLAVE_CSC=79,
				};



				enum lss_bit_rate
				{
				  BIT_RATE_1000 = 1000, /**<CANOpen LSS 1000k bits per second */
				  BIT_RATE_800 = 800,   /**<CANOpen LSS 800k bits per second */
				  BIT_RATE_500 = 500,   /**<CANOpen LSS 500k bits per second */
				  BIT_RATE_250 = 250,   /**<CANOpen LSS 250k bits per second */
				  BIT_RATE_125 = 125,   /**<CANOpen LSS 125k bits per second */
				  BIT_RESERVED = 0,     /**<CANOpen LSS reserved */
				  BIT_RATE_50 = 50,     /**<CANOpen LSS 50k bits per second */
				  BIT_RATE_20 = 20,     /**<CANOpen LSS 20k bits per second */
				  BIT_RATE_10 = 10      /**<CANOpen LSS 10k bits per second */
				};




				typedef struct
				{
					uint8_t        nodeId;
					UNSIGNED32     vendorID;
					UNSIGNED32     productCode;
					UNSIGNED32     revisionNumber;
					UNSIGNED32     revisionNumberHigh;
					UNSIGNED32     revisionNumberLow;
					UNSIGNED32     serialNumber;
					UNSIGNED32     serialNumberHigh;
					UNSIGNED32     serialNumberLow;
					UNSIGNED16     switchDelay;
					UNSIGNED8      tableSelector;
					UNSIGNED8      tableIndex;
				}CO_LSS_Tx_Data_t;


				typedef struct
				{
					UNSIGNED8 slaveCommandSpecifier;
					UNSIGNED8 errorCode;
					UNSIGNED8 specificCode;
					UNSIGNED8 mode;
					UNSIGNED32 vendorID;
					UNSIGNED32 productID;
					UNSIGNED32 revisionNumber;
					UNSIGNED32 serialNumber;
					bool_t     isRespTimeout;
					bool_t 	  CANrxNew;
				}CO_LSS_Resp_Data_t;


				typedef enum
				{
						CO_LSS_ERROR_NO                 = 0,
						CO_LSS_ERROR_ILLEGAL_ARGUMENT   = -1,
						CO_LSS_ERROR_OUT_OF_MEMORY      = -2,
						CO_LSS_ERROR_TIMEOUT            = -3,
				}CO_LSS_ReturnError_t;


				typedef enum
				{
					CO_LSS_ST_IDLE=0X00,
					CO_LSS_ST_SWITCH_MODE_GLOBAL_TO_CONFIGURATION_MODE_REQ=0X10,
					CO_LSS_ST_SWITCH_MODE_GLOBAL_TO_OPERATION_MODE_REQ=0X11,
					CO_LSS_ST_SWITCH_MODE_SELECTIVE_REQ=0X20,
					CO_LSS_ST_SWITCH_MODE_SELECTIVE_RESP=0X21,
					CO_LSS_ST_CONFIGURE_NODE_ID_REQ=0X30,
					CO_LSS_ST_CONFIGURE_NODE_ID_RESP=0X31,
					CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_REQ=0X40,
					CO_LSS_ST_CONFIGURE_BIT_TIMING_PARAM_RESP=0X41,
					CO_LSS_ST_ACTIVATE_BIT_TIMING_PARAM_REQ=0X50,
					CO_LSS_ST_STORE_CONFIGURATION_REQ=0X60,
					CO_LSS_ST_STORE_CONFIGURATION_RESP=0X61,
					CO_LSS_ST_INQUIRE_VENDOR_ID_REQ=0X70,
					CO_LSS_ST_INQUIRE_VENDOR_ID_RESP=0X71,
					CO_LSS_ST_INQUIRE_PRODUCT_ID_REQ=0X80,
					CO_LSS_ST_INQUIRE_PRODUCT_ID_RESP=0X81,
					CO_LSS_ST_INQUIRE_REVISION_ID_REQ=0X90,
					CO_LSS_ST_INQUIRE_REVISION_ID_RESP=0X91,
					CO_LSS_ST_INQUIRE_SERIAL_NUM_REQ=0XA0,
					CO_LSS_ST_INQUIRE_SERIAL_NUM_RESP=0XA1,
					CO_LSS_ST_IDENTITY_REMOTE_SLAVE_REQ=0XB0,
					CO_LSS_ST_IDENTIFY_SLAVE_RESP=0XC0,
				}CO_LSS_state_t;

				typedef struct
				{
					CO_LSS_Tx_Data_t  LSSTxdata;
					CO_LSS_Resp_Data_t LSSRespData;
					CO_LSS_state_t  state;
					uint16_t        timeoutTimer;
					CO_CANmodule_t *CANdevTx;
					CO_CANtx_t     *CANtxBuff;
				}CO_LSS_t;



				int8_t CO_LSS_process(
						CO_LSS_t               *LSS,
						bool_t                  NMTisStopped,
						uint16_t                timeDifference_ms,
						uint16_t                SDOtimeoutTime,
						uint16_t               *timerNext_ms);


				CO_LSS_ReturnError_t CO_LSS_init(
						CO_LSS_t			   *LSS,
						CO_CANmodule_t         *CANdevRx,
						uint16_t                CANdevRxIdx,
						CO_CANmodule_t         *CANdevTx,
						uint16_t                CANdevTxIdx);


				void CO_LSS_ClearRespData(
						CO_LSS_t *LSS);


#endif /* COASL_INCLUDE_CO_LSS_H_ */
