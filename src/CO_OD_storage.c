/*
 * CANopen Object Dictionary storage object for Linux SocketCAN.
 *
 * @file        CO_OD_storage.c
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
#include "CO_SDO.h"
#include "CO_Emergency.h"
#include "CO_OD_storage.h"
#include "crc16-ccitt.h"

#include <stdio.h>
#include <string.h>     /* for memcpy */
#include <stdlib.h>     /* for malloc, free */
#include"Logger.h"

#define RETURN_SUCCESS  0
#define RETURN_ERROR   -1


/******************************************************************************/
CO_SDO_abortCode_t CO_ODF_1010(CO_ODF_arg_t *ODF_arg) {
//**************************************************************************************************/
	if(LEVEL_1){
		  				 sprintf(logLine,"FILE:CO_OD_storage.C||"
		  						 "Call: CO_ODF_1010"
		  						 "\n, msg: start");
		  			   	 logPrint(LOG,logLine);}
//**************************************************************************************************/


	CO_OD_storage_t *odStor;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    odStor = (CO_OD_storage_t*) ODF_arg->object;
//**************************************************************************************************/
	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_OD_storage.C||"
								 "Call: CO_ODF_1010"
								 "\n, msg: call CO_getUint32 ");
						 logPrint(LOG,logLine);}
//**************************************************************************************************/
    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading) {
        /* don't change the old value */
        CO_memcpy(ODF_arg->data, (const uint8_t*)ODF_arg->ODdataStorage, 4U);
//**************************************************************************************************/
	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_OD_storage.C||"
								 "Call: CO_ODF_1010"
								 "\n, msg: check subindex of ODF_arg");
						 logPrint(LOG,logLine);}
//**************************************************************************************************/



        if(ODF_arg->subIndex == 1) {
            /* store parameters */
if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_OD_storage.C||"
					 "Call: CO_ODF_1010"
					 "\n, msg: check if the value at subindex 1 is ASCII equivalent of 'SAVE'");
			 			 logPrint(LOG,logLine);}

            if(value == 0x65766173UL) {

if(LEVEL_1){
 sprintf(logLine,"FILE:CO_OD_storage.C||"
		 "Call: CO_ODF_1010"
		 "\n, msg: CO_OD_storage_saveSecure is called ");
			 logPrint(LOG,logLine);}


                if(CO_OD_storage_saveSecure(odStor->odAddress, odStor->odSize, odStor->filename) != 0) {
                    ret = CO_SDO_AB_HW;
                }
            }
            else {
	if(LEVEL_1){
	 sprintf(logLine,"FILE:CO_OD_storage.C||"
			 "Call: CO_ODF_1010"
			 "\n, ERROR: the value at subindex 1 is not ASCII equivalent of 'SAVE' ");
				 logPrint(ERROR,logLine);}

                ret = CO_SDO_AB_DATA_TRANSF;
            }
        }
    }

    return ret;
}


/******************************************************************************/
CO_SDO_abortCode_t CO_ODF_1011(CO_ODF_arg_t *ODF_arg) {

if(LEVEL_1){
 sprintf(logLine,"FILE:CO_OD_storage.C||"
		 "Call: CO_ODF_1011"
		 "\n, msg: start ");
			 logPrint(LOG,logLine);}



    CO_OD_storage_t *odStor;
    uint32_t value;
    CO_SDO_abortCode_t ret = CO_SDO_AB_NONE;

    odStor = (CO_OD_storage_t*) ODF_arg->object;

    if(LEVEL_1){
		 sprintf(logLine,"FILE:CO_OD_storage.C||"
				 "Call: CO_ODF_1011"
				 "\n, msg: call CO_getUint32 ");
		 logPrint(LOG,logLine);}

    value = CO_getUint32(ODF_arg->data);

    if(!ODF_arg->reading) {
        /* don't change the old value */
        CO_memcpy(ODF_arg->data, (const uint8_t*)ODF_arg->ODdataStorage, 4U);

        if(ODF_arg->subIndex >= 1) {
            /* restore default parameters */

if(LEVEL_1){
 sprintf(logLine,"FILE:CO_OD_storage.C||"
		 "Call: CO_ODF_1011"
		 "\n, msg: check if the value at subindex 1 is ASCII equivalent of 'LOAD'");
			 logPrint(LOG,logLine);}


            if(value == 0x64616F6CUL) {

            	if(LEVEL_1){
            	 sprintf(logLine,"FILE:CO_OD_storage.C||"
            			 "Call: CO_ODF_1011"
            			 "\n, msg: CO_OD_storage_restoreSecure is called ");
            				 logPrint(LOG,logLine);}

                if(CO_OD_storage_restoreSecure(odStor->filename) != 0) {
                    ret = CO_SDO_AB_HW;
                }
            }
            else {
	if(LEVEL_1){
	 sprintf(logLine,"FILE:CO_OD_storage.C||"
			 "Call: CO_ODF_1011"
			 "\n, ERROR: the value at subindex 1 is not ASCII equivalent of 'LOAD' ");
				 logPrint(ERROR,logLine);}

                ret = CO_SDO_AB_DATA_TRANSF;
            }
        }
    }

    return ret;
}


/******************************************************************************/
int CO_OD_storage_saveSecure(
        uint8_t                *odAddress,
        uint32_t                odSize,
        char                   *filename)
{

	if(LEVEL_1){
	 sprintf(logLine,"FILE:CO_OD_storage.C||"
			 "Call: CO_OD_storage_saveSecure"
			 "\n, msg:start ");
				 logPrint(LOG,logLine);}
    int ret = RETURN_SUCCESS;

    char *filename_old = NULL;
    uint16_t CRC = 0;

    /* Generate new string with extension '.old' and rename current file to it. */
    filename_old = malloc(strlen(filename)+10);

    if(LEVEL_1){
    	 sprintf(logLine,"FILE:CO_OD_storage.C||"
    			 "Call: CO_OD_storage_saveSecure"
    			 "\n, msg:check if the filename_old is not null ");
    	 logPrint(LOG,logLine);}

    if(filename_old != NULL) {
        strcpy(filename_old, filename);
        strcat(filename_old, ".old");

        remove(filename_old);
        if(rename(filename, filename_old) != 0) {

if(LEVEL_1){
		 sprintf(logLine,"FILE:CO_OD_storage.C||"
				 "Call: CO_OD_storage_saveSecure"
				 "\n, ERROR:rename of filename to filename_old failed ");
		 logPrint(ERROR,logLine);}

            ret = RETURN_ERROR;
        }
    } else {

    	 if(LEVEL_1){
    	    	 sprintf(logLine,"FILE:CO_OD_storage.C||"
    	    			 "Call: CO_OD_storage_saveSecure"
    	    			 "\n, ERROR:check if the filename_old is null ");
    	    	 logPrint(ERROR,logLine);}
        ret = RETURN_ERROR;
    }

    if(LEVEL_1){
        	    	 sprintf(logLine,"FILE:CO_OD_storage.C||"
        	    			 "Call: CO_OD_storage_saveSecure"
        	    			 "\n, msg:open a new file and write data to it begins ");
        	    	 logPrint(LOG,logLine);}


    /* Open a new file and write data to it, including CRC. */
    if(ret == RETURN_SUCCESS) {
        FILE *fp = fopen(filename, "w");
        if(fp != NULL) {

            CO_LOCK_OD();
            fwrite((const void *)odAddress, 1, odSize, fp);
            CRC = crc16_ccitt((unsigned char*)odAddress, odSize, 0);
            CO_UNLOCK_OD();

            fwrite((const void *)&CRC, 1, 2, fp);
            fclose(fp);
        } else {

if(LEVEL_1){
		 sprintf(logLine,"FILE:CO_OD_storage.C||"
				 "Call: CO_OD_storage_saveSecure"
				 "\n, ERROR:opening a new file and writing failed ");
		 logPrint(ERROR,logLine);}
            ret = RETURN_ERROR;
        }
    }

    /* Verify data */
if(LEVEL_1){
 sprintf(logLine,"FILE:CO_OD_storage.C||"
		 "Call: CO_OD_storage_saveSecure"
		 "\n, msg:veirfy the data written into the new file begins ");
 logPrint(LOG,logLine);}


    if(ret == RETURN_SUCCESS) {
        void *buf = NULL;
        FILE *fp = NULL;
        uint32_t cnt = 0;
        uint16_t CRC2 = 0;

        buf = malloc(odSize + 4);
        if(buf != NULL) {
            fp = fopen(filename, "r");
            if(fp != NULL) {
                cnt = fread(buf, 1, odSize, fp);
                CRC2 = crc16_ccitt((unsigned char*)buf, odSize, 0);
                /* read also two bytes of CRC */
                cnt += fread(buf, 1, 4, fp);
                fclose(fp);
            }
            free(buf);
        }
        /* If size or CRC differs, report error */
        if(buf == NULL || fp == NULL || cnt != (odSize + 2) || CRC != CRC2) {

if(LEVEL_1){
		 sprintf(logLine,"FILE:CO_OD_storage.C||"
				 "Call: CO_OD_storage_saveSecure"
				 "\n, ERROR:verification of data written to new file failed ");
		 logPrint(ERROR,logLine);}

            ret = RETURN_ERROR;
        }
    }

    /* In case of error, set back the old file. */
    if(ret != RETURN_SUCCESS && filename_old != NULL) {
        remove(filename);
        rename(filename_old, filename);
    }

    free(filename_old);

    return ret;
}


/******************************************************************************/
int CO_OD_storage_restoreSecure(char *filename) {

	if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_OD_storage.C||"
					 "Call: CO_OD_storage_restoreSecure"
					 "\n, msg:start ");
			 logPrint(LOG,logLine);}


    int ret = RETURN_SUCCESS;
    FILE *fp = NULL;

    /* If filename already exists, rename it to '.old'. */
    fp = fopen(filename, "r");

    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_OD_storage.C||"
    					 "Call: CO_OD_storage_restoreSecure"
    					 "\n, msg:check if the file pointer is null ");
    			 logPrint(LOG,logLine);}



    if(fp != NULL) {
        char *filename_old = NULL;

        fclose(fp);

        filename_old = malloc(strlen(filename)+10);

        if(LEVEL_1){
            			 sprintf(logLine,"FILE:CO_OD_storage.C||"
            					 "Call: CO_OD_storage_restoreSecure"
            					 "\n, msg:check if the filename_old is null ");
            			 logPrint(LOG,logLine);}
        if(filename_old != NULL) {
            strcpy(filename_old, filename);
            strcat(filename_old, ".old");

            remove(filename_old);
            if(rename(filename, filename_old) != 0) {
                ret = RETURN_ERROR;
            }
            free(filename_old);
        }
        else {
        	if(LEVEL_1){
        	    			 sprintf(logLine,"FILE:CO_OD_storage.C||"
        	    					 "Call: CO_OD_storage_restoreSecure"
        	    					 "\n, ERROR:filename_old is null ");
        	    			 logPrint(ERROR,logLine);}

            ret = RETURN_ERROR;
        }
    }

    /* create an empty file and write "-\n" to it. */
    if(LEVEL_1){
        			 sprintf(logLine,"FILE:CO_OD_storage.C||"
        					 "Call: CO_OD_storage_restoreSecure"
        					 "\n, msg:begin creation of an empty file and start writing ");
        			 logPrint(LOG,logLine);}

    if(ret == RETURN_SUCCESS) {
        fp = fopen(filename, "w");
        if(fp != NULL) {
            fputs("-\n", fp);
            fclose(fp);
        } else {

        	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_OD_storage.C||"
								 "Call: CO_OD_storage_restoreSecure"
								 "\n, ERROR:filename_old is null ");
						 logPrint(ERROR,logLine);}

            ret = RETURN_ERROR;
        }
    }

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_OD_storage_init(
        CO_OD_storage_t        *odStor,
        uint8_t                *odAddress,
        uint32_t                odSize,
        char                   *filename)
{
	if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_OD_storage.C||"
					 "Call: CO_OD_storage_init"
					 "\n, msg:start ");
			 logPrint(LOG,logLine);}


    CO_ReturnError_t ret = CO_ERROR_NO;
    void *buf = NULL;

    /* verify arguments */

    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_OD_storage.C||"
    					 "Call: CO_OD_storage_init"
    					 "\n, msg:check if odStor and odAddress are null ");
    			 logPrint(LOG,logLine);}


    if(odStor==NULL || odAddress==NULL) {



    	if(LEVEL_1){
    				 sprintf(logLine,"FILE:CO_OD_storage.C||"
    						 "Call: CO_OD_storage_init"
    						 "\n, ERROR: Arguments are illegal");
    				 logPrint(ERROR,logLine);}


        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* configure object variables and allocate buffer */

    if(LEVEL_1){
    			 sprintf(logLine,"FILE:CO_OD_storage.C||"
    					 "Call: CO_OD_storage_init"
    					 "\n, msg:begin configure object variables and allocate buffer ");
    			 logPrint(LOG,logLine);}




    if(ret == CO_ERROR_NO) {
        odStor->odAddress = odAddress;
        odStor->odSize = odSize;
        odStor->filename = filename;
        odStor->fp = NULL;
        odStor->tmr1msPrev = 0;
        odStor->lastSavedMs = 0;

        buf = malloc(odStor->odSize);
        if(buf == NULL) {

        	if(LEVEL_1){
						 sprintf(logLine,"FILE:CO_OD_storage.C||"
								 "Call: CO_OD_storage_init"
								 "\n, ERROR:Buffer allocated is null, out of memory");
						 logPrint(ERROR,logLine);}

            ret = CO_ERROR_OUT_OF_MEMORY;
        }
    }

    if(LEVEL_1){
        			 sprintf(logLine,"FILE:CO_OD_storage.C||"
        					 "Call: CO_OD_storage_init"
        					 "\n, msg:read data from the file and verify CRC ");
        			 logPrint(LOG,logLine);}

    /* read data from the file and verify CRC */
    if(ret == CO_ERROR_NO) {
        FILE *fp;
        uint32_t cnt = 0;
        uint16_t CRC[2];

        fp = fopen(odStor->filename, "r");
        if(fp) {
            cnt = fread(buf, 1, odStor->odSize, fp);
            /* read also two bytes of CRC from file */
            cnt += fread(&CRC[0], 1, 4, fp);
            CRC[1] = crc16_ccitt((unsigned char*)buf, odStor->odSize, 0);
            fclose(fp);
        }

        if(cnt == 2 && *((char*)buf) == '-') {
            /* file is empty, default values will be used, no error */
            if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_OD_storage.C||"
							 "Call: CO_OD_storage_init"
							 "\n, ERROR:empty file ");
					 logPrint(ERROR,logLine);}

            ret = CO_ERROR_NO;
        }
        else if(cnt != (odStor->odSize + 2)) {
            /* file length does not match */
        	if(LEVEL_1){
        						 sprintf(logLine,"FILE:CO_OD_storage.C||"
        								 "Call: CO_OD_storage_init"
        								 "\n, ERROR:file length does not match  ");
        						 logPrint(ERROR,logLine);}

            ret = CO_ERROR_DATA_CORRUPT;
        }
        else if(CRC[0] != CRC[1]) {
            /* CRC does not match */
           	if(LEVEL_1){
            						 sprintf(logLine,"FILE:CO_OD_storage.C||"
            								 "Call: CO_OD_storage_init"
            								 "\n, ERROR:CRC does not match  ");
            						 logPrint(ERROR,logLine);}

            ret = CO_ERROR_CRC;
        }
        else {
            /* no errors, copy data into Object dictionary */
            memcpy(odStor->odAddress, buf, odStor->odSize);
        }
    }

    free(buf);

    return ret;
}


/******************************************************************************/
CO_ReturnError_t CO_OD_storage_autoSave(
        CO_OD_storage_t        *odStor,
        uint16_t                timer1ms,
        uint16_t                delay)
{


	if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_OD_storage.C||"
						 "Call: CO_OD_storage_autoSave"
						 "\n, msg:start ");
				 logPrint(LOG,logLine);}

    CO_ReturnError_t ret = CO_ERROR_NO;

    /* verify arguments */
    if(LEVEL_1){
       			 sprintf(logLine,"FILE:CO_OD_storage.C||"
       					 "Call: CO_OD_storage_autoSave"
       					 "\n, msg:check if odStor and odAddress are null ");
       			 logPrint(LOG,logLine);}



    if(odStor==NULL || odStor->odAddress==NULL) {

    	if(LEVEL_1){
    	    				 sprintf(logLine,"FILE:CO_OD_storage.C||"
    	    						 "Call: CO_OD_storage_autoSave"
    	    						 "\n, ERROR: Arguments are illegal");
    	    				 logPrint(ERROR,logLine);}


        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* don't save file more often than delay */
    if(LEVEL_1){
         			 sprintf(logLine,"FILE:CO_OD_storage.C||"
         					 "Call: CO_OD_storage_autoSave"
         					 "\n, msg:check if last save is moe than delay ");
         			 logPrint(LOG,logLine);}


    if(odStor->lastSavedMs < delay) {
        odStor->lastSavedMs += timer1ms - odStor->tmr1msPrev;
    }
    else {
        void *buf = NULL;
        bool_t saveData = false;

        /* allocate buffer and open file if necessary */
        if(LEVEL_1){
           			 sprintf(logLine,"FILE:CO_OD_storage.C||"
           					 "Call: CO_OD_storage_autoSave"
           					 "\n, msg:begin configure object variables and allocate buffer ");
           			 logPrint(LOG,logLine);}



        if(ret == CO_ERROR_NO) {
            buf = malloc(odStor->odSize);
            if(odStor->fp == NULL) {
                odStor->fp = fopen(odStor->filename, "r+");
            }
            if(buf == NULL || odStor->fp == NULL) {
            	   if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_OD_storage.C||"
							 "Call: CO_OD_storage_autoSave"
							 "\n, ERROR:out of memory");
					 logPrint(ERROR,logLine);}

                ret = CO_ERROR_OUT_OF_MEMORY;
            }
        }

        /* read data from the beginning of the file */

        if(LEVEL_1){
                  			 sprintf(logLine,"FILE:CO_OD_storage.C||"
                  					 "Call: CO_OD_storage_autoSave"
                  					 "\n, msg:begin read data from the beginning of the file ");
                  			 logPrint(LOG,logLine);}
        if(ret == CO_ERROR_NO) {
            uint32_t cnt = 0;

            rewind(odStor->fp);
            cnt = fread(buf, 1, odStor->odSize, odStor->fp);

            if(cnt == 2 && *((char*)buf) == '-') {
                /* file is empty, data will be saved. */
                saveData = true;
            }
            else if(cnt == odStor->odSize) {
                /* verify, if data differs */
                if(memcmp((const void *)buf, (const void *)odStor->odAddress, odStor->odSize) != 0) {
                    saveData = true;
                }
            }
            else {
                /* file length does not match */

			  if(LEVEL_1){
					 sprintf(logLine,"FILE:CO_OD_storage.C||"
							 "Call: CO_OD_storage_autoSave"
							 "\n, ERROR:file length does not match ");
					 logPrint(LOG,logLine);}
                ret = CO_ERROR_DATA_CORRUPT;
            }
        }

        /* Save the data to the file only if data differs. */
        if(LEVEL_1){
			 sprintf(logLine,"FILE:CO_OD_storage.C||"
					 "Call: CO_OD_storage_autoSave"
					 "\n,msg:save data to the file only if the data differs ");
			 logPrint(LOG,logLine);}
        if(ret == CO_ERROR_NO && saveData) {
            uint16_t CRC;

            /* copy data to temporary buffer */
            memcpy(buf, odStor->odAddress, odStor->odSize);

            rewind(odStor->fp);
            fwrite((const void *)buf, 1, odStor->odSize, odStor->fp);

            /* write also CRC */
            CRC = crc16_ccitt((unsigned char*)buf, odStor->odSize, 0);
            fwrite((const void *)&CRC, 1, 2, odStor->fp);

            fflush(odStor->fp);

            odStor->lastSavedMs = 0;
        }

        free(buf);
    }

    odStor->tmr1msPrev = timer1ms;

    return ret;
}

void CO_OD_storage_autoSaveClose(CO_OD_storage_t *odStor) {
	 if(LEVEL_1){
				 sprintf(logLine,"FILE:CO_OD_storage.C||"
						 "Call: CO_OD_storage_autoSaveClose"
						 "\n,msg:start ");
				 logPrint(LOG,logLine);}

    if(odStor->fp != NULL) {
        fclose(odStor->fp);
    }
}
