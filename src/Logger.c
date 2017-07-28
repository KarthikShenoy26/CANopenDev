/*
 * Logger.c
 *
 *  Created on: Jul 9, 2017
 *      Author: karsh
 */

#include "Logger.h"


//****************************
//Global variables
//****************************
FILE * allLog;
FILE * errLog;
//****************************
int startLogger()
{
	if(DEBUG_LEVEL>0)
	{

	    time_t rawtime;
	    char logFilePath [255];
	    char errorFilePath [255];

	    //getting current timestamp for naming the log.
	    time (&rawtime);

	    //append location where we need to create the log file.
		sprintf(logFilePath,"/home/log_%d",(int)rawtime);
		sprintf(errorFilePath,"/home/Error_%d",(int)rawtime);

		//append .log extension
		strcat(logFilePath,".log");
		strcat(errorFilePath,".log");

		//pointer to the buffer containing name of the log file to be created.
		char *bufferPtr = logFilePath;
        char *errorLogPtr=errorFilePath;

        //remove ':' from the timestamp string and replace it with '_'
		for (; *bufferPtr; ++bufferPtr)
		{
		    if (*bufferPtr == ' ')
		     *bufferPtr = '_';

		    if (*bufferPtr==':')
		   	 *bufferPtr='_';
		}

		for (; *errorLogPtr; ++errorLogPtr)
		{
		    if (*errorLogPtr == ' ')
		    	*errorLogPtr = '_';

		    if (*errorLogPtr==':')
		    	*errorLogPtr='_';
		}

		//Printing log file path
		printf("\n log file path=%s\n",logFilePath);
		//Printing Errorlog file path
		printf("\n Error log file path=%s\n",errorFilePath);

		//Create and open the file with write access.
		allLog = fopen (logFilePath, "w+");
		errLog = fopen (errorFilePath, "w+");
		//check whether file is created or not
		if(allLog==NULL)
		{
			//if file not created. stop execution of the program. With a message
			perror("Logger.h : StartLogger : ## Cannot create Log file. check log path!!!!!");
			return -1;
		}
		if(errLog==NULL)
		{
			//if file not created. stop execution of the program. With a message
			perror("Logger.h : StartLogger : ## Cannot create Error Log file. check log path!!!!!");
			return -1;
		}
	}

	if(PRINT_DEBUG==PRINT_ON_CONSOLE) printf("\n Logger print on CONSOLE only");
	if(PRINT_DEBUG==PRINT_ON_LOGFILE)fprintf(allLog,"\n Logger print on LOG file only\n");
	if(PRINT_DEBUG==PRINT_ON_BOTH) {  printf("\n Logger print on CONSOLE and LOG \n");
									  fprintf(allLog,"\n Logger print on CONSOLE and LOG \n");
									}

	    return 1;
}

//****************************
int stopLogger()
{
	if(fclose(allLog)>0)
	{
		perror("Logger.h: stopLogger : ## cannot close the log file.");
		return -1;

	}
	if(fclose(errLog)>0)
	{
		perror("Logger.h: stopLogger : ## cannot close the log file.");
		return -1;

	}
	return 1;
}

//****************************
void logPrint(int logid,char* logLine)
{
	switch(logid)
	{
	case LOG:
			if(PRINT_DEBUG==PRINT_ON_CONSOLE) printf("%s\n\n",logLine);
			if(PRINT_DEBUG==PRINT_ON_LOGFILE)fprintf(allLog,"%s\n\n",(char*)logLine);
			if(PRINT_DEBUG==PRINT_ON_BOTH)
			{
				printf("%s\n\n",logLine);
				fprintf(allLog,"%s\n\n",(char*)logLine);
			}
			break;
	case ERROR:
		   //print error in error file and well as log file.
			if(PRINT_DEBUG==PRINT_ON_CONSOLE)
				{

					printf("ERROR:\n%s\n\n",logLine);
				}
			if(PRINT_DEBUG==PRINT_ON_LOGFILE)
				{
					fprintf(allLog,"ERROR:\n%s\n\n",(char*)logLine);
					fprintf(errLog,"ERROR:\n%s\n\n",(char*)logLine);
				}
			if(PRINT_DEBUG==PRINT_ON_BOTH)
			{
				printf("ERROR:\n%s\n\n",logLine);
				fprintf(allLog,"ERROR:\n%s\n\n",(char*)logLine);
				fprintf(errLog,"ERROR:\n%s\n\n",(char*)logLine);
			}
			break;
	default:
			//print on console by default
			printf("%s\n\n",logLine);
		break;
	}
}
//****************************

