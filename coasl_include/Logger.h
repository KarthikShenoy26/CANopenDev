/*
 * Logger.h
 *
 *  Created on: Jul 9, 2017
 *      Author: karsh
 */
/*
 * USAGE:
 * 			1>   if(LEVEL_1){
 * 				 sprintf(logline,"sbb acncbdcc");
 * 			   	 logPrint(LOG,logline);}
 *
 * 			2>   if(LEVEL_1){
 * 				 sprintf(logline,"sbb acncbdcc");
 * 			     logPrint(ERROR,logline);}
 *
 */
//   if(LEVEL_1){sprintf(logline,"sbb acncbdcc");   logPrint(ERROR,logline);}




#ifndef COASL_INCLUDE_LOGGER_H_
#define COASL_INCLUDE_LOGGER_H_

#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>


#define WHERE printf("\n\n I AM HERE \n\n")
//LogIDs for logging
#define LOG 0
#define ERROR 1
//*********************************************
//These are the Log Level options.
#define LEVEL_1 DEBUG_LEVEL>0
#define LEVEL_2 DEBUG_LEVEL>1
#define LEVEL_3 DEBUG_LEVEL>2
#define LEVEL_4 DEBUG_LEVEL>4
#define LEVEL_5 DEBUG_LEVEL>5
#define LEVEL_6 DEBUG_LEVEL>6
#define LEVEL_7 DEBUG_LEVEL>7
#define LEVEL_8 DEBUG_LEVEL>8
#define LEVEL_9 DEBUG_LEVEL>9

//These are log print options
#define NO_LOG 0
#define PRINT_ON_CONSOLE 1
#define PRINT_ON_LOGFILE 2
#define PRINT_ON_BOTH 3
//**********************************************


//*********************************************
//Change setting for changing debug level

	//This is Debug print is available for level
	#define DEBUG_LEVEL 1

//Change setting for changing Print options
	//Print option is set to following
	#define PRINT_DEBUG PRINT_ON_BOTH

//*********************************************
//Global variable settings
//*********************************************
		//only single instance of logger can be created.
		extern int fileDescrpt;
		//Max of 250 characters can be printed in a message
		char logLine[250];

//*********************************************
//Function list
//*********************************************
	//******************
	//Application use
	//******************
			//Startlogger is called in beginning in main() .This sets logs for entire program execution
			int startLogger();

			//Stoplogger stops the logger.
			//This is called in the end of the main(). This disables logging.
			int stopLogger();

			//This prints the log in different files depending on the logid .
			void logPrint(int logid,char* logLine);

#endif /* COASL_INCLUDE_LOGGER_H_ */
