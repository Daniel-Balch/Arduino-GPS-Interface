// Daniel Balch
// ECE 387 C (Professor Yamuna Rajasekhar)
// Midterm Chip Interface Assignment (GPS Reader/Display)
// 3-11-2016

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
//#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include <util/setbaud.h>
#include <ctype.h>

// Packet type codes
#define PC_TEST "000"                 // test/dummy command
#define PRX_ACK "001"                 // acknowledge response
#define PRX_SYS_MSG "010"             // system message
#define PRX_TEXT_MSG "011"            // system text message
#define PC_HOT_RESTART "101"          // hot restart command
#define PC_WARM_RESTART "102"         // warm restart command
#define PC_COLD_RESTART "103"         // cold restart command
#define PC_FULL_COLD_RESTART "104"    // full cold restart command
#define PSET_ECHO_UPDATE "220"        // NMEA echo update rate setting command
#define PSET_BAUD "251"               // NMEA baud rate setting command
#define PSET_SBAS "313"               // SBAS enable/disable setting command
#define PQ_SBAS "413"                 // SBAS setting query
#define PRX_SBAS "513"                // SBAS response (from query)
#define PSET_OUTPUTS "314"            // NMEA sentence data fields frequency setting command
#define PQ_OUTPUTS "414"              // NMEA sentence data fields frequency setting query
#define PRX_OUTPUTS "514"             // NMEA sentence data fields frequency setting response (from query)
#define PSET_SPEED_THRESHOLD "386"    // Navigation speed change threshold setting command
#define PQ_SPEED_THRESHOLD "447"      // Navigation speed change threshold setting query
#define PRX_SPEED_THRESHOLD "527"     // Navigation speed change threshold setting response (from query)
#define PSET_AIC "286"                // Enable/disable AIC mode setting command
#define PSET_QZSS_FORMAT "351"        // Enable/disable QZSS NMEA format setting command
#define PSET_QZSS_SVC "352"           // Enable/disable QZSS service setting command

// Common GPS Packet Phrases
#define P_TKR_ID "PMTK"               // Packet Talker ID (phrase found in each packet after preamble)
#define DATA_TAG "GPRMC"             // Preamble of RMC (Recommended Minimum Coordinates for Navigation) format data sentence

// GPS Operational Parameters
#define GPS_BAUD "9600"               // GPS Data rate
#define ECHO_REF_INTERVAL "200"       // NMEA Echo Interval in milliseconds
#define NMEA_OUTPUT_FIELDS "0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"   // NMEA outputs selection
#define NAV_SPD_THRESHOLD "0.2"       // Navigation Speed Change Threshold in m/s
#define NAV_SPD_THRESHOLD_B "0.20"    // Navigation Speed Change Threshold in m/s (for query response only)
#define ENABLE_QZSS_FORMAT '0'        // Disable QZSS format in NMEA output
#define ENABLE_QZSS_SVC '1'           // Enable QZSS service
#define ENABLE_SBAS '1'               // Enable SBAS feature
#define ENABLE_AIC '1'                // Enable AIC feature

// Display-related commands and parameters
#define BIT_INTERVAL 167        // interval in microseconds
#define ALLZERO 0b00000000
#define INIT_A 0b00011100
#define INIT_B 0b00010100
#define SET_DISP_FORMAT_A 0b00010100
#define SET_DISP_FORMAT_B 0b00010001
#define DISP_OFF_A 0b00010000
#define DISP_OFF_B 0b00010001
#define CLR_DISP_A 0b00010000
#define CLR_DISP_B 0b00011000
#define SET_ENTRY_MODE_A 0b00010000
#define SET_ENTRY_MODE_B 0b00010010
#define DISP_ON_A 0b00010000
#define DISP_ON_B 0b00010011
#define NEW_LINE_A 0b00010101
#define NEW_LINE_B 0b00010001
#define STARTUP_DELAY_A 65

// Program Operational Parameters
#define RX_BUFFER_CAP 260             // Size of input stream buffer (max. packet size + 5) in bytes
#define TX_BUFFER_CAP 260             // Size of output buffer (max. packet size + 5) in bytes
#define DISP_BUFFER_CAP 34            // Size of display output buffer in bytes
#define INIT_PACKET_INTERVAL 50       // Initialization (test and settings) packet time interval in milliseconds
#define INIT_ATTEMPT_THRESHOLD 15     // Maximum number of tries to initialize a variable before restarting
#define RESTART_DELAY_LENGTH 1000     // Time to wait after restarting GPS module in milliseconds
#define UPDATE_INTERVAL 350           // Time to wait between updates for info and display in milliseconds
#define DATA_ATTEMPTS_THRESHOLD 35    // Number of data retrieval attempts before lighting up warning LED
#define NO_SIG_MSG "Signal Strength\nToo Low"     // Message to display if GPS info updates unsuccessful
#define INIT_MSG "Initializing...\nStage "  // Part of message to display when GPS is being started

// function prototypes
static int buildRestartPacket(char *outBuffer, int errorLevel);
static int getChecksum(char *sum, char *outBuffer);
static int buildInitPacket(char *outBuffer, int progStage, char query);
static int findInputPacket(char *inBuffer, int progStage, char query);
static char checkResponseCorrect(char *inBuffer, int responseIndex, int progStage, char query);
static char checkDataValidity(char *inBuffer, int responseIndex);
static int initDisplay(void);
static int sendDisplayBytes(char byteA, char byteB);
static int parseInfo(char *inBuffer, int responseIndex, char *latLong, char *timeDate, char *speedCrs);
static int updateDisplay(char *dispBuffer, char *message, int progStage);
static int initTimer1(void);
static int initUSART(void);
static uint8_t receiveByte(void);
static int transmitByte(uint8_t data);
static int sendPacket(char *outBuffer);
static int readInput(char *inBuffer);

int main(void)
{
   initDisplay();
   //DDRD = 0b00000010;
   initUSART();
   initTimer1();
   int progStage = 0;
   int initAttempts = 0;
   int errorLevel = 0;
   char query = '1';
   char *inBuffer = (char * ) malloc(RX_BUFFER_CAP);
   char *outBuffer = (char * ) malloc(TX_BUFFER_CAP);
   char *dispBuffer = (char * ) malloc(DISP_BUFFER_CAP);
   char *latLong = (char * ) malloc(34);
   char *timeDate = (char * ) malloc(34);
   char *speedCrs = (char * ) malloc(34);
   int responseIndex = -1;
   int displayMode = 1;
   int dataAttempts = 0;
   updateDisplay(dispBuffer,INIT_MSG,1);
   while (progStage < 9) {
      char validForQuery = '0';
      if ((progStage == 2) || (progStage == 4) || (progStage == 7)) {
         validForQuery = '1';
      }   
      if (initAttempts == 0) {
         if (errorLevel > 0) {
            buildRestartPacket(outBuffer, errorLevel);
            sendPacket(outBuffer);
            if (errorLevel == 4) {
               errorLevel = 0;
            }   
            _delay_ms(RESTART_DELAY_LENGTH);
         }   
         buildInitPacket(outBuffer, progStage, query);
      }
      sendPacket(outBuffer);
      readInput(inBuffer);
      responseIndex = findInputPacket(inBuffer, progStage, query);
      if (responseIndex >= 0) {
         char desiredResponse = checkResponseCorrect(inBuffer, responseIndex, progStage, query);
         if (desiredResponse == '1') {
            initAttempts = 0;
            errorLevel = 0;
            if ((query == '0') && (validForQuery == '1')) {
               query = '1';
            } else if ((query == '1') && (validForQuery == '1')) {
               progStage++;
               updateDisplay(dispBuffer,INIT_MSG,(progStage+1));
            } else if ((query == '1') && (validForQuery == '0')) {
               query = '0';
            } else {
               progStage++;
               updateDisplay(dispBuffer,INIT_MSG,(progStage+1));
            }   
         } else {
            if (query == '0') {
               if (initAttempts < INIT_ATTEMPT_THRESHOLD) {
                  initAttempts++;
               } else {
                  initAttempts = 0;
                  errorLevel++;
                  if (errorLevel >= 4) {
                     progStage = 0;
                     query = '1';
                  }   
               }
            } else {
               initAttempts = 0;
               errorLevel = 0;
               query = '0';
            }
         }
      } else {
         if (initAttempts < INIT_ATTEMPT_THRESHOLD) {
            initAttempts++;
         } else {
            initAttempts = 0;
            errorLevel++;
            if (errorLevel >= 4) {
               progStage = 0;
               query = '1';
            }
         }
      }
   }
   while (1) {
      readInput(inBuffer);
      responseIndex = findInputPacket(inBuffer, progStage, 'f');
      char valid = checkDataValidity(inBuffer,responseIndex);
      if ((responseIndex >= 0) && (valid == '1')) {
         dataAttempts = 0;
         parseInfo(inBuffer, responseIndex, latLong, timeDate, speedCrs);
         switch (displayMode) {
            case 1:
               updateDisplay(dispBuffer,latLong, -1);
               break;
            case 2:
               updateDisplay(dispBuffer,timeDate, -1);
               break;
            case 3:
               updateDisplay(dispBuffer,speedCrs, -1);
               break;
            default:
               updateDisplay(dispBuffer,latLong, -1);
               break;
         }
      } else {
         if (dataAttempts >= DATA_ATTEMPTS_THRESHOLD) {
            updateDisplay(dispBuffer,NO_SIG_MSG, -1);
         } else {
            dataAttempts++;
         }
      }   
   }
   return 0;
}

int buildRestartPacket(char *outBuffer, int errorLevel)
{
   if (errorLevel <= 1) {
      sprintf(outBuffer,"%s%s",P_TKR_ID,PC_HOT_RESTART);
   } else if (errorLevel == 2) {
      sprintf(outBuffer,"%s%s",P_TKR_ID,PC_WARM_RESTART);
   } else if (errorLevel == 3) {
      sprintf(outBuffer,"%s%s",P_TKR_ID,PC_COLD_RESTART);
   } else {
      sprintf(outBuffer,"%s%s",P_TKR_ID,PC_FULL_COLD_RESTART);
   }
   char *checksum = (char * ) malloc(4);
   getChecksum(checksum, outBuffer);
   char cr = ((char) 13);
   char lf = ((char) 10);
   char *tempBuffer = (char * ) malloc((strlen(outBuffer))+(strlen(checksum))+5);
   sprintf(tempBuffer,"$%s*%s%c%c",outBuffer,checksum,cr,lf);
   strcpy(outBuffer,tempBuffer);
   free(tempBuffer);
   free(checksum);
   return 0;
}

int getChecksum(char *sum, char *outBuffer)
{
   int c = 0;
   while(*outBuffer) {
      c ^= *outBuffer++;
   }
   sprintf(sum,"%x",c);
   int i = 0;
   while (sum[i]) {
      sum[i] = toupper(sum[i]);
      i++;
   }   
   return 0;
}

int buildInitPacket(char *outBuffer, int progStage, char query)
{
   char go = '1';
   switch (progStage) {
      case 0:
         if (query == '0') {
            sprintf(outBuffer,"%s%s",P_TKR_ID,PC_TEST);
         } else {
            go = '0';
         }   
         break;
      case 1:
         if (query == '0') {
            sprintf(outBuffer,"%s%s,%s",P_TKR_ID,PSET_BAUD,GPS_BAUD);
         } else {
            go = '0';
         }    
         break;
      case 2:
         if (query == '1') {
            sprintf(outBuffer,"%s%s",P_TKR_ID,PQ_OUTPUTS);
         } else {
            sprintf(outBuffer,"%s%s,%s",P_TKR_ID,PSET_OUTPUTS,NMEA_OUTPUT_FIELDS);
         }
         break;
      case 3:
         if (query == '0') {
            sprintf(outBuffer,"%s%s,%s",P_TKR_ID,PSET_ECHO_UPDATE,ECHO_REF_INTERVAL);
         } else {
            go = '0';
         }      
         break;
      case 4:
         if (query == '1') {
            sprintf(outBuffer,"%s%s",P_TKR_ID,PQ_SPEED_THRESHOLD);
         } else {
            sprintf(outBuffer,"%s%s,%s",P_TKR_ID,PSET_SPEED_THRESHOLD,NAV_SPD_THRESHOLD);
         }
         break;
      case 5:
         if (query == '0') {
            sprintf(outBuffer,"%s%s,%c",P_TKR_ID,PSET_QZSS_FORMAT,ENABLE_QZSS_FORMAT);
         } else {
            go = '0';
         }      
         break;
      case 6:
         if (query == '0') {
            sprintf(outBuffer,"%s%s,%c",P_TKR_ID,PSET_QZSS_SVC,ENABLE_QZSS_SVC);
         } else {
            go = '0';
         }      
         break;
      case 7:
         if (query == '1') {
            sprintf(outBuffer,"%s%s",P_TKR_ID,PQ_SBAS);
         } else {
            sprintf(outBuffer,"%s%s,%c",P_TKR_ID,PSET_SBAS,ENABLE_SBAS);
         }
         break;
      case 8:
         if (query == '0') {
            sprintf(outBuffer,"%s%s,%c",P_TKR_ID,PSET_AIC,ENABLE_AIC);
         } else {
            go = '0';
         }      
         break;
   }
   if (go == '1') {
      char *checksum = (char * ) malloc(4);
      getChecksum(checksum, outBuffer);
      char cr = ((char) 13);
      char lf = ((char) 10);
      char *tempBuffer = (char * ) malloc(((strlen(outBuffer)+strlen(checksum)+5)));
      sprintf(tempBuffer,"$%s*%s%c%c",outBuffer,checksum,cr,lf);
      strcpy(outBuffer, tempBuffer);
      free(tempBuffer);
      free(checksum);
   }
   return 0;
}

int findInputPacket(char *inBuffer, int progStage, char query)
{
   char go = '1';
   char *expectedPhrase = (char * ) malloc(17);
   char *sample = (char * ) malloc(17);
   switch (progStage) {
      case 0:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PC_TEST);
         } else {
            go = '0';
         }
         break;
      case 1:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_BAUD);
         } else {
            go = '0';
         }
         break;
      case 2:
         if (query == '1') {
            sprintf(expectedPhrase,"$%s%s",P_TKR_ID,PRX_OUTPUTS);
         } else {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_OUTPUTS);
         }
         break;
      case 3:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_ECHO_UPDATE);
         } else {
            go = '0';
         }
         break;
      case 4:
         if (query == '1') {
            sprintf(expectedPhrase,"$%s%s",P_TKR_ID,PRX_SPEED_THRESHOLD);
         } else {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_SPEED_THRESHOLD);
         }
         break;
      case 5:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_QZSS_FORMAT);
         } else {
            go = '0';
         }
         break;
      case 6:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_QZSS_SVC);
         } else {
            go = '0';
         }
         break;
      case 7:
         if (query == '1') {
            sprintf(expectedPhrase,"$%s%s",P_TKR_ID,PRX_SBAS);
         } else {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_SBAS);
         }
         break;
      case 8:
         if (query == '0') {
            sprintf(expectedPhrase,"$%s%s,%s",P_TKR_ID,PRX_ACK,PSET_AIC);
         } else {
            go = '0';
         }
         break;
      default:
         sprintf(expectedPhrase,"$%s",DATA_TAG);
         break;
   }
   if ((go == '1') && (strstr(inBuffer, expectedPhrase) != NULL)) {
      int i = 0;
      char finished = '0';
      while ((finished == '0') && (i < ((strlen(inBuffer)) - (strlen(sample)) + 1))) {
         strncpy(sample,(inBuffer+i),(strlen(sample)));
         if (strcmp(sample,expectedPhrase) == 0) {
            finished = '1';
            break;
         }
         i++;
      }
      free(expectedPhrase);
      free(sample);
      if (progStage < 9) {
         return i;
      } else {
         char statusFound = '0';
         char dataStatus = '0';
         int j = i;
         int commaCount = 0;
         char cr = ((char) 13);
         char lf = ((char) 10);
         while ((statusFound == '0') && (j < ((strlen(inBuffer)) - 1))) {
            char current = inBuffer[j];
            char next = inBuffer[(j+1)];
            if (current == ',') {
               commaCount++;
               if (commaCount == 2) {
                  dataStatus = next;
                  statusFound = '1';
                  break;
               }   
            } else if ((current == cr) && (next == lf)) {
               statusFound = '1';
               break;
            }
            j++;
         }
         if ((statusFound == '1') && (dataStatus == 'A')) {
            return i;
         } else {
            return -1;
         }
      }         
   } else if (go == '0') {
      free(expectedPhrase);
      free(sample);      
      return 0;
   } else {
      free(expectedPhrase);
      free(sample);      
      return -1;
   }
}

char checkResponseCorrect(char *inBuffer, int responseIndex, int progStage, char query)
{
   char *expectedPacket = (char * ) malloc(47);
   char go = '1';
   switch (progStage) {
      case 0:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PC_TEST);
         } else {
            go = '0';
         }
         break;
      case 1:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_BAUD);
         } else {
            go = '0';
         }
         break;
      case 2:
         if (query == '1') {
            sprintf(expectedPacket,"%s%s,%s",P_TKR_ID,PRX_OUTPUTS,NMEA_OUTPUT_FIELDS);
         } else {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_OUTPUTS);
         }
         break;
      case 3:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_ECHO_UPDATE);
         } else {
            go = '0';
         }
         break;
      case 4:
         if (query == '1') {
            sprintf(expectedPacket,"%s%s,%s",P_TKR_ID,PRX_SPEED_THRESHOLD,NAV_SPD_THRESHOLD_B);
         } else {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_SPEED_THRESHOLD);
         }
         break;
      case 5:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_QZSS_FORMAT);
         } else {
            go = '0';
         }
         break;
      case 6:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_QZSS_SVC);
         } else {
            go = '0';
         }
         break;
      case 7:
         if (query == '1') {
            sprintf(expectedPacket,"%s%s,%c",P_TKR_ID,PRX_SBAS,ENABLE_SBAS);
         } else {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_SBAS);
         }
         break;
      case 8:
         if (query == '0') {
            sprintf(expectedPacket,"%s%s,%s,3",P_TKR_ID,PRX_ACK,PSET_AIC);
         } else {
            go = '0';
         }
         break;
   }
   if (go == '1') {
      char *testPacket = (char * ) malloc(47);
      char *checksum = (char * ) malloc(4);
      getChecksum(checksum,expectedPacket);
      char cr = ((char) 13);
      char lf = ((char) 10);
      char *tempPacket = (char * ) malloc(((strlen(expectedPacket))+(strlen(checksum))+5));
      sprintf(tempPacket,"$%s*%s%c%c",expectedPacket,checksum,cr,lf);
      strcpy(expectedPacket,tempPacket);
      free(tempPacket);
      int fullEnd = -1;
      int i = (responseIndex + 1);
      while ((fullEnd < 0) && (i < ((strlen(inBuffer)) - 1))) {
         char current = inBuffer[i];
         char next = inBuffer[(i+1)];
         if ((current == cr) && (next == lf)) {
            fullEnd = next;
            break;
         }
         i++;
      }
      strncpy(testPacket,(inBuffer + responseIndex), (fullEnd - responseIndex + 1));
      free(checksum);
      if (strcmp(testPacket,expectedPacket) == 0) {
         free(testPacket);
         free(expectedPacket);
         return '1';
      } else {
         free(testPacket);
         free(expectedPacket);
         return '0';
      }
   } else {
      free(expectedPacket);
      return '1';
   }
}

char checkDataValidity(char *inBuffer, int responseIndex)
{
   int endIndex = -1;
   int i = (responseIndex + 1);
   char endFound = '0';
   char cr = ((char) 13);
   char lf = ((char) 10);
   char *testChecksum = (char * ) malloc(4);
   while ((endFound == '0') && i < ((strlen(inBuffer)) - 2)) {
      char current = inBuffer[i];
      char next = inBuffer[(i+1)];
      char nextB = inBuffer[(i+2)];
      if (current == '*') {
         endIndex = (current - 1);
         sprintf(testChecksum,"%c%c",next,nextB);
         endFound = '1';
         break;
      } else if ((next == cr) && (nextB == lf)) {
         endFound = '1';
         break;
      }
      i++;
   }
   if ((endFound == '1') && (endIndex > responseIndex)) {
      char *expectedChecksum = (char * ) malloc(4);
      char *packetBody = (char * ) malloc((endIndex - responseIndex + 3));
      strncpy(packetBody,(inBuffer + responseIndex + 1), (endIndex - responseIndex));
      getChecksum(expectedChecksum,packetBody);
      if (strcmp(expectedChecksum,testChecksum) == 0) {
         free(testChecksum);
         free(expectedChecksum);
         free(packetBody);
         return '1';
      }
      free(expectedChecksum);
      free(packetBody);
   }
   free(testChecksum);
   return '0';
}

int initDisplay()
{
   DDRB = 0b00111111;
   PORTB = ALLZERO;
   _delay_ms(STARTUP_DELAY_A);
   int i;
   char byte = ((char) ALLZERO);
   for (i = 0; i < 13; i++) {
      switch (i) {
         case 0:
            byte = ((char) INIT_A);
            break;
         case 1:   
            byte = ((char) INIT_A);
            break;
         case 2:   
            byte = ((char) INIT_A);
            break;
         case 3:   
            byte = ((char) INIT_B);
            break;
         case 4:   
            byte = ((char) SET_DISP_FORMAT_A);
            break;
         case 5:   
            byte = ((char) SET_DISP_FORMAT_B);
            break;
         case 6:   
            byte = ((char) DISP_OFF_A);
            break;
         case 7:   
            byte = ((char) DISP_OFF_B);
            break;
         case 8:   
            sendDisplayBytes(CLR_DISP_A,CLR_DISP_B);
            break;
         case 9:   
            byte = ((char) SET_ENTRY_MODE_A);
            break;
         case 10:   
            byte = ((char) SET_ENTRY_MODE_B);
            break;
         case 11:   
            byte = ((char) DISP_ON_A);
            break;
         case 12:   
            byte = ((char) DISP_ON_B);
            break;
      }
      if (i != 8) {
         PORTB = byte;
         _delay_us(BIT_INTERVAL);
         PORTB = ALLZERO;
         _delay_us(BIT_INTERVAL);
      }
      if (i == 0) {
         _delay_ms(10);
      } else if (i == 1) {
         _delay_us(150);
      } else if (i == 10) {
         _delay_ms(50);
      }
   }
   _delay_us(100);
   return 0;   
}

int sendDisplayBytes(char byteA, char byteB)
{
   PORTB = byteA;
   _delay_us(BIT_INTERVAL);
   PORTB = ALLZERO;
   _delay_us(BIT_INTERVAL);
   PORTB = byteB;
   _delay_us(BIT_INTERVAL);
   PORTB = ALLZERO;
   _delay_us(BIT_INTERVAL);
   return 0;
}

int parseInfo(char *inBuffer, int responseIndex, char *latLong, char *timeDate, char *speedCrs)
{
   char packetFinished = '0';
   char cr = ((char) 13);
   char lf = ((char) 10);
   int commaCounter = 0;
   int commaA = -1;
   int commaB = -1;
   int commaC = -1;
   int commaD = -1;
   int commaE = -1;
   int commaF = -1;
   int i = responseIndex;
   while ((packetFinished == '0') && (i < (strlen(inBuffer)))) {
      char current = inBuffer[i];
      if ((current == cr) || (current == lf) || (current == '*')) {
         packetFinished = '1';
         break;
      } else if (current == ',') {
         commaCounter++;
         switch (commaCounter) {
            case 1:
               commaA = i;
               break;
            case 2:
               commaB = i;
               break;
            case 3:
               commaC = i;
               break;
            case 7:
               commaD = i;
               break;
            case 9:
               commaE = i;
               break;
            case 10:
               commaF = i;
               packetFinished = '1';
               break;
         }
      }
      i++;
   }
   int commasValidA = ((commaA >= 0) && (commaB > commaA));
   int commasValidB = ((commaC > commaB) && (commaD > commaC) && (commaE > commaD) && (commaF > commaE));
   if (commasValidA && commasValidB && (packetFinished == '1')) {
      char *tempA = (char * ) malloc(4);
      char *tempB = (char * ) malloc(4);
      char *tempC = (char * ) malloc(4);
      char *tempD = (char * ) malloc(5);
      char *tempE = (char * ) malloc(11);
      char *tempF = (char * ) malloc(11);
      if (((commaB-commaA) >= 7) && ((commaF-commaE) >= 7)) {
         strncpy(tempA,(inBuffer+commaA+1),2);
         strncpy(tempB,(inBuffer+commaA+3),2);
         strncpy(tempC,(inBuffer+commaA+5),2);
         strncpy(tempD,(inBuffer+commaE+1),2);
         strncpy(tempE,(inBuffer+commaE+3),2);
         strncpy(tempF,(inBuffer+commaE+5),2);
         sprintf(timeDate,"%s/%s/20%s\n%s:%s:%s UTC",tempE,tempD,tempF,tempA,tempB,tempC);
      } else {
         sprintf(timeDate,"%s","ERROR");
      }
      if ((commaD-commaC) >= 21) {
         int latEnd = 0;
         char latEndFound = '0';
         int j = (commaC + 1);
         while ((latEndFound == '0') && (j < commaD)) {
            char currentLatInd = inBuffer[j];
            if ((currentLatInd == 'N') || (currentLatInd == 'S')) {
               latEndFound = '1';
               latEnd = j;
               break;
            } else {
               j++;
            }
         }
         if (latEndFound == '1') {
            char latSign = inBuffer[latEnd];
            strncpy(tempA,(inBuffer+commaC+1),2);
            strncpy(tempE,(inBuffer+commaC+3),(latEnd-commaC-5));
            char longSign = inBuffer[(commaD-1)];
            strncpy(tempD,(inBuffer+latEnd+2),3);
            strncpy(tempF,(inBuffer+latEnd+5),(commaD-latEnd-6));
            sprintf(latLong,"%c %s° %s'\n%c %s° %s'",latSign,tempA,tempE,longSign,tempD,tempF);
         } else {
            sprintf(latLong,"%s","Error");
         }
      } else {
         sprintf(latLong,"%s","Error");
      }
      if ((commaE-commaD) >= 8) {
         char midCommaFound = '0';
         int midComma = 0;
         int k = (commaD+1);
         while ((midCommaFound == '0') && (k < commaE)) {
            char currentVel = inBuffer[k];
            if (currentVel == ',') {
               midCommaFound = '1';
               midComma = k;
               break;
            } else {
               k++;
            }
         }
         if (midCommaFound == '1') {
            strncpy(tempE,(inBuffer+commaD+1),(midComma-commaD-1));
            strncpy(tempF,(inBuffer+midComma+1),(commaE-midComma-1));
            sprintf(speedCrs,"Heading %s°\n@ %s knots",tempF,tempE);
         } else {
            sprintf(speedCrs,"%s","Error");
         }
      } else {
         sprintf(speedCrs,"%s","Error");
      }
      free(tempA);
      free(tempB);
      free(tempC);
      free(tempD);
      free(tempE);
      free(tempF);   
   } else {
      sprintf(timeDate,"%s","Error");
      sprintf(latLong,"%s","Error");
      sprintf(speedCrs,"%s","Error");
   }
   return 0;
}   

int updateDisplay(char *dispBuffer, char *message, int progStage)
{
   sendDisplayBytes(CLR_DISP_A,CLR_DISP_B);
   _delay_ms(3);
   if ((progStage > 0) && (progStage < 10)) {
      sprintf(dispBuffer,"%s%i of 9",message,progStage);
   } else {
      sprintf(dispBuffer,"%s",message);
   }   
   int length = strlen(dispBuffer);
   int i;
   char byteA = ((char) ALLZERO);
   char byteB = ((char) ALLZERO);
   for (i = 0; i < length; i++) {
      char current = dispBuffer[i];
      switch(current) {
         case '\n':
            byteA = ((char) NEW_LINE_A);
            byteB = ((char) NEW_LINE_B);
            break;
         case ' ':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110000);
            break;
         case '!':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111000);
            break;
         case '"':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110100);
            break;
         case '#':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111100);
            break;
         case '$':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110010);
            break;
         case '%':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111010);
            break;
         case '&':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110110);
            break;
         case '\'':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111110);
            break;
         case '(':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110001);
            break;
         case ')':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111001);
            break;
         case '*':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110101);
            break;
         case '+':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111101);
            break;
         case ',':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110011);
            break;
         case '-':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111011);
            break;
         case '.':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00110111);
            break;
         case '/':
            byteA = ((char) 0b00110100);
            byteB = ((char) 0b00111111);
            break;
         case '0':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110000);
            break;
         case '1':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111000);
            break;
         case '2':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110100);
            break;
         case '3':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111100);
            break;
         case '4':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110010);
            break;
         case '5':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111010);
            break;
         case '6':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110110);
            break;
         case '7':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111110);
            break;
         case '8':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110001);
            break;
         case '9':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111001);
            break;
         case ':':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110101);
            break;
         case ';':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111101);
            break;
         case '<':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110011);
            break;
         case '=':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111011);
            break;
         case '>':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00110111);
            break;
         case '?':
            byteA = ((char) 0b00111100);
            byteB = ((char) 0b00111111);
            break;
         case '@':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110000);
            break;
         case 'A':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111000);
            break;
         case 'B':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110100);
            break;
         case 'C':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111100);
            break;
         case 'D':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110010);
            break;
         case 'E':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111010);
            break;
         case 'F':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110110);
            break;
         case 'G':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111110);
            break;
         case 'H':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110001);
            break;
         case 'I':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111001);
            break;
         case 'J':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110101);
            break;
         case 'K':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111101);
            break;
         case 'L':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110011);
            break;
         case 'M':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111011);
            break;
         case 'N':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00110111);
            break;
         case 'O':
            byteA = ((char) 0b00110010);
            byteB = ((char) 0b00111111);
            break;
         case 'P':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110000);
            break;
         case 'Q':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111000);
            break;
         case 'R':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110100);
            break;
         case 'S':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111100);
            break;
         case 'T':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110010);
            break;
         case 'U':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111010);
            break;
         case 'V':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110110);
            break;
         case 'W':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111110);
            break;
         case 'X':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110001);
            break;
         case 'Y':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111001);
            break;
         case 'Z':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110101);
            break;
         case '[':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111101);
            break;
         case '\\':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110011);
            break;
         case ']':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111011);
            break;
         case '^':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00110111);
            break;
         case '_':
            byteA = ((char) 0b00111010);
            byteB = ((char) 0b00111111);
            break;
         case '`':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110000);
            break;
         case 'a':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111000);
            break;
         case 'b':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110100);
            break;
         case 'c':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111100);
            break;
         case 'd':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110010);
            break;
         case 'e':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111010);
            break;
         case 'f':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110110);
            break;
         case 'g':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111110);
            break;
         case 'h':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110001);
            break;
         case 'i':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111001);
            break;
         case 'j':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110101);
            break;
         case 'k':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111101);
            break;
         case 'l':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110011);
            break;
         case 'm':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111011);
            break;
         case 'n':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00110111);
            break;
         case 'o':
            byteA = ((char) 0b00110110);
            byteB = ((char) 0b00111111);
            break;
         case 'p':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110000);
            break;
         case 'q':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111000);
            break;
         case 'r':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110100);
            break;
         case 's':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111100);
            break;
         case 't':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110010);
            break;
         case 'u':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111010);
            break;
         case 'v':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110110);
            break;
         case 'w':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111110);
            break;
         case 'x':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110001);
            break;
         case 'y':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111001);
            break;
         case 'z':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110101);
            break;
         case '{':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111101);
            break;
         case '|':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110011);
            break;
         case '}':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00111011);
            break;
         case '~':
            byteA = ((char) 0b00111110);
            byteB = ((char) 0b00110111);
            break;
         case '­':
            byteA = ((char) 0b00110101);
            byteB = ((char) 0b00111011);
            break;
         case '°':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00110000);
            break;
         case '±':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00111000);
            break;
         case '´':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00110010);
            break;
         case 'µ':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00111010);
            break;
         case '·':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00111110);
            break;
         case '¸':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00110001);
            break;
         case 'º':
            byteA = ((char) 0b00111101);
            byteB = ((char) 0b00110101);
            break;
      }
      sendDisplayBytes(byteA,byteB);
   }
   return 0;
}

int initUSART()
{
   UBRR0H = UBRRH_VALUE;                        /* defined in setbaud.h */
   UBRR0L = UBRRL_VALUE;
   #if USE_2X
      UCSR0A |= (1 << U2X0);
   #else
      UCSR0A &= ~(1 << U2X0);
   #endif
   UCSR0B = (1 << TXEN0) | (1 << RXEN0);     // Enable USART transmitter/receiver
   UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 data bits, 1 stop bit
   return 0;
}

int initTimer1()
{
   TCCR1B |= ((1 << CS11) | (1 << CS10));
   return 0;
}   

int transmitByte(uint8_t data)
{
   loop_until_bit_is_set(UCSR0A, UDRE0);
   UDR0 = data;   /* send data */
return 0;
}

int sendPacket(char *outBuffer)
{
   int i;
   for (i = 0; i < (strlen(outBuffer)); i++) {
      uint8_t current = ((uint8_t) outBuffer[i]);
      transmitByte(current);
   }
   return 0;
}

int readInput(char *inBuffer)
{
   int i = 0;
   TCNT1 = 0;
   int timerValue = (TCNT1 >> 4);
   int lastIndex = (RX_BUFFER_CAP-1);
   while ((i < lastIndex) && (timerValue <= UPDATE_INTERVAL)) {     
      char byte = ((char) receiveByte());
      inBuffer[i] = byte;
      i++;
      timerValue = (TCNT1 >> 4);
   }
   inBuffer[lastIndex] = '\0';
   return 0;
}

uint8_t receiveByte() 
{
   loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
   return UDR0;                                /* return register value */
}