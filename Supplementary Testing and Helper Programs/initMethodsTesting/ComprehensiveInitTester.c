#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

// generic query response packets (need cr and lf appended to all)
#define GENERIC_SBAS_RX_PKT "$PMTK513,1*28"
#define GENERIC_OUTPUTS_RX_PKT "$PMTK514,0,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2B"
#define GENERIC_SPD_THRESHOLD_RX_PKT "$PMTK386,0.2*3F"
#define NEMA_SEQ_A "$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70"
#define NEMA_SEQ_B "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68"
#define NEMA_SEQ_C "$$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62"

static int buildRestartPacket(char *outBuffer, int errorLevel);
static int getChecksum(char *sum, char *outBuffer);
static int buildInitPacket(char *outBuffer, int progStage, char query);
static int findInputPacket(char *inBuffer, int progStage, char query);
static char checkResponseCorrect(char *inBuffer, int responseIndex, int progStage, char query);
static char checkDataValidity(char *inBuffer, int responseIndex);
static int parseInfo(char *inBuffer, int responseIndex, char *latLong, char *timeDate, char *speedCrs);
static int arbitraryInBufferCreate(char *inBuffer, char *text, int index);

int main()
{
   char cr = ((char) 13);
   char lf = ((char) 10);
   char *outBuffer = (char * ) malloc(260);
   char *inBuffer = (char * ) malloc(260);
   char *latLong = (char * ) malloc(34);
   char *timeDate = (char * ) malloc(34);
   char *speedCrs = (char * ) malloc(34);
   int i;
   printf("Restart Packet Testing:\n\n");
   for (i = 1; i <=4; i++) {
      buildRestartPacket(outBuffer,i);
      printf("%s\n\n",outBuffer);
   }
   printf("_____________________________________________________\n");
   printf("Build Init Packet Testing:\n\n");
   for (i = 0; i < 9; i++) {
      buildInitPacket(outBuffer, i, '1');
      printf("Stage: %i\nQuery: %s\n",i,outBuffer);
      buildInitPacket(outBuffer, i, '0');
      printf("Command/Setting: %s\n\n",outBuffer);
      printf("----------------------------------------------------\n");
   }
   printf("_____________________________________________________\n");
   printf("Find and Check Valid Packet Testing:\n\n");
   for (i = 1; i <= 3; i++) {
      int progStage = 0;
      int arbIndex = -1;
      switch (i) {
         case 1:
            progStage = 7;
            arbIndex = 117;
            arbitraryInBufferCreate(inBuffer, GENERIC_SBAS_RX_PKT, arbIndex);
            break;
         case 2:
            progStage = 2;
            arbIndex = 50;
            arbitraryInBufferCreate(inBuffer, GENERIC_OUTPUTS_RX_PKT, arbIndex);
            break;
         case 3:
            progStage = 4;
            arbIndex = 96;
            arbitraryInBufferCreate(inBuffer, GENERIC_SPD_THRESHOLD_RX_PKT, arbIndex);
            break;
      }
      printf("Response Packet: %s\n",inBuffer);
      int searchResultIndex = findInputPacket(inBuffer, progStage, '1');
      printf("Actual Index: %i\nCalculated Index (negative if rejected): %i\n",arbIndex,searchResultIndex);
      char correctness = checkResponseCorrect(inBuffer, arbIndex, progStage, '1');
      printf("Correctness Estimate: %c\n\n",correctness);
      printf("-----------------------------------------------------------------------------\n\n");
   }
   printf("___________________________________________________________________________________\n\n");
   printf("NEMA Sequence Testing:\n\n");
   for (i = 1; i <= 3; i++) {
      int arbIndex = -1;
      switch (i) {
         case 1:
            arbIndex = 21;
            arbitraryInBufferCreate(inBuffer, NEMA_SEQ_A, arbIndex);
            break;
         case 2:
            arbIndex = 77;
            arbitraryInBufferCreate(inBuffer, NEMA_SEQ_B, arbIndex);
            break;
         case 3:
            arbIndex = 99;
            arbitraryInBufferCreate(inBuffer, NEMA_SEQ_C, arbIndex);
            break;
      }
      printf("NEMA Packet: %s\n",inBuffer);
      int searchResultIndex = findInputPacket(inBuffer, 9, 'f');
      printf("Actual Index: %i\nCalculated Index (negative if rejected): %i\n",arbIndex,searchResultIndex);
      char validity = checkDataValidity(inBuffer, arbIndex);
      printf("Validity Estimate: %c\n\n",validity);
      printf("Parsed Data: \n");
      parseInfo(inBuffer, searchResultIndex, latLong, timeDate, speedCrs);
      printf("Lat/Long: \n%s\n\nTime/Date: \n%s\n\nSpeed/Heading: \n%s\n\n",latLong,timeDate,speedCrs);
      printf("-----------------------------------------------------------------------------\n\n");
   }
   return 0;
}

int arbitraryInBufferCreate(char *inBuffer, char *text, int index)
{
   char cr = ((char) 13);
   char lf = ((char) 10);
   int i;
   char bla = 'a';
   int count = 0;
   int textCount = 0;
   for (i = 0; i < 259; i++) {
      if ((i < index) || (i >= (index+(strlen(text))+2))) {
         switch (count) {
            case 0:
               bla = 't';
               break;
            case 1:
               bla = '!';
               break;
            case 2:
               bla = '$';
               break;
            case 3:
               bla = '*';
               break;
            case 4:
               bla = 'q';
               break;           
            case 5:
               bla = 'f';
               break;
            case 6:
               bla = '?';
               break;
            case 7:
               bla = '@';
               break;
         }
         inBuffer[i] = bla;
         if (count < 7) {
            count++;
         } else {
            count = 0;
         }
      } else if ((i >= index) && (i < (index+(strlen(text))))) {
         inBuffer[i] = text[textCount];
         textCount++;
      } else if ((i >= index) && (i == (index+(strlen(text))))) {
         inBuffer[i] = cr;
      } else if ((i >= index) && (i == (index+(strlen(text)+1)))) {
         inBuffer[i] = lf;
      }
   }
   inBuffer[259] = ((char) 0);
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
   sprintf(outBuffer,"$%s*%s%c%c",outBuffer,checksum,cr,lf);
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
      sprintf(expectedPacket,"$%s*%s%c%c",expectedPacket,checksum,cr,lf);
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
       