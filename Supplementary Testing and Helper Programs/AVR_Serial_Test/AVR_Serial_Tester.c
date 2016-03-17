//#include "USART.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
//#include <avr/iom328p.h>
#include "usart.h"
//#include "pinDefines.h"

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
#define TEST_PKT "$PMTK000*32"

//#define BAUD 115200
//#define F_CPU 16000000
#define UPDATE_RATE 350    // in milliseconds
#define INIT_MSG "Initializing..."
#define ACQ_MSG "Acquiring\nData..."
#define RX_CAP 34
#define DISP_CAP 34
#define DISP_HOLD_TIME 2000

static int updateInBuffer(char *inBuffer);
//static int getNextInFragment(char *displayBufferB, char *inBuffer, int index);
static int initDisplay(void);
static int sendDisplayBytes(char byteA, char byteB);
static int updateDisplay(char *displayBuffer, char *message, int progStage);
static int initTimer1(void);
static int initUSART(void);
static uint8_t receiveByte(void);
static int transmitByte(uint8_t data);
static int sendTestPacket(void);

int main(void)
{
   initDisplay();
   initUSART();
   initTimer1();
   char *displayBuffer = (char * ) malloc(DISP_CAP);
   //char *displayBufferB = (char * ) malloc(DISP_CAP);
   char *inBuffer = (char * ) malloc(RX_CAP);
   updateDisplay(displayBuffer,INIT_MSG,10);
   int counter = 0;
   while (1) {
      //updateDisplay(displayBuffer,ACQ_MSG,10);
      if (counter == 15) {
         sendTestPacket();
         updateInBuffer(inBuffer);
         updateDisplay(displayBuffer,inBuffer,10);
         _delay_ms(DISP_HOLD_TIME);
         counter = 0;
      } else {
         updateInBuffer(inBuffer);
         updateDisplay(displayBuffer,inBuffer,10);
         _delay_ms(500);
      }
      counter++;   
      //int index = 0;
      /*while (index < (strlen(inBuffer)-1)) {
         getNextInput(inBuffer,index);
         updateDisplay(displayBuffer,displayBufferB,10);
         index += 32;
         if (index < (strlen(inBuffer)-1)) {
            _delay_ms(DISP_HOLD_TIME);
         }   
      }*/
   }
   return 0;
}

int sendTestPacket()
{
   int i;
   for (i = 0; i < (strlen(TEST_PKT)); i++) {
      uint8_t current = ((uint8_t) TEST_PKT[i]);
      transmitByte(current);
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

int updateInBuffer(char *inBuffer)
{
   int i = 0;
   TCNT1 = 0;
   int timerValue = (TCNT1 >> 4);
   int lastIndex = (RX_CAP-1);
   while ((i < lastIndex) && (timerValue <= UPDATE_RATE)) {
      if (i == 15) {
         inBuffer[i] = '\n';
      } else {      
         char byte = ((char) receiveByte());
         inBuffer[i] = byte;
      }   
      i++;
      //timerValue = (TCNT1 >> 4);
   }
   inBuffer[lastIndex] = '\0';
   return 0;
}

int transmitByte(uint8_t data)
{
   loop_until_bit_is_set(UCSR0A, UDRE0);
   UDR0 = data;   /* send data */
return 0;
}

/*int getNextInFragment(char *displayBufferB, char *inBuffer, int index)
{
   char *tempA = (char * ) malloc(18);
   char *tempB = (char * ) malloc(18);
   strncpy(tempA,(inBuffer+index),16);
   strncpy(tempB,(inBuffer+index+16),16);
   sprintf(displayBufferB,"%s\n%s",tempA,tempB);
   free(tempA);
   free(tempB);
   return 0;
}*/

uint8_t receiveByte() 
{
   loop_until_bit_is_set(UCSR0A, RXC0);       /* Wait for incoming data */
   return UDR0;                                /* return register value */
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
            _delay_ms(3);
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

int updateDisplay(char *displayBuffer, char *message, int progStage)
{
   sendDisplayBytes(CLR_DISP_A,CLR_DISP_B);
   _delay_ms(3);
   if ((progStage > 0) && (progStage < 10)) {
      sprintf(displayBuffer,"%s%i of 9",message,progStage);
   } else {
      sprintf(displayBuffer,"%s",message);
   }   
   int length = strlen(displayBuffer);
   int i;
   char byteA = ((char) ALLZERO);
   char byteB = ((char) ALLZERO);
   for (i = 0; i < length; i++) {
      char current = displayBuffer[i];
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