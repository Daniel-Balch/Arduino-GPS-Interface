/* Functions to initialize, send, receive over USART

   initUSART requires BAUD to be defined in order to calculate
     the bit-rate multiplier.
 */

#ifndef BAUD                          /* if not defined in Makefile... */
#define BAUD  9600                     /* set a safe default baud rate */
#endif

                                  /* These are defined for convenience */
#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)
//#define BAUD 115200
//#define F_CPU 16000000

/* Takes the defined BAUD and F_CPU,
   calculates the bit-clock multiplier,
   and configures the hardware USART                   */
static int initUSART(void);

/* Blocking transmit and receive functions.
   When you call receiveByte() your program will hang until
   data comes through.  We'll improve on this later. */
static int transmitByte(uint8_t data);
static uint8_t receiveByte(void);

static int printString(const char myString[]);
             /* Utility function to transmit an entire string from RAM */
static int readString(char myString[], uint8_t maxLength);
/* Define a string variable, pass it to this function
   The string will contain whatever you typed over serial */

static int printByte(uint8_t byte);
                  /* Prints a byte out as its 3-digit ascii equivalent */
static int printWord(uint16_t word);
        /* Prints a word (16-bits) out as its 5-digit ascii equivalent */

static int printBinaryByte(uint8_t byte);
                                     /* Prints a byte out in 1s and 0s */
static char nibbleToHex(uint8_t nibble);
static int printHexByte(uint8_t byte);
                                   /* Prints a byte out in hexadecimal */
static uint8_t getNumber(void);
/* takes in up to three ascii digits,
 converts them to a byte when press enter */
