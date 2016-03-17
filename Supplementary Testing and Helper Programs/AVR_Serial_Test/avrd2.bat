make clean
make
"C:\Program Files (x86)\Arduino\hardware\tools\avr/bin/avrdude" -C "C:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -patmega328p -c arduino -P COM7 -b 115200 -D -U flash:w:C:\Users\MYDELL~1\Documents\ece387\MIDTER~1/AVR_Serial_Test/AVR_LCD_Tester.hex:i