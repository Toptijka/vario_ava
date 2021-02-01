

#include "mltbt05.h"


#include <SoftwareSerial.h>
// RX, TX
SoftwareSerial mySerial(4,3);

#define UART_SPEED 9600  // 2400 //57600 //115200 //19200 // 38400 //9600
/*
AT+BAUD1——-1200
AT+BAUD2——-2400
AT+BAUD3——-4800
AT+BAUD4——-9600
AT+BAUD5——19200
AT+BAUD6——38400
AT+BAUD7——57600
AT+BAUD8——115200
AT+BAUD9——230400
AT+BAUDA——460800
AT+BAUDB——921600
AT+BAUDC—-1382400
*/

const char name [] = "fanfario";
MLTBT05 my_bt (3,4,5,UART_SPEED);


void setup() {
  
  mySerial.begin(UART_SPEED);
  
  my_bt.configure(name);
  
  mySerial.println("Hello AVA!");

}

void loop() {

}
