/*
Part of http://cutecare.ru project
Author: evgeny.savitsky@gmail.com
*/
#include "mltbt05.h"

/**
 * Constructor
 */
MLTBT05::MLTBT05(int rxPin, int txPin, int resetPin, long baud)
{
	bleRxPin = rxPin;
	bleTxPin = txPin;
	bleResetPin = resetPin;
	bleBaud = baud;
}

void MLTBT05::configure(const char * bleName)
{
	pinMode(bleRxPin, OUTPUT);
	
	//SoftwareSerial bleSerial(bleTxPin, bleRxPin);
	Serial.begin(bleBaud);

  delay(5000);
  Serial.println("Ready to configurate?");
  delay(1000);
  Serial.println("GO!");
	
	char text[32] = "";
	sprintf(text, "AT+NAME%s", bleName);
	sendCommand(text);
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
/* for JDY-23
0——*115200
1——57600
2——38400
3——19200
4——9600
5——4800
6——2400
*/

  //sendCommand(&bleSerial, "AT+ORGL");
  sendCommand( "AT+BAUD1");
	sendCommand( "AT+ROLE3");	// slave mode
	sendCommand( "AT+TYPE0");	// unsecure, no pin required
	sendCommand( "AT+POWE3");	// max RF power
	sendCommand( "AT+ADVI3");	// advertising interval 300ms
	sendCommand( "AT+MARJ0x0000");    
	sendCommand( "AT+MINO0x0000");
	sendCommand( "AT+RESET");    // sleep
	
	pinMode(bleRxPin, INPUT);
}

void MLTBT05::setData(unsigned int minor, unsigned int major, bool autosleep)
{
	pinMode(bleRxPin, OUTPUT);
	wakeUpBLE();

	//SoftwareSerial bleSerial(bleTxPin, bleRxPin);
	Serial.begin(bleBaud);

	char buff[32] = "";
	sprintf(buff, "AT+MARJ0x%04X", major);
	sendCommand(buff);
	sprintf(buff, "AT+MINO0x%04X", minor);
	sendCommand(buff);
	if (autosleep) {
		sendCommand( "AT+RESET");
	}

	pinMode(bleRxPin, INPUT);
}

void MLTBT05::major(unsigned int value, bool autosleep)
{
	pinMode(bleRxPin, OUTPUT);
	wakeUpBLE();

	//SoftwareSerial bleSerial(bleTxPin, bleRxPin);
	Serial.begin(bleBaud);

	char buff[32] = "";
	sprintf(buff, "AT+MARJ0x%04X", value);
	sendCommand(buff);
	if (autosleep) {
		sendCommand( "AT+RESET");
	}

	pinMode(bleRxPin, INPUT);
}

void MLTBT05::minor(unsigned int value, bool autosleep)
{
	pinMode(bleRxPin, OUTPUT);
	wakeUpBLE();

	//SoftwareSerial bleSerial(bleTxPin, bleRxPin);
	Serial.begin(bleBaud);

	char buff[32] = "";
	sprintf(buff, "AT+MINO0x%04X", value);
	sendCommand(buff);
	if (autosleep) {
		sendCommand( "AT+RESET");
	}

	pinMode(bleRxPin, INPUT);
}

void MLTBT05::wakeUpBLE()
{
	pinMode(bleResetPin, OUTPUT);
	delay(300);
	digitalWrite(bleResetPin, HIGH);
	delay(300);
	digitalWrite(bleResetPin, LOW);
	pinMode(bleResetPin, INPUT);
}

void MLTBT05::sendCommand(const char * data) {
	delay(250);
	Serial.println(data);
}
