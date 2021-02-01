/*
Part of http://cutecare.ru project
Author: evgeny.savitsky@gmail.com
*/

#ifndef MLTBT05_h
#define MLTBT05_h

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#ifndef WDTCR
  #include <SoftwareSerial.h>
#else
  #include <SoftSerial.h> 
  #define SoftwareSerial SoftSerial
#endif

class MLTBT05 {
  public:
	  MLTBT05(int rxPin = 3, int txPin = 4, int resetPin = 5, long baud = 115200);
	  void configure(const char * bleName);
	  void setData(unsigned int minor = 0, unsigned int major = 0, bool autosleep = true);
	  void major(unsigned int value, bool autosleep = true);
	  void minor(unsigned int value, bool autosleep = true);

  private:
	  void sendCommand(const char * data);
	  void wakeUpBLE();

	  int bleTxPin = 0;
	  int bleRxPin = 0;
	  int bleResetPin = 0;
	  long bleBaud = 0;
};

#endif
