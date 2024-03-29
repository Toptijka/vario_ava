//This scetch was modified by Andrey Vorotnikov
//Use BMP280 pressure sensor and BT module CC2541
//BT_RX <---------> Pin_3 (TX)
//BT_TX <---------> Pin_4 (RX)
//BMP280_SCL <----> Pin_A5
//BMP280_SDA <----> Pin_A4
//Battery_level <-> Pin_A6

// to V.V.: странно делать датчик весом 10 грамм с питанием от внешнего аккумулятора весом 200 грамм.

//Source: http://www.parapentiste.info/forum/bons-plans/vario-bluetooth-pour-xctrack-lk8000-sur-vos-tablettes-ou-smartphones-t48268.0.html

//This comments from original sketch
/*
Arduino vario by Benjamin PERRIN 2017 / rx_dati'Up

Based on Arduino vario by Jaros, 2012 and vario DFelix 2013

$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

Credits:
(1) http://code.google.com/p/bmp085driver/                             //bmp085 library
(2) http://mbed.org/users/tkreyche/notebook/bmp085-pressure-sensor/    //more about bmp085 and average filter
(3) http://code.google.com/p/rogue-code/                               //helpfull tone library to make nice beeping without using delay
*/

// #define CLOCK_DIVIDE
// //#define INITIAL_EEPROM

// #ifdef CLOCK_DIVIDE
//  #define DIV_FACTOR 2
//  #define BT_SPEED 215000 //9600 // 38400
// #else
//  #define DIV_FACTOR 1
//  #define BT_SPEED 115200 //9600 // 38400
// #endif

#define LOOPS 100

// #include "LowPower.h"
#include <GyverPower.h>
#include <GyverUART.h>
#include <GyverTimers.h>
#include <GyverBME280.h>  
#include <stdlib.h>                     //we need that to use dtostrf() and convert float to string
#include <math.h>
#include "vario_ava.h"

/////////////////////////////////////////
float Temperature = 0;
float Pressure = 101325.0;
long Altitude;
int vario;
long vario_average;
float vario_filtered;
const float p0 = 101325.0;              //Pressure at sea level (Pa)
unsigned long get_time1 = 0;
// unsigned long get_time2 = 0;
unsigned long get_time3 = 0;
unsigned long get_time4 = 0;

volatile unsigned long button_time = 0;
volatile boolean button = false;
volatile boolean maybe_pwdown = false;
volatile boolean change_volume = false;
volatile boolean change_flight = false;
volatile unsigned int update_freq = 0xffff;
// volatile unsigned int update_duty = 0xffff;
volatile int fshift = 0;

unsigned int freq;

int      vcc_out_pin = 5;
int      bt_stat_pin = 6;
int      bt_pwrc_pin = 7;
int      bt_rst_pin = 8;
int      battery_pin = A6;
int      battery_tr_pin = 10;
int      button_pin = 3;
float      battery_level;
bool replace_battery = 0;
int fake_battery;
float batt_average;

#define buzz_max_array 100
// #define alt_max_array 50
// #define MAX_VOLUME 800
// #define MED_VOLUME 300
// #define MIN_VOLUME 150

bool paramoo = 0;

// unsigned long alt_array[alt_max_array];
// unsigned long time_array[alt_max_array];
// int alt_array_pointer = 0;

bool first_time = 1;
int buzz_array[buzz_max_array];
int buzz_array_pointer = 0;
// int buzz_volume_up;
byte buzz_cnt = 0;
byte buzz_period = 7;
enum flight_mode {st_mute, st_down_0, st_down_1, st_up_0, st_up_1, st_up_2, st_up_3, st_buzzzz};
flight_mode flight_mode = st_mute;
unsigned long time_old, time_new;
long alt_old;

unsigned long time_end_of_flight;
unsigned long time_flight_stop;
unsigned long time_flight_start;
unsigned long time_start_working;

bool buzz_en = 0;
bool programming_mode = 0;
bool bt_connect = 0;
bool flight = 0;
bool sleep = 0;
bool want_sleep = 0;
// bool sending = 0;
byte cnt_loop = 1;

byte button_cnt = 0;

GyverBME280 bme;

void buzz_flight(unsigned int freq, bool buzblink)
{
  if (!buzz_en || !buzblink)
    {
    //PWM_square_D9(freq);// + (vario_average - vario_average%10)*2 - 20);
    //pwm = SetPinFrequencySafe(9,freq);
    PWM_frequency(9, freq, CORRECT_PWM);
    //pwmWrite(9, 128);
    PWM_set(9, buzz_volume);
    buzz_en = 1;
    }
  else
    {
    //PWM_default(9);
    //pwmWrite(9, 0);
    disable_timer();
    // PWM_set(9, 0);
    buzz_en = 0;
    }
}

void wait(int del)
{
  unsigned long start_sleep = millis();
int ii = 0;
  while (millis() < start_sleep + del/DIV_FACTOR) {
    power.sleep(SLEEP_FOREVER);
    // delay(1);
    ii++;
  }

// uart.print("ii: ");
// uart.println(ii);
  // delay(del/DIV_FACTOR);
  // delay(120/DIV_FACTOR);
}

void buzz_set_volume(int in_dat)
{
  freq_shift_off();
  int volume = (in_dat == 0)? MAX_VOLUME : in_dat;
  unsigned int freq = 700;
  buzz_volume = in_dat;
  for (int i = 0; i < 5; i++) {
    PWM_frequency(9, freq, CORRECT_PWM);
    PWM_set(9, volume);
    delay(60 / DIV_FACTOR);
    // PWM_set(9, 0);
    disable_timer();
    delay(60 / DIV_FACTOR);
    if (in_dat == 0) {
      volume -= 40;
      freq -= 40;
    }
  }
  // freq_shift_on();
}

void press_button() {
  button = 1;

  button_time = millis();
}

void bme_init() {
  bme.setFilter(FILTER_COEF_16);
  bme.setTempOversampling(OVERSAMPLING_2);
  bme.setPressOversampling(OVERSAMPLING_16);
  bme.setStandbyTime(STANDBY_500US);
  if(!bme.begin()) while (1) {
    uart.println("Where is BMP280?!");
    delay(1000);
    };
}

void bme_read() {
  Pressure = bme.readPressure();
  Altitude = (44330.0 * (1.0 - pow(((float)Pressure / p0), 0.190295)) * 100.0);
}

void pwr_down() {
  power.setSleepMode(POWERDOWN_SLEEP);
  button_cnt = 0;
  sleep = 1;
  sensor_pwr_off();
  disable_timer();
  power.sleep(SLEEP_FOREVER);
}

void sensor_pwr_off() {
uart.end(); 
digitalWrite(0, 0);
digitalWrite(1, 0);
digitalWrite(bt_pwrc_pin, 0);
digitalWrite(bt_rst_pin, 0);
digitalWrite(vcc_out_pin, 0);
}
int varray [20];
int varray_pointer = 0;

void sensor_pwr_on()
{
uart.begin(BT_SPEED);
digitalWrite(vcc_out_pin, 1);
digitalWrite(bt_pwrc_pin, 1);
digitalWrite(bt_rst_pin, 0);
delay(100/DIV_FACTOR);
digitalWrite(bt_rst_pin, 1);
delay(500/DIV_FACTOR);
uart.println(F("Try to connect bmp280..."));
bme_init();
uart.println(F("bmp280!!!"));
power.setSleepMode(IDLE_SLEEP);
time_end_of_flight = millis();
vario_filtered = 0;
first_time = 1;
varray_pointer  = 0;
buzz_hello();
delay(500/DIV_FACTOR);
bme_read();
}

void buzz_hello()
{
freq_shift_off();
PWM_frequency(9, 3000/**DIV_FACTOR*/, CORRECT_PWM);
PWM_set(9, 300);//buzz_volume);
delay(500/DIV_FACTOR);
// PWM_set(9, 0);
disable_timer();
// freq_shift_on();
}

void setup()
{
#ifdef CLOCK_DIVIDE
power.setSystemPrescaler(PRESCALER_2);
#endif

pinMode(bt_stat_pin, INPUT);
pinMode(vcc_out_pin, OUTPUT);
pinMode(bt_pwrc_pin, OUTPUT);
pinMode(bt_rst_pin, OUTPUT);

// uart.begin(BT_SPEED);     // start communication with the bt-module
  
// uart.println(F("Try to connect bmp280..."));
// bme_init();
// uart.println(F("bmp280!!!"));

pinMode(9, OUTPUT); //buzzer pin
pinMode(SDA, INPUT);
pinMode(SCL, INPUT);

#ifdef INITIAL_EEPROM
  default_params();
  update_params();
#else
  read_params();
#endif

sensor_pwr_on();

  // buzz_hello();

for (int i = 0; i < buzz_max_array; i++)
  buzz_array[i] = 0;

pinMode(battery_tr_pin, OUTPUT);
digitalWrite(battery_tr_pin,1);
batt_average = ((float) analogRead(battery_pin)) * 2 * 3.3 / 1024;
for (int i = 0; i < 100; i++)
  read_voltage();

pinMode(button_pin, INPUT_PULLUP);
attachInterrupt(1, press_button, FALLING);
time_start_working = millis();
working_time = 0;
//TCCR0B = 5;
//OCR0A = 255;
//OCR0B = 255;

// Timer2.setPeriod(200000);
// Timer2.enableISR(CHANNEL_A);

// delay (1000);

// freq_shift_on();

}

ISR(TIMER1_OVF_vect) {
  // OCR1A = OCR1A + 1;//update_duty;
      //   OCR1AH = highByte(duty);
      // OCR1AL = lowByte(duty);
  ICR1 = update_freq;
  // ICR1 += fshift;
  // freq_shift();
}

/* ************************* LOOP ****************************************** */

void loop(void)
{

bt_connect = digitalRead(bt_stat_pin);

String rx_dat="";

if(!programming_mode) {
  if (!sleep) {


/* ************************ VARIO ****************************************** */
  if (cnt_loop % (LOOPS/10) == 1) { // calculating time
  bme_read();

time_new = millis() * DIV_FACTOR;

// vario = -(analogRead(A2) - 100);
vario = first_time? 0 : 1000 * ((float)(Altitude - alt_old) / (float)(time_new - time_old));

first_time = 0;

// if (varray_pointer < 20){
//   varray[varray_pointer] = vario;
//   varray[varray_pointer+1] = first_time;
//   varray_pointer+=2;
//   }

// first_time = 0;

time_old = time_new;
alt_old = Altitude;

if (!bt_connect || buzz_always != 0 || send_average) {
  buzz_array[buzz_array_pointer] = vario;
  if (buzz_array_pointer != buzz_size_array - 1) buzz_array_pointer++;
  else buzz_array_pointer = 0;
}

// sending = 1;
wait(1000/LOOPS - 5);
}
/* ********************** Data to tranceive ******************************** */
else if (cnt_loop % (LOOPS/10) == 2) { // sending data time
  if(bt_connect) {
    //PWM_default(9);
    //pwmWrite(9, 0);
    // PWM_set(9, 0);
    String str_out = String("$LK8EX1," + String((long)Pressure, DEC) + String(",") + String(Altitude / 100, DEC) + String(","));
    
    if (send_average==1) str_out = String(str_out + String(vario_average, DEC));
    else str_out = String(str_out + String(vario, DEC));

    str_out = String(str_out + String(",") + String((int)Temperature, DEC) + String(",") + String(fake_battery/*battery_level*/, DEC) + String(","));
    unsigned int checksum_end, ai, bi;                                               // Calculating checksum for data string
    for (checksum_end = 0, ai = 1; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    //creating now NMEA uart output LK8EX1 protocol format:
    //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
    //uart.println(millis());
    uart.print(str_out);                 //print data string
    uart.print("*");                     //end of protocol string
    uart.println(checksum_end, HEX);     //print calculated checksum on the end of the string in HEX
    //uart.println(millis());

  }
// sending = 0;
 // uart.println(vario_average);
// uart.println(freq);
// uart.println(buzz_period);
wait(1000/LOOPS - 3);
}

/* ********************** Dummy ******************************** */
else {
wait(1000/LOOPS);
}

if (cnt_loop >= LOOPS) cnt_loop = 1;
else cnt_loop ++;
/* ************************** Buzzer ********************************************* */

bool buzzer_en = (!bt_connect || buzz_always != 0) && buzz_volume != 0;

if (buzzer_en || send_average) average();

if (buzzer_en) buzzer();
else disable_timer();

/* ********************** Flight filter and count ******************************** */

if (cnt_loop % (LOOPS/10) == 1) {

  vario_filtered = vario_filtered * 15.0 / 16.0 + (float)vario / 16.0;
  // uart.println(vario_filtered);

  if ((vario_filtered < 20 && vario_filtered > -20) && !flight) time_flight_start = millis();
  
  if (!flight)
    if (millis() > time_flight_start + flight_start_filter/DIV_FACTOR) {
      flight = 1;
      flight_time = 0;
      // buzz_start_of_flight(buzz_volume);
    }

  
  if ((vario_filtered > 20 || vario_filtered < -20) && flight) time_flight_stop = millis();
  
  if (flight) {
    if (millis() > time_flight_stop + flight_stop_filter*1000/DIV_FACTOR) {
      // uart.println(millis());
      // uart.println(time_flight_stop);
      // uart.println(flight);
      flight = 0;
      buzz_end_of_flight(buzz_volume);
      count_flights++;
      update_int(adr_count_flights,count_flights);
      // uart.println(flight);
    }

    if (millis() > time_flight_start + (flight_time+1)*60000/DIV_FACTOR) {
      flight_time++;
      update_int(adr_flight_time,flight_time);
      total_flight_time++;
      update_int(adr_total_flight_time,total_flight_time);
    }

  }

if (flight) time_end_of_flight = millis();
else if(millis() > time_end_of_flight + pwdown_time*60000/DIV_FACTOR) {
    time_end_of_flight = millis();
    want_sleep = 1;
}

/* **************************** Working time ************************************* */

if (millis() > time_start_working + (working_time+1)*60000/DIV_FACTOR) {
      working_time++;
      total_working_time++;
      update_int(adr_total_working_time,total_working_time);
    }

/* ********************** Battery <-> Temperature ******************************** */

  if (millis() >= (get_time3 + 2000/DIV_FACTOR))    //rotate temperature and battery_level
  {
  Temperature = bme.readTemperature();

  if (display_temp == 2) {
    uart.print("$XCTOD,");
    uart.println(Temperature);
    }
  
  if (replace_battery & display_temp == 1)
    {
    fake_battery = (float)Temperature;
    if (Temperature <= 0)
      {
      fake_battery *= -1;
      fake_battery += 100;
      }
    replace_battery = 0;
    }
  else
    {
    battery_level = (read_voltage() * (float) battery_calibration / 100 - 3.6) * 100 / (4.2 - 3.6) ;
    battery_level = constrain(battery_level,0,100);
    fake_battery = (int) battery_level + 1000;
    replace_battery = 1;
    }
  get_time3 = millis();
  }

if (millis() > (get_time4 + 60000/DIV_FACTOR) && (int) battery_level < battery_alarm_level)
  {
  freq_shift_off();
  for (int i=0 ; i<8 ; i++) {
    buzz_flight(300,1);
    delay(50);
    }
  // freq_shift_on();

  get_time4 = millis();
  }
}
/* ********************** Jump to progamming mode ******************************** */

if(uart.available())
  {
  delay(200/DIV_FACTOR);
  while (uart.available())
    //uart.print((char)uart.read());
    rx_dat += (char)(uart.read());
    // uart.println(rx_dat);
  if (rx_dat.startsWith("program"))
    {
    //PWM_default(9);
    //pwmWrite(9, 0);
    // PWM_set(9, 0);
    disable_timer();
    programming_mode = 1;
    print_message(welcome_message,strlen_P(welcome_message));
    //for (int i = 0; i < strlen_P(welcome_message); i++)
      //uart.print((char)pgm_read_byte(&welcome_message[i]));
    }
  }
}
}
/* ********************** Receive data ******************************** */
else { //programming_mode
  //PWM_frequency(9, buzz_freq, CORRECT_PWM);//FAST_PWM);
  //PWM_set(9, buzz_volume);

if(uart.available())
  {
  delay(200);
  while (uart.available())
    rx_dat += (char)(uart.read());
  if (rx_dat.startsWith("p0=")) buzz_size_array = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p1=")) buzz_up_thres = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p2=")) buzz_down_thres = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p3=")) buzz_up_start_freq = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p4=")) buzz_down_start_freq = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p5=")) buzz_set_volume(constrain(rx_dat.substring(3,rx_dat.length()-2).toInt(), 0, MAX_VOLUME));
  if (rx_dat.startsWith("p6=")) pwdown_time = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p7=")) freq_increment = constrain(rx_dat.substring(3,rx_dat.length()-2).toInt(),1,10);
  // if (rx_dat.startsWith("p8="))  = rx_dat.substring(3,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p9=")) send_average = constrain(rx_dat.substring(3,rx_dat.length()-2).toInt(),0,1);
  if (rx_dat.startsWith("p10=")) hold_button = constrain(rx_dat.substring(4,rx_dat.length()-2).toInt(),0,10000);
  if (rx_dat.startsWith("p11=")) battery_alarm_level = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p12=")) display_temp = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p13=")) buzz_always = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p14=")) flight_stop_filter = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p15=")) flight_start_filter = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p16=")) flight_time = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p17=")) total_flight_time = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p18=")) count_flights = rx_dat.substring(4,rx_dat.length()-2).toInt();
  if (rx_dat.startsWith("p19=")) total_working_time = rx_dat.substring(4,rx_dat.length()-2).toInt();
  // if (rx_dat.startsWith("p20=")) 


  //if (rx_dat.startsWith("p19=")) vario = rx_dat.substring(4,rx_dat.length()-2).toInt();
  //if (rx_dat.startsWith("p20=")) buzz_freq = rx_dat.substring(4,rx_dat.length()-2).toInt();

  if (rx_dat.startsWith("save"))
    {
    buzz_size_array = constrain(buzz_size_array,1,buzz_max_array);
    update_params();
    uart.println("Whrite Ok!");
    programming_mode = 0;
    delay(2000/DIV_FACTOR);
    }

  if (rx_dat.startsWith("report"))
    {
    uart.println("p0=" + String(buzz_size_array,DEC));
    uart.println("p1=" + String(buzz_up_thres,DEC));
    uart.println("p2=" + String(buzz_down_thres,DEC));
    uart.println("p3=" + String(buzz_up_start_freq,DEC));
    uart.println("p4=" + String(buzz_down_start_freq,DEC));
    uart.println("p5=" + String(buzz_volume,DEC));
    uart.println("p6=" + String(pwdown_time,DEC));
    uart.println("p7=" + String(freq_increment,DEC));
    // uart.println("p8=" + String(,DEC));
    uart.println("p9=" + String(send_average,DEC));
    delay(100);
    uart.println("p10=" + String(hold_button,DEC));
    uart.println("p11=" + String(battery_alarm_level,DEC));
    uart.println("p12=" + String(display_temp,DEC));
    uart.println("p13=" + String(buzz_always!=0,DEC));
    uart.println("p14=" + String(flight_stop_filter,DEC));
    uart.println("p15=" + String(flight_start_filter,DEC));
    uart.println("p16=" + String(flight_time,DEC));
    uart.println("p17=" + String(total_flight_time,DEC));
    uart.println("p18=" + String(count_flights,DEC));
    uart.println("p19=" + String(total_working_time,DEC));
    // uart.println("p20=" + String(flight_stop_filter,DEC));
    }

  if (rx_dat.startsWith("help")) print_message(help_message, strlen_P(help_message));

  if (rx_dat.startsWith("exit") || !bt_connect) {
    read_params();
    programming_mode = 0;
  }

  if (rx_dat.startsWith("default")) default_params();

  // if (rx_dat.startsWith("var"))
  //   for(int i=0;i<20;i++)
  //     uart.println(varray[i]);

  if (rx_dat.startsWith("calibrate")) battery_calibration = (int)(4.2 * 100/read_voltage());

  if (rx_dat.startsWith("sleep")) {
    uart.println("want sleep");
    flight = 0;
    read_params();
    programming_mode = 0;
    buzz_pwdown(buzz_volume);
    pwr_down();
  }

  if (rx_dat.startsWith("moo")) {
    paramoo = 1;
    print_message(moo_message,sizeof(moo_message));
  }

  if (rx_dat.startsWith("yes") && paramoo) {
    paramoo = 0;
    print_message(paramoo_message,sizeof(paramoo_message));
  }

  }
}

/* ********************** Change volume/flight/power mode ******************************** */

if (want_sleep) {
  want_sleep = 0;
  buzz_pwdown(buzz_volume);
  pwr_down();
  }

if (sleep && (millis() - button_time) > 1000/DIV_FACTOR && (hold_button == 0)) pwr_down();

if (button) {
  if (digitalRead(button_pin)) {
    if (sleep) {
      if (hold_button == 0) {
        if (button_cnt == 2) {
          // buzz_hello();
          sensor_pwr_on();
          sleep = 0;
        }
      }
      else pwr_down();
      button_cnt++ ;
      button = 0;
    }
    else if (buzz_volume < MIN_VOLUME) buzz_set_volume(MIN_VOLUME);
    else if (buzz_volume < MED_VOLUME) buzz_set_volume(MED_VOLUME);
    else if (buzz_volume < MAX_VOLUME) buzz_set_volume(MAX_VOLUME);
    else buzz_set_volume(0);
    button = 0;
  }
  else
    if (sleep) {
      if (hold_button != 0) {
        if (millis() > button_time + hold_button / DIV_FACTOR) {
          sensor_pwr_on();
          sleep = 0;
        }
      }
    }
    else if (millis() > button_time + 3000 / DIV_FACTOR) {
      flight = 0;
      buzz_end_of_flight(buzz_volume);
      time_flight_start = millis();
      button = 0;
      maybe_pwdown = 1;
    }

}

if (maybe_pwdown) {
  if (millis() > button_time + 6000 / DIV_FACTOR) {
    maybe_pwdown = 0;
    buzz_pwdown(buzz_volume);
    pwr_down();
  }

}

if (digitalRead(button_pin)) maybe_pwdown = 0;
  
} // loop

void average() {
  byte weight;

  for (int i=0;i<buzz_size_array;i++)
    {
    if (i == buzz_array_pointer - 1) weight = cnt_loop % (LOOPS/10);
    else if (i == buzz_array_pointer - 2) weight = LOOPS/10 - cnt_loop % (LOOPS/10);
    else weight = LOOPS/10; // maximum of weight
    vario_average += buzz_array[i] * weight;
    }

vario_average = vario_average / (buzz_size_array * (LOOPS / 10));
}

void buzzer() {
// if (buzz_volume == 0) disable_timer();
// else if (/*bt_connect || */buzz_always != 0) {
  // freq_shift_on();

// if (cnt_loop % (LOOPS/10) == 1) uart.println (vario_average);

  float log10_vario;
  unsigned int wanted_freq;

  if (flight_mode == st_up_0) {
    log10_vario = log10(constrain(vario_average,1,1000));
    wanted_freq = (unsigned int) (log10_vario*200.0)+buzz_up_start_freq;
  } else {
    log10_vario = log10(constrain(abs(vario_average+100),1,1000));
    wanted_freq = constrain((unsigned int) (buzz_down_start_freq - (log10_vario*150.0)),50,buzz_down_start_freq);
  }

  // float log10_vario = log10(constrain(vario_average/*-buzz_up_thres*/,1,1000));
  // unsigned int wanted_freq = (flight_mode == st_up_0)? (unsigned int) (log10_vario*200.0)+buzz_up_start_freq :
  //                                          constrain((buzz_down_start_freq + (vario_average+buzz_down_thres)/buzz_down_factor),1,buzz_down_start_freq);

  freq = constrain(wanted_freq, freq-freq_increment, freq+freq_increment);

  buzz_period = (byte)(0.4*LOOPS) - (byte) (log10_vario * LOOPS/10) ;

// uart.println(vario_average);
// uart.println(freq);
// uart.println(buzz_period);

  update_freq = (freq > 256)? 8000000 / (freq * DIV_FACTOR) : 1000000 / (freq * DIV_FACTOR);

  // if(buzz_cnt >= 5) update_duty = (TCCR1B == 0b00010001)? buzz_volume/DIV_FACTOR : buzz_volume/(DIV_FACTOR*8);
  // else update_duty = (TCCR1B == 0b00010001)? buzz_cnt*buzz_volume/(DIV_FACTOR*5) : buzz_cnt*buzz_volume/(DIV_FACTOR*8*5);

  if (flight_mode == st_up_0 && buzz_cnt == 0) {
    freq_shift_on();
    buzz_flight(freq/**DIV_FACTOR*/,1);
    // buzz_volume_up = buzz_volume * ;
  } else if (flight_mode == st_buzzzz) {
    // freq_shift_on();
    buzz_flight(freq/**DIV_FACTOR*/,1);
  } else if (flight_mode == st_down_0) {
    freq_shift_on();
    // freq_shift_off();
    buzz_flight(freq/**DIV_FACTOR*/,0);
  } else if (flight_mode == st_mute) disable_timer();

  buzz_cnt = (buzz_cnt >= buzz_period)? 0 : buzz_cnt + 1;

  // if      (vario_average >= buzz_up_3_thres && flight) flight_mode = st_up_3;
  // else if (vario_average >= buzz_up_2_thres && flight) flight_mode = st_up_2;
  // else if (vario_average >= buzz_up_1_thres && flight) flight_mode = st_up_1;
  //else 
  if (vario_average >= buzz_up_thres && flight) {
    if (flight_mode != st_up_0) freq = buzz_up_start_freq;
    flight_mode = st_up_0;
  }
  // else if (vario_average >= 0 && flight) {
  //   if (flight_mode != st_buzzzz) freq = buzz_down_start_freq;
  //   flight_mode = st_buzzzz;
  // }
  else if (vario_average <= buzz_down_thres && flight) {
    if (flight_mode != st_down_0) freq = wanted_freq;//buzz_down_start_freq;
    flight_mode = st_down_0;    
  }
  else if (buzz_cnt == 0) flight_mode = st_mute;

  // if (flight_mode == st_mute) freq = buzz_up_start_freq;

  // }

} // Buzzer


void print_message(const char * message, int size) {
  for (int i = 0; i < size; i++) {
    uart.print((char)pgm_read_byte(&message[i]));
    if (i % 64 == 63) delay(100 / DIV_FACTOR);
  }
} //print_message