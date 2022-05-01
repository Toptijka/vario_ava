//based on GyverPWM

#ifndef vario_ava_h
#define vario_ava_h
#include <Arduino.h>
#include <EEPROM.h>

#define CLOCK_DIVIDE
//#define INITIAL_EEPROM

#ifdef CLOCK_DIVIDE
 #define DIV_FACTOR 2
 #define BT_SPEED 115200 //215000 //9600 // 38400
#else
 #define DIV_FACTOR 1
 #define BT_SPEED 57600 //115200 //9600 // 38400
#endif

#define MAX_VOLUME 800
#define MED_VOLUME 400
#define MIN_VOLUME 150

#define adr_buzz_size_array       0*2
#define adr_buzz_up_thres         1*2
#define adr_buzz_down_thres       2*2
#define adr_buzz_up_start_freq    3*2
#define adr_buzz_down_start_freq  4*2
#define adr_buzz_volume           5*2
#define adr_pwdown_time           6*2
#define adr_freq_increment        7*2
#define adr_count_flights         8*2
#define adr_send_average          9*2
#define adr_hold_button          10*2
#define adr_battery_alarm_level  11*2
#define adr_display_temp         12*2
#define adr_buzz_always          13*2
#define adr_flight_stop_filter   14*2
#define adr_flight_start_filter  15*2
#define adr_flight_time          16*2
#define adr_total_flight_time    17*2
#define adr_battery_calibration  18*2
#define adr_total_working_time   19*2


#define CORRECT_PWM 1
#define FAST_PWM 0

/*
	Библиотека для расширенной генерации ШИМ на ATmega328 (Arduino UNO/Nano/Pro Mini...)
	Разработчики: Egor Zaharov и AlexGyver
	Версия библиотеки 1.4 от 16.09.2019
*/

// ============== Функции для расширенной генерации ШИМ сигнала ==============

// Данные функции убирают один ШИМ выход у 8-ми битных таймеров, оставляя нам ШИМ пины D3, D5, D9 и D10 на ATmega328

void PWM_frequency(byte pin, long freq, uint8_t correct);
/*	PWM_freqency(пин, частота, режим) - запустить ШИМ с выбранной частотой
	- Пины: D3 (таймер 2), D5 (таймер 0 - сломает millis/delay), D9 и D10 (таймер 1)
	- Режим: 0 (FAST_PWM), 1 (CORRECT_PWM)
	- Частота: 250-200'000 Гц для всех таймеров
	- Для изменения заполнения используй PWM_set
		- Разрядность в этом режиме приведена к 8 битам, на деле шаги изменения разные!
*/

void PWM_resolution(byte pin, byte res, uint8_t correct);
/*	PWM_resolution(пин, разрядность, режим) - запустить ШИМ с выбранной разрядностью
	- Пины: D3 (таймер 2), D5 (таймер 0 - сломает millis/delay), D9 и D10 (таймер 1)
	- Режим: 0 (FAST_PWM), 1 (CORRECT_PWM)
	- Разрешение: D3 (4-8 бит), D5 (4-8 бит), D9 и D10 (4-16 бит)
	- Частота в этом режиме выбирается автоматически максимальная согласно возможностям таймера (см. таблицу)
	- Для изменения заполнения используй PWM_set
		- Пределы заполнения для разной разрядности указаны в таблице
*/

void PWM_set(byte pin, unsigned int duty);
/*	PWM_set(пин, заполнение) - изменить заполнение на выбранном пине
	- Пин: D3, D5, D6, D9, D10, D11
	- Заполнение: зависит от разрешения и режима (см. таблицу)
		- При использовании PWM_frequency разрядность составляет 8 бит (0-255)
		- При использовании PWM_resolution макс. значение заполнения равно (2^разрядность - 1), также смотри таблицу
*/

void PWM_detach(byte pin);		// отключает ШИМ на выбранном пине (позволяет использовать digital Read/Write)
void PWM_attach(byte pin);		// подключает ШИМ на выбранном пине (с последними настройками)
void PWM_default(byte pin);		// сброс настроек соответствующего пину таймера на "стандартные" для Arduino

void PWM_16KHZ_D3(byte duty);
/* 	Запуск ШИМ с частотой 16 кГц на пине D3
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 8 битам (заполнение 0-255)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_20KHZ_D3(byte duty);
/* 	Запуск ШИМ с частотой 20 кГц на пине D3
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 8 битам (заполнение 0-255)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_16KHZ_D5(byte duty);
/* 	Запуск ШИМ с частотой 16 кГц на пине D5
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 8 битам (заполнение 0-255)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_20KHZ_D5(byte duty);
/* 	Запуск ШИМ с частотой 20 кГц на пине D5
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 8 битам (заполнение 0-255)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_16KHZ_D9(int duty);
/* 	Запуск ШИМ с частотой 16 кГц (15.6 кГц) на пине D9
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность ровно 10 бит (заполнение 0-1023)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_20KHZ_D9(int duty);
/* 	Запуск ШИМ с частотой 20 кГц на пине D9
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 10 битам (заполнение 0-1023)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_16KHZ_D10(int duty);
/* 	Запуск ШИМ с частотой 16 кГц (15.6 кГц) на пине D10
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность ровно 10 бит (заполнение 0-1023)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */

void PWM_20KHZ_D10(int duty);
/* 	Запуск ШИМ с частотой 20 кГц на пине D10
	- Отменяет настройки PWM_frequency/PWM_resolution
	- Разрядность приведена к 10 битам (заполнение 0-1023)
	- Заполнение меняет сама (не нужно вызывать PWM_set) */
	
float PWM_square_D9(float frequency);
/*	Генератор меандра (квадратная волна) на пине D9
	Частота от 2 Гц до 8 Мгц, шаг частоты зависит от частоты
	(начиная с 0.01 Гц и заканчивая десятками кГц!!!)
	Отключить можно вызвав PWM_detach(9);
	Для сброса таймера в режим по-умолчанию - PWM_default(9);
	Возвращает установленную частоту в герцах!
	
	Частота		Погрешность
	300 Гц		0.01 Гц
	700 Гц		0.05 Гц
	900 ГЦ		0.1 Гц
	2 кГц		0.5 Гц
	3 кГц		1 Гц
	4 кГц		2 Гц
	9 кГц		10 Гц
	20 кГц		50 Гц
	30 кГц		110 Гц
	63 кГц		500 Гц
	90 кГц		1000 Гц
*/

/*
	============= Таблица №1 частот для расширенной генерации ШИМ (PWM_resolution) =============
 _________________________________________________________________________________________________________________________
|Разрядность, бит   |4      |5      |6      |7      |8     |9       |10      |11     |12     |13     |14    |15    |16    |
|___________________|_______|_______|_______|_______|______|________|________|_______|_______|_______|______|______|______|
|Макс. значение duty|15     |31     |63     |127    |255   |511     |1023    |2047   |4095   |8191   |16383 |32767 |65535 |
|___________________|_______|_______|_______|_______|______|________|________|_______|_______|_______|______|______|______|
|Fast   | Пины 3, 5 |1 МГц  |516 кГц|254 кГц|126 кГц|63 кГц|-       |-       |-      |-      |-      |-     |-     |-     |
|PWM    | 9, 10     |1 МГц  |516 кГц|254 кГц|126 кГц|63 кГц|31.2 кГц|15.6 кГц|7.8 кГц|3.9 кГц|1.9 кГц|980 Гц|488 Гц|244 Гц|
|_______|___________|_______|_______|_______|_______|______|________|________|_______|_______|_______|______|______|______|
|Correct| Пины 3, 5 |571 кГц|266 кГц|129 кГц|63 кГц |32 кГц|-       |-       |-      |-      |-      |-     |-     |-     |
|PWM    | 9, 10     |571 кГц|266 кГц|129 кГц|63 кГц |32 кГц|15.7 кГц|7.8 кГц |3.9 кГц|1.9 кГц|976 Гц |488 Гц|244 Гц|122 Гц|
|_______|___________|_______|_______|_______|_______|______|________|________|_______|_______|_______|______|______|______|
*/

// ============ Функции для настройки стандартной генерации ШИМ сигнала (analogWrite) ============

// Данные функции НЕ убирают один ШИМ выход у 8-ми битных таймеров, можно использовать все 6 ШИМ пинов с настроенной частотой! См. таблицу.

void PWM_prescaler(byte pin, byte mode);
/*	PWM_prescaler(пин, режим) - установить предделитель таймера (меняет частоту ШИМ)
	- Пин: D3, D5, D6, D9, D10, D11
	- Режим: 1-7, см. таблицу частот
*/

void PWM_mode(byte pin, uint8_t mode);
/*	PWM_mode(пин, режим) - установить режим генерации ШИМ
	- Пин: D3, D5, D6, D9, D10, D11
	- Режим: 0 - FastPWM, 1 - Phase-correct, см. таблицу частот
*/

void PWM_TMR1_8BIT();	// Установить таймер 1 (ШИМ на D9 и D10) в режим 8 бит. См. таблицу частот

void PWM_TMR1_10BIT();	// Установить таймер 1 (ШИМ на D9 и D10) в режим 10 бит. См. таблицу частот

/*
	========== Таблица №2 частот для стандартной генерации ШИМ (PWM_prescaler) ==========
	
	Timer 0 по умолчанию работает в режиме Fast PWM 
	Timer 1 и 2 по умолчанию работают в режиме Phase-correct
 _______________________________________________________________________________________________
|       | Timer0 (пины 5 и 6) 8 bit | Timer 1 (пины 9 и 10) 10 bit  | Timer2 (пины 3 и 11) 8 bit|
|       | Timer1 (пины 9 и 10) 8 bit|                               |                           |
|       |___________________________|_______________________________|___________________________|
|mode   | Phase-correct | Fast PWM  | Phase-correct     | Fast PWM  | Phase-correct | Fast PWM  |
|_______|_______________|___________|___________________|___________|_______________|___________|
|1      | 31.4 kHz      | 62.5 kHz  | 7.8 kHz           | 15.6 kHz  | 31.4 kHz      | 62.5 kHz  |
|2      | 4 kHz         | 7.8 kHz   | 977 Hz            | 2 kHz     | 4 kHz         | 8 kHz     |
|3      | 490 Hz        | 976 Hz    | 122 Hz            | 244 Hz    | 980 Hz        | 2 kHz     |
|4      | 122 Hz        | 244 Hz    | 30 Hz             | 61 Hz     | 490 Hz        | 980 Hz    |
|5      | 30 Hz         | 61 Hz     | 7.6 Hz            | 15 Hz     | 245 Hz        | 490 Hz    |
|6      | -             | -         | -                 | -         | 122 Hz        | 244 Hz    |
|7      | -             | -         | -                 | -         | 30 Hz         | 60 Hz     |
|_______|_______________|___________|___________________|___________|_______________|___________|
*/


/* **************************** my funcs ********************************** */
void freq_shift_on ();
void freq_shift_off ();
void freq_shift ();
void disable_timer ();
void buzz_end_of_flight(int in_dat);
// void buzz_start_of_flight(int in_dat);
void buzz_pwdown(int in_dat);

const char welcome_message[] PROGMEM = {"\nWelcome to programming mode!\nYou can send the command \"help\"\n"};
const char help_message[] PROGMEM = {"\
In this mode you can set parameters.\n\n\
Version: 1.5\n\
up_freq = log10(vario)*200.0+buzz_up_start_freq\n\
down_freq = buzz_down_start_freq - log10(|vario+100|)*150.0\n\n\
p0 - buzz_size_array (20) [1-100]\n\
p1 - buzz_up_thres, cm/s (20)\n\
p2 - buzz_down_thres, cm/s (-150)\n\
p3 - buzz_up_start_freq, Hz (400)\n\
p4 - buzz_down_start_freq, Hz (300)\n\
p5 - buzz_volume (200) [0-800]\n\
p6 - pwdown_time, min (60)\n\
p7 - freq_increment, (2)\n\
p9 - send_average, (1)\n\
p10 - hold_button, ms (0) [0-10000]\n\
p11 - battery_alarm_level, % (20)\n\
p12 - display_temp (0)\n\
p13 - buzz_always (0)\n\
p14 - flight_stop_filter, s (60)\n\
p15 - flight_start_filter, ms (4000)\n\
p16 - flight_time, min\n\
p17 - total_flight_time, min\n\
p19 - total_working_time, min\n\
Example: \"p0=20\"\n\n\
\"default\": reset to default values\n\n\
\"calibrate\": use this command after fully charging the battery\n\n\
\"report\": report current values\n\n\
\"sleep\": power down\n\n\
\"exit\": exit without save\n\n\
\"save\": save & exit\n"};

const byte moo_message[] PROGMEM = {32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 40, 95, 95, 41, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 40, 111, 111, 41, 10, 32, 32, 32, 32, 47, 45, 45, 45, 45, 45, 45, 92, 47, 10, 32, 32, 47, 32, 124, 32, 32, 32, 32, 32, 32, 124, 124, 10, 32, 42, 32, 47, 92, 45, 45, 45, 47, 92, 10, 32, 126, 126, 126, 126, 126, 126, 126, 126, 10, 46, 46, 46, 34, 72, 97, 118, 101, 32, 121, 111, 117, 32, 109, 111, 111, 101, 100, 32, 116, 111, 100, 97, 121, 63, 34, 46, 46, 46};
const byte paramoo_message[] PROGMEM = {95, 46, 46, 126, 126, 42, 42, 42, 42, 126, 126, 46, 46, 95, 10, 32, 32, 32, 32, 32, 92, 47, 32, 32, 32, 32, 32, 32, 32, 32, 92, 47, 10, 32, 32, 32, 32, 32, 32, 32, 92, 32, 32, 32, 32, 32, 32, 40, 95, 95, 41, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 92, 32, 32, 32, 32, 40, 45, 32, 45, 41, 10, 32, 32, 32, 42, 45, 45, 45, 44, 45, 45, 45, 45, 45, 45, 92, 47, 10, 32, 32, 32, 32, 32, 32, 32, 32, 124, 32, 32, 32, 32, 32, 32, 124, 124, 10, 32, 32, 32, 32, 32, 32, 32, 47, 92, 45, 45, 45, 47, 92};

/* ********************************************************** */

extern bool flight;
extern int battery_tr_pin;
extern float batt_average;
extern int battery_pin;

extern int buzz_size_array;
extern int buzz_up_thres;
extern int buzz_down_thres;
extern int buzz_up_start_freq;
extern int buzz_down_start_freq;
extern int buzz_volume;
extern unsigned int pwdown_time;
extern int freq_increment;
// extern int ;
extern int send_average;
extern unsigned int hold_button;
extern int battery_alarm_level;
extern int display_temp;
extern int buzz_always;
extern int flight_total;
extern int flight_last;
// extern int ;
extern int flight_time;
extern unsigned int total_flight_time;
extern unsigned int count_flights;
extern unsigned int flight_stop_filter;
extern unsigned int flight_start_filter;
extern int battery_calibration;
extern unsigned int working_time, total_working_time;
extern volatile unsigned int update_freq;
extern volatile int fshift;

void update_int(int addr, int val);
void update_params();
int read_int(int addr);
void read_params();
void default_params();
void freq_shift_on();
void freq_shift_off();

/* ********************************************************** */

float read_voltage();
// void wait(int del);

#endif


