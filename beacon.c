/* ----------------------------------------------------
 * File    : beacon.c
 * Title   : GSM-GPS tracking thing
 * Author  : Alex Weber, alex@tinkerlog.com, http://tinkerlog.com
 * Hardware: ATmega8, 4.096MHz
 * Software: WinAVR20070525
 *
 * Disclaimer:
 * This software comes "as is". Without any warranty.
 * 
 * License:
 * This is published under the Creative Commons License Attribution-NonCommercial.
 * You are free to distribute and adapt for non-commercial use.
 * See http://creativecommons.org/licenses/by-nc/3.0/ for more details.
 *  
 * History:
 * 2007/07/12	GPS position received from the GM862. Not parsed.
 * 2007/07/28	First SMS with link to google maps sent.
 * 2007/08/08	Sending SMS with positions every 2 minutes
 * 
 * 
 * Known issues:
 * - There is still a bug in the UART routines. Sending may hang with 
 *   large messages. For now it works.
 * 
 * Notes:
 * - Search for EDIT THIS in the source to find part that you need to adapt
 *   to your setting (PIN and SMS number). 
 * 
 * TODO:
 * - Use PIN and SMS from eeprom
 * 
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "uart.h"
#include "suart.h"

#define TRUE 1
#define FALSE 0

#define MONITOR_LED PD4
#define MODEM_ON_SWITCH PC5


// setup a file for printf usage
FILE uart_file = FDEV_SETUP_STREAM(suart_putc_f, suart_getc_f, _FDEV_SETUP_RW);


// types
typedef struct {
	const char *name;
	unsigned char command;
	void (*work)(void);
} menu_item;

typedef struct {
	uint8_t lat_deg;
	uint32_t lat_min;
	uint8_t lon_deg;
	uint32_t lon_min;
	int32_t alt;
	uint8_t fix;
} gps_position;


// EEPROM storage
uint16_t reboot_counter EEMEM = 0;
uint8_t sms_gateway[15] EEMEM = "7676245";		// EDIT THIS! Where to send the SMS
uint8_t pin[5] EEMEM = "1067";					// EDIT THIS! Insert your pin

// statics

#define MODEM_OFF			0
#define MODEM_STARTING		1
#define MODEM_ON			2
#define MODEM_SHUTTING_DOWN	3
#define MODEM_INITIALIZED	4
#define MODEM_POS_FIX		5
#define MODEM_ERRORED		6

static volatile uint8_t modem_state = 0;

static volatile uint8_t seconds = 0;
static volatile uint8_t minutes = 0;
static volatile uint8_t hours = 0;
static volatile uint8_t days = 0;
static volatile uint16_t ms_count = 0;

// current gps position
static volatile gps_position act_gps_position;

// strings
const char cr_P[] PROGMEM = "\n";
const char line_P[] PROGMEM = "------\n";
const char firmware_P[] PROGMEM = "Beacon v0.03 2007/08/06";
const char menu_option_P[] PROGMEM = " - ";
const char prompt_P[] PROGMEM = "key>";
const char error_unknown_command_P[] PROGMEM = "unknown key: %c\n";
const char got_P[] PROGMEM = "got: %s\n";
const char google_maps_P[] PROGMEM = "alex@tinkerlog.com http://maps.google.com/maps?q=%d.%lu,%d.%lu%%28Alex%%29&t=k&z=16";
const char position_P[] PROGMEM = "%d.%lu %d.%lu %li\n";
const char ok_P[] PROGMEM = " --> OK\n";
const char no_response_P[] PROGMEM = " --> NO RESPONSE\n";
const char no_fix_P[] PROGMEM = "no fix\n";
const char error_P[] PROGMEM = "ERROR\n";
const char interactive_P[] PROGMEM = "INTERACTIVE\n";
const char run_P[] PROGMEM = "RUN\n";
const char wait_P[] PROGMEM = "waiting ...\n";
// menu strings
const char report_P[] PROGMEM = "Report status";
const char modem_state_P[] PROGMEM = "Modem %d\n";
const char change_sms_P[] PROGMEM = "Change sms phone number";
const char menu_P[] PROGMEM = "Menu"; 
const char change_pin_P[] PROGMEM = "Change pin";
const char switch_on_P[] PROGMEM = "Switch modem on/off";
const char init_modem_P[] PROGMEM = "Init modem";
const char send_sms_P[] PROGMEM = "Send SMS";
const char cold_gps_P[] PROGMEM = "Cold start GPS";
const char request_gps_P[] PROGMEM = "Request GPS";
const char reboots_P[] PROGMEM = "reboots: %d\n";
const char sms_gateway_P[] PROGMEM = "SMS gateway: %s\n";
const char pin_P[] PROGMEM = "PIN: %s\n";
const char uptime_P[] PROGMEM = "uptime: %02dD %02d:%02d:%02d\n";
// modem strings
const char AT_P[] PROGMEM       = "AT";					// say hello
const char OK_P[] PROGMEM       = "\r\nOK\r\n";			// response
const char ATIPR_P[] PROGMEM    = "AT+IPR=19200";		// use 19200 baud
const char ATCPIN_P[] PROGMEM   = "AT+CPIN=1067";		// EDIT THIS! send pin
const char ATCMEE_P[] PROGMEM   = "AT+CMEE=2";			// extended error format
const char ATCMGF_P[] PROGMEM   = "AT+CMGF=1";			// sms text mode
const char ATCSCS_P[] PROGMEM	= "AT+CSCS?";			// select char set
const char ATCMGS_P[] PROGMEM   = "AT+CMGS=\"7676245\"";	// EDIT THIS! send sms
//const char ATCREG_P[] PROGMEM   = "AT+CREG?";			// network?
//const char ATGPSR_P[] PROGMEM   = "AT$GPSR=1";		// reset GPS, cold start
const char ATGPSR_P[] PROGMEM   = "AT$GPSR=2";			// reset GPS, warm start
const char ATGPSACP_P[] PROGMEM = "AT$GPSACP";			// position request


// sequence of commands to initialize the modem
const char *MODEM_INIT_SEQ[] = {
	AT_P, ATIPR_P, ATCPIN_P, ATCMEE_P };


// prototypes
void inc_reboot_counter(void);
void switch_led(void);
void clock_tick(void);
void show_report(void);

void show_menu(void);
void change_sms(void);
void change_pin(void);
uint8_t handle_command(uint8_t command);

uint8_t uart_gets_timeout(char *buf, uint16_t timeout);
void switch_modem(void);
void init_modem(void);
void send_sms(char *message);
void send_position_sms(void);
void network_status(void);
void cold_gps(void);
void request_gps(void);
char *skip(char *str, char match);
char *read_token(char *str, char *buf, char delimiter);
void parse_gps(char *gps_msg);


/*
 * Increments the reboot counter.
 */
void inc_reboot_counter(void) {
	uint16_t mem_reboots = eeprom_read_word(&reboot_counter);
	mem_reboots++;
	eeprom_write_word(&reboot_counter, mem_reboots);
}


/*
 * Switches the modem on and off.
 */
void switch_modem(void) {
	uint8_t i = 0;

	PORTC |= (1 << MODEM_ON_SWITCH);
	for (i = 0; i < 200; i++) {
		_delay_ms(10);
	}
	PORTC &= ~(1 << MODEM_ON_SWITCH);
	if (modem_state == MODEM_OFF) {
		modem_state = MODEM_ON;
	}
}



/*
 * Sends a request to the modem and stores the response into
 * the buffer.
 */
uint8_t request_modem(
		const char *command, uint16_t timeout, uint8_t check, char *buf) {
			
	uint8_t count = 0;
	char *found = 0;

	printf_P(command);
	
	uart_puts_P(command);
	uart_putc('\r');
	count = uart_gets_timeout(buf, timeout);
	if (count) {
		if (check) {
			found = strstr_P(buf, OK_P);
			if (found) {
				printf_P(ok_P);
			}
			else {
				printf("%d, %s\n", count, buf);  
				//printf_P(got_P, buf);
			} 
		}
		else {
			printf("%d, %s\n", count, buf);  
			//printf_P(cr_P);
		}
	}
	else {
		printf_P(no_response_P);
	}
	return count;
}



/*
 * Gets a response from the modem.
 */
uint8_t uart_gets_timeout(char *buf, uint16_t timeout) {
	uint8_t count = 0;
	uint8_t us_count = 0;
	uint16_t c;
	*buf = 0;
	while (timeout) {
		c = uart_getc();
		if (c == UART_NO_DATA) {
			_delay_us(10);
			if (us_count++ == 100) {
				timeout--;
				us_count = 0;
			}
		}
		else {
			count++;
			*buf++ = c;
		}
	}
	if (count != 0) {
		*buf = 0;
		count++;
	}
	return count;
}



/*
 * Initializes the modem with an initialization sequence.
 * TODO: send the pin from EEPROM
 */
void init_modem(void) {
	char buf[100];
	uint8_t i = 0;
	for (i = 0; i < 4; i++) {
		if (!request_modem(MODEM_INIT_SEQ[i], 1000, TRUE, buf)) {
			modem_state = MODEM_ERRORED;
			return;
		}
	}
	modem_state = MODEM_INITIALIZED;
}



/*
 * Sends an SMS.
 */
void send_sms(char *message) {
	char buf[200];

	request_modem(ATCMGF_P, 1500, TRUE, buf);
	request_modem(ATCMGS_P, 1500, FALSE, buf);		// TODO: use phone number from eeprom
	printf_P(got_P, buf);
	uart_puts(message);
	uart_putc(0x1a);
	//uart_putc(0x0d);
	uart_gets_timeout(buf, 2000);
	printf_P(got_P, buf);
	uart_gets_timeout(buf, 4000);
	printf_P(got_P, buf);
	
}



/*
 * Sends the current position via SMS if it is a valid (fix) 
 * position.
 */
void send_position_sms(void) {
	
	char buf[200];	
	if (act_gps_position.fix > 0) {
		sprintf_P(buf, google_maps_P,  
			act_gps_position.lat_deg, act_gps_position.lat_min,
			act_gps_position.lon_deg, act_gps_position.lon_min
		);
		send_sms(buf);
	}
	
	//send_sms("alex@tinkerlog.com dies ist ein test und der geht uber eine ganze menge zeichen");	
}

/*
void network_status(void) {
	char buf[100];
	request_modem(ATCREG_P, 1000, FALSE, buf);
	printf_P(got_P, buf);			
}
*/



/*
 * Starts the GPS over.
 */
void cold_gps(void) {
	char buf[100];
	request_modem(ATGPSR_P, 2000, TRUE, buf);
	printf_P(got_P, buf);			
}



/*
 * Request the GPS position. 
 * Position is parsed and converted and stored globally.
 */
void request_gps(void) {
	char buf[150];
	request_modem(ATGPSACP_P, 4000, FALSE, buf);
	printf_P(got_P, buf);
	if (strlen(buf) > 29) {
		act_gps_position.fix = 0;	// invalidate actual position			
		parse_gps(buf);
		if (act_gps_position.fix > 0) {
			printf_P(position_P,  
				act_gps_position.lat_deg, act_gps_position.lat_min,
				act_gps_position.lon_deg, act_gps_position.lon_min, 
				act_gps_position.alt);
			modem_state = MODEM_POS_FIX;
		}
	}
	else {
		act_gps_position.fix = 0;
		printf_P(no_fix_P);
		modem_state = MODEM_INITIALIZED;
	}
}



/*
 * Skips the string until the given char is found.
 */
char *skip(char *str, char match) {
	uint8_t c = 0;
	while (TRUE) {
		c = *str++;
		if ((c == match) || (c == '\0')) {
			break;
		}
	}
	return str;
}



/*
 * Reads a token from the given string. Token is seperated by the 
 * given delimiter.
 */
char *read_token(char *str, char *buf, char delimiter) {
	uint8_t c = 0;
	while (TRUE) {
		c = *str++;
		if ((c == delimiter) || (c == '\0')) {
			break;
		}
		else if (c != ' ') {
			*buf++ = c;
		}
	}
	*buf = '\0';
	return str;
}



/*
 * Parse and convert the given string into degrees and minutes.
 * Example: 5333.9472N --> 53 degrees, 33.9472 minutes
 * converted to: 53.565786 degrees 
 */
void parse_degrees(char *str, uint8_t *degree, uint32_t *minutes) {
	char buf[6];
	uint8_t c = 0;
	uint8_t i = 0;
	char *tmp_str;
	
	tmp_str = str;
	while ((c = *tmp_str++) != '.') i++;
	strlcpy(buf, str, i-1);
	*degree = atoi(buf);
	 	
	tmp_str -= 3;
	i = 0;
	while (TRUE) {
		c = *tmp_str++;
		if ((c == '\0') || (i == 5)) {
			break;
		}
		else if (c != '.') {
			buf[i++] = c;
		}
	}
	buf[i] = 0;
	*minutes = atol(buf);
	*minutes *= 16667;
	*minutes /= 1000;
}



/*
 * Parse and convert the position tokens. 
 */
void parse_position(gps_position *pos, char *lat_str, char *lon_str, char *alt_str) {
	char buf[10];
	parse_degrees(lat_str, &pos->lat_deg, &pos->lat_min);
	parse_degrees(lon_str, &pos->lon_deg, &pos->lon_min);
	read_token(alt_str, buf, '.');
	pos->alt = atol(buf);
}



/*
 * Parse the given string into a position record.
 * example:
 * $GPSACP: 120631.999,5433.9472N,00954.8768E,1.0,46.5,3,167.28,0.36,0.19,130707,11\r
 */
void parse_gps(char *gps_msg) {

	char time[7];
	char lat_buf[12];
	char lon_buf[12];
	char alt_buf[7];
	char fix;
	char date[7];
	char nr_sat[4];
	
	gps_msg = skip(gps_msg, ':');					// skip prolog
    gps_msg = read_token(gps_msg, time, '.');		// time, hhmmss
	gps_msg = skip(gps_msg, ',');					// skip ms
	gps_msg = read_token(gps_msg, lat_buf, ',');	// latitude
	gps_msg = read_token(gps_msg, lon_buf, ',');	// longitude
	gps_msg = skip(gps_msg, ',');					// hdop
	gps_msg = read_token(gps_msg, alt_buf, ',');	// altitude
	fix = *gps_msg++;								// fix, 0, 2d, 3d
	gps_msg++;
	gps_msg = skip(gps_msg, ',');					// cog, cource over ground
	gps_msg = skip(gps_msg, ',');					// speed [km]
	gps_msg = skip(gps_msg, ',');					// speed [kn]
	gps_msg = read_token(gps_msg, date, ',');		// date ddmmyy
	gps_msg = read_token(gps_msg, nr_sat, '\n');	// number of sats

	if (fix != '0') {
		parse_position(&act_gps_position, lat_buf, lon_buf, alt_buf);
		act_gps_position.fix = fix;
	}

}


/*
 * Switch the debug led on and off.
 */
void switch_led(void) {
	static uint8_t led_on = FALSE;
	if (led_on) {
		PORTD &= ~(1 << MONITOR_LED);
		led_on = FALSE;
	}
	else {
		PORTD |= (1 << MONITOR_LED);
		led_on = TRUE;
	}
}



/*
 * SIGNAL TIMER0_OVF_vect
 * Handles overflow interrupts of timer 0.
 * With prescaler 8 it is calles with 2000 Hz.
 */
SIGNAL(TIMER0_OVF_vect) {
	// count ms
	if (++ms_count == 2000) {
		clock_tick();
		ms_count = 0;
	}
}



/*
 * clock_tick
 * Adds a second to the clock.
 */
void clock_tick(void) {
	seconds++;
	if (seconds == 60) {
		seconds = 0;
		minutes++;
		if (minutes == 60) {
			minutes = 0;
			hours++;
			if (hours == 24) {
				hours = 0;
				days++;
			}
		}
	}
}



/* ---------------------------------
 * menu definition
 */
#define MAX_MENU_ITEMS 9
const menu_item menu[] = {
	{menu_P, 'm', show_menu},
	{report_P, 'r', show_report},
	{change_sms_P, 'a', change_sms},
	{change_pin_P, 'p', change_pin},
	{switch_on_P, 'o', switch_modem},
	{init_modem_P, 'i', init_modem},
	//{network_status_P, 'n', network_status},
	{send_sms_P, 's', send_position_sms},
	{cold_gps_P, 'c', cold_gps},
	{request_gps_P, 'g', request_gps}
}; 



/*
 * Displays the menu.
 */
void show_menu(void) {
	uint8_t i = 0;
	
	printf_P(cr_P);
	printf_P(cr_P);
	printf_P(line_P);
	printf_P(firmware_P); printf_P(cr_P);
	
	for (i = 0; i < MAX_MENU_ITEMS; i++) {
		printf("%c", menu[i].command);
		printf_P(menu_option_P);
		printf_P(menu[i].name);
		printf_P(cr_P); 
	}
	printf_P(line_P);
	printf_P(prompt_P);	
	
}



/*
 * Handles incomming commands.
 * Tries to find a menu entry for the given command char.
 * If a menu was found, the function gets executed.
 */
uint8_t handle_command(uint8_t command) {
	uint8_t i = 0;

	printf_P(cr_P);
	
	for (i = 0; i < MAX_MENU_ITEMS; i++) {
		if (command == menu[i].command) {
			printf_P(menu[i].name);
			printf_P(cr_P);
			if (menu[i].work) {
				menu[i].work();
			}
			break;
		}
	}
	if (i == MAX_MENU_ITEMS) {
		printf_P(error_unknown_command_P, command);
	}
	
	return 0;
}



/*
 * Displays all status informations.
 */
void show_report(void) {
	char buffer[30];
	
	uint16_t mem_reboots = eeprom_read_word(&reboot_counter);
	printf_P(reboots_P, mem_reboots);
	
	printf_P(uptime_P, days, hours, minutes, seconds);	
	
	eeprom_read_block(buffer, sms_gateway, 15);
	printf_P(sms_gateway_P, &buffer);

	eeprom_read_block(buffer, pin, 5);
	printf_P(pin_P, &buffer);
	
	printf_P(modem_state_P, modem_state);
	
}



/*
 * Reads a value from eeprom, reads user input and stores the
 * value back to eeprom.
 */
void modify_str(char* str) {
	char buffer[30];
	uint8_t i;
	uint16_t c;
	eeprom_read_block(buffer, str, 15);
	printf("old value: %s\r\n>", buffer);
	for (i = 0; i < 15; i++) {
		while ((c = uart_getc()) == UART_NO_DATA) {}
		if (c == '\r') {
			break;
		}
		buffer[i] = c;
	}
	buffer[i] = 0;
	eeprom_write_block(buffer, str, 15);
}



/*
 * change_sms
 */
void change_sms(void) {
	modify_str((char*)sms_gateway);
}

/*
 * Change the pin.
 */
void change_pin(void) {
	modify_str((char*)pin);
}


#define MODE_NONE			0
#define MODE_SWITCH_MODEM	1
#define MODE_INIT_MODEM		2
#define MODE_REQUEST_GPS	3
#define MODE_SEND_POSITION	4
#define MODE_WAIT			100
#define MODE_WAIT2			101
#define MODE_ERRORED		102
#define MODE_STOP			103


int main(void) {
	
	uint8_t i = 0;
	uint16_t command = 0;
	uint8_t mode = MODE_NONE;
	uint8_t next_mode = MODE_NONE;
	uint8_t wakeup_seconds = 0;
	uint8_t wakeup_minutes = 0;

	// timer 0 setup, prescaler 8
	TCCR0 |= (1 << CS11);
	// enable timer 0 interrupt
	TIMSK |= (1 << TOIE0);	
	

	// enable monitor led
	DDRD |= (1 << MONITOR_LED);
	
	// enable on/off switch
	DDRC |= (1 << MODEM_ON_SWITCH);
	PORTC &= ~(1 << MODEM_ON_SWITCH);

	// say hello
	for (i = 0; i < 10; i++) {
		switch_led();
		_delay_ms(100);
	}
	for (i = 0; i < 200; i++) {
		_delay_ms(100);
	}
	switch_led();
	
	inc_reboot_counter();
	init_uart();	
	init_suart();
	sei();
	
	stdout = stdin = &uart_file;
		
	while (TRUE) {
		
		switch (mode) {
			case MODE_NONE:
				show_menu();
				mode = MODE_SWITCH_MODEM;
				break;
			case MODE_SWITCH_MODEM:
				printf_P(switch_on_P); printf_P(cr_P);
				switch_modem();
				//mode = MODE_WAIT;
				mode = MODE_STOP;
				next_mode = MODE_INIT_MODEM;
				wakeup_seconds = (seconds + 10) % 60;
				break;
			case MODE_INIT_MODEM:
				printf_P(init_modem_P); printf_P(cr_P);
				init_modem();
				if (modem_state == MODEM_ERRORED) {
					next_mode = MODE_ERRORED;
				}
				else {
					next_mode = MODE_REQUEST_GPS;
				}
				mode = MODE_WAIT;
				wakeup_seconds = (seconds + 15) % 60;
				break;
			case MODE_REQUEST_GPS:
				request_gps();
				if (modem_state == MODEM_POS_FIX) {
					mode = MODE_SEND_POSITION;
				}
				else {
					mode = MODE_WAIT;
					next_mode = MODE_REQUEST_GPS;
					wakeup_seconds = (seconds + 15) % 60;				
				}
				break;
			case MODE_SEND_POSITION:
				send_position_sms();
				wakeup_seconds = seconds;
				wakeup_minutes = (minutes + 2) % 60;
				mode = MODE_WAIT2;
				next_mode = MODE_REQUEST_GPS;
				break;
			case MODE_WAIT:
				if (seconds == wakeup_seconds) {
					mode = next_mode;
				}
				break;
			case MODE_WAIT2:
				if ((seconds == wakeup_seconds) && (minutes == wakeup_minutes)) {
					mode = next_mode;
				}
				break;
			case MODE_ERRORED:
				printf_P(error_P);
				mode = MODE_SWITCH_MODEM;				
				break;
			case MODE_STOP:
				break;
			default:
				break;
		}
	
		
		command = suart_getc_nowait();
		switch (command) {
			case -1:
			case '\r':
			case '\n':
				break;
			case '1':
				if (mode == MODE_WAIT) {
					mode = MODE_STOP;
				}
				else {
					next_mode = mode;
					mode = MODE_STOP;
				}
				printf_P(interactive_P);
				break;
			case '2':
				mode = next_mode;
				next_mode = MODE_NONE;
				printf_P(run_P);
				break; 
			default:
				handle_command(command);
				break;
		}
		
	}
	
	return 0;
	
}
