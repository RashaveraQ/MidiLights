// Mr.Midi 2

// Hardware:
// ATmega8
// No crystal required (8MHz internal clock is sufficient)
// Uses UART for MIDI-IO
// SPI-IO plus extra Pin (CS) for SD-card
// Control buttons on PIND2-7
// LCD on PORTC

// Circuit Description:
/*
Take the circuit from Mr.Midi2
available on www.mikrocontroller.net!
*/

// Software:
// MMC-Driver from alp_mp3 - Open Source Atmel AVR based MP3 Player (modified, though)
// My own MIDI-File-Parser (Format 0 only)
// Compiles with WinAVR plus AVRStudio 4

// Further Optimizations:
// byte-shift->union: 2 Bytes


/*TODO
- Record bleibt am Schluss hängen bei USE_RTC
- Midi-Protokoll ist irgendwie anders (Evanescence-Lied)
*/

#define __AVR_ATmega168__	// akashi

#include "mrmidi2.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef LED_DISP
	#include <avr/eeprom.h>
#else
	#define eeprom_read_byte(a) 0
	#define eeprom_read_word(a) 0
	#define eeprom_write_byte(a, b)
	#define eeprom_write_word(a, b)
#endif
#include <avr/wdt.h>
#ifdef RANDOMSONG
	#include <stdlib.h>
#endif
#ifndef LED_DISP
	#include "lcd.h"
#else
	#define lcd_data(a)
	#define lcd_setcur(a, b)
	#define lcd_data(a)
	#define lcd_string(a, b)
	#define lcd_init()
	#define lcd_hex_u08(a)
	#define lcd_hex_u16(a)
	#define lcd_number(a, b)
	#define lcd_number_k(a)
	#define lcd_nibble(a)
	#define send_all_off(a)
#endif
#include "mmc.h"
#include "spi.h"
#include "delay.h"
#include "fat16.h"
#include "types.h"

#ifdef __AVR_ATmega168__
#ifdef RC5
	#include "rc5.h"
#endif

#ifdef USE_RTC
	#include "clock.h"
#endif

//ATmega168 - Umbenennungen
#define UDR UDR0
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define UDRE UDRE0
#define RXEN RXEN0
#define TXEN TXEN0
#define RXCIE RXCIE0
#define OCR2 OCR2A
#define TCCR0 TCCR0B
#define TCCR2 TCCR2B
#endif

#ifndef F_CPU
	#define F_CPU 8000000UL
#endif

// RTC Timer 2
#define RTC_PRESCALER		256
#define RTC_RELOAD_TIMER	250
#define RTC_SOFT_COUNT		125
#if F_CPU != (RTC_PRESCALER*RTC_RELOAD_TIMER*RTC_SOFT_COUNT)
	#error RTC_NOT_ADJUSTED
#endif

#ifdef MIDI_MONITOR
	#define MAX_TICK	5
	#define MAX_DECAY	7
#endif

#ifdef BACKLIGHT
	#define PORT_BL	PORTB
	#define PIN_BL	0x80
#endif

// Buttons
#define KEY_PORT	PIND
#define KEY_PORT2	PINB

#ifdef __AVR_ATmega168__
  #ifndef LED_DISP
	#define KEY_MASK	0xf8
	#define KEY_PLAY	0x80
  #else
	#define KEY_MASK	0x7c
	#define KEY_PLAY	0x04
  #endif
#else
	#define KEY_MASK	0x7c
	#define KEY_PLAY	0x04
#endif

#ifdef KEY_REPEAT
	#define KEY_REP_TIME_INIT	80
	#define KEY_REP_TIME		20
	#define FAST_SCROLL			30
	#ifdef RC5
		#define KEY_REP_TIME_INIT_IR	5
		#define KEY_REP_TIME_IR			1
	#endif
#endif

#define KEY_MASK2	0x03

#define KEY_STOP	0x08
#define KEY_REC		0x10
#define KEY_NEXT	0x20
#define KEY_LAST	0x40
#define KEY_LEFT	0x02
#define KEY_RIGHT	0x01

#define RC5_CODE_PLAY	0x35
#define RC5_CODE_STOP	0x36
#define RC5_CODE_REC	0x37
#define RC5_CODE_LEFT	0x32
#define RC5_CODE_RIGHT	0x34
#define RC5_CODE_NEXT	0x20
#define RC5_CODE_LAST	0x21
#define RC5_CODE_DELETE	0x31
#define RC5_CODE_MENU	0x1d
#define RC5_CODE_PLAYNEXT	0x29
#define RC5_CODE_PLAYLAST	0x22
#ifdef MIDI_MONITOR
	#define RC5_CODE_MON	0x3E
#endif

#if defined(CALIBRATE) || defined(MANUAL_CALIBRATE)
	#define DEFAULT_OSCCAL_POS	32
#endif


// States
#define STOP	0
#define PLAY	1
#define REC		2
#define ABORT_REC	3
#define DELETE	4
#define MENU	5
#define ERR		0x80

// EEPROM
#define EE_VERSION_POS	0
#define EE_DRUMCH_POS	1
#define EE_LYRICS_POS	2
#define EE_SSECT_POS	3
#define EE_MUTE_POS		5
#define EE_REPEAT_POS	7
#define EE_BLIGHT_POS	8
#define EE_MONITOR_POS	9
#define EE_OSCCAL_POS	10

// Menu
enum menu_entries_e {
	M_DRUMCH,
	M_LYR,
#ifdef OLD_CARDSCAN
	M_SSECT,
#endif
	M_MUTE,
	M_REP,
#ifdef BACKLIGHT
	M_LIGHT,
#endif
#ifdef MIDI_MONITOR
	M_MIDIMON,
#endif
#ifdef MANUAL_CALIBRATE
	M_MANCAL,
#endif
#ifdef CALIBRATE
	M_CAL,
#endif
#ifdef USE_RTC
	M_SETCLOCK,
	M_SETDATE,
#endif
	MAX_MENU
};

// Flags
#define KEY_FLAG	0x01
#define DT_FLAG		0x02
#define OCCHNG_FLAG	0x04
#ifdef RC5
	#define IR_FLAG		0x08
	#define IR_TGL		0x10
#endif
#ifdef OLD_CARDSCAN
	#define FAT_FLAG	0x20
#endif
#ifdef DIREKTWAHL
	#define DIRNUM_FLAG	0x40
#endif
#define MMCOK_FLAG	0x80

#define TIME_DISP	0x01
#ifdef KEY_REPEAT
	#define TIME_KPT	0x02
#endif

// MMC
#ifdef OLD_CARDSCAN
	#define START_SECTOR	528	// This is the sector number where the search for midi files begins (no FAT32 readings!)
#endif

// UART
#define UART_BAUD_RATE	31250
#define RX_BUF_SIZE		256	// lasts for at least 80ms on full MIDI data load (for *really* slow MMC write sector delays and FAT allocation searches)

// Variables
u08 sharedmem[512];
u08 rx_buf[RX_BUF_SIZE], rx_rdp=0, rx_wrp=0;	// RX buffer
u08 flags=KEY_FLAG, dspd=0, state=STOP;	// Flags
u16 tempo, new_ocr1a;	// Tempo memory
u32 trk_len;	// Track length
u32 time=0, delta_time=0;	// Deltatimes
//u16 rtime;
u08 time_min, time_sec, real_time=0, limit_count=RTC_SOFT_COUNT; // RTC
s08 speed, transpose;
u08 file_num=0, file_cnt=0;	// File
u16 *file_pos;
#ifdef OLD_CARDSCAN
	u32 last;
#endif
// Menu values
u08 ee_drumch=9, ee_lyr=1, ee_rep=2, ee_midimon=1, menu_cnt, s_ch=0, gl_lyr=false;
#ifdef BACKLIGHT
	u08 ee_light=1;
#endif
#ifdef OLD_CARDSCAN
	u16 ee_ssect=START_SECTOR;
#endif
u16 ee_mute=0;
u08 timeflag=0; // Multi-purpose Timer 2
#ifdef MIDI_MONITOR
	u08 midisig[16], notecnt[16], tickcnt, decaycnt;
#endif
#ifdef USE_RTC
    struct time_st {
        u08 gl_sek, gl_min, gl_std, gl_tag, gl_mon, gl_jahr;
    };
    extern struct time_st gl_time;
	#define gl_sek gl_time.gl_sek
	#define gl_min gl_time.gl_min
	#define gl_std gl_time.gl_std
	#define gl_tag gl_time.gl_tag
	#define gl_mon gl_time.gl_mon
	#define gl_jahr gl_time.gl_jahr
    extern const u08 TAGE_IM_MONAT[];
	#ifdef RTC_NUMERIC
    	u08 clockedit;
	#endif
#endif
#ifdef ADC_TEMPO
	u16 ad_value=1024>>1;	// ADC
#endif
#ifdef BUFFERED_READ
	#define PLAYBUF_SIZE RX_BUF_SIZE
	#define PBS_EMPTY 1
	#define PBS_FULL 2
	#define playbuf rx_buf
	#define playbuf_rdp rx_rdp
	#define playbuf_wrp rx_wrp
	u08 playbuf_state=PBS_EMPTY;
#endif

extern struct fat_filedata_st fat_filedata;
extern u32 sect;
extern u08 filemode;


// Local Functions
static void hw_init(void);
static void menu_save(void);
static void sendbyte(u08 byte);
static u08 fetchbyte(void);
static void checkbyte(u08 b);
static u08 checkstring(u08 *buf, u08 len);
static u32 fetch_varlen(void);
static void key_detect(void);
static void debounce(u08 mode);
static void wait_for_deltatime(void);
static void clear_time(void);
static void start_time(void);
static void stop_time(void);
#ifndef LED_DISP
	static void send_all_ch(u08 data);
	static void send_all_off(void);
	static u08 getdata(void);
	static void writebyte(u08 b);
	static void writestring(u08 *buf, u08 len);
	static void write_varlen(u32 value);
	static void write_time(void);
	//static void print(u08 y, u08 x, char *s);
	static void print_time(void);
	static void print_pause(void);
	static void print_main(void);
	static void print_menu(void);
	static void print_transpose(void);
	static void print_playinfo(void);
	static void print_tempo(void);
	static void draw_mute(void);
#else
	#define getdata() 0
	#define writebyte(a)
	#define writestring(a,b)
	#define write_varlen(a)
	#define write_time()
	//#define print(a,b,c) 
	#define print_time()
	#define print_pause()
	#define print_main()
	#define print_menu()
	#define print_transpose()
	#define print_tempo()
	#define print_playinfo() 
	#define draw_mute()
#endif


//---------------------------------------------------

#ifdef LED_DISP
/** Lässt die LED endlos blinken.
*/
void error(void) {
	while (1) {
		PORTC |= 1;
		delay_ms(120);
		PORTC &= ~1;
		delay_ms(120);
	}
}
#endif



/** Initialisiert die MCU.
	Ports, Timer, Interrupts, UART
	Highlevel Funktionen:
		LCD
		MMC/SD
		IR
	Liest EEPROM-Inhalt (oder setzt die Defaultwerte)
*/
static void hw_init(void) {
	u08 temp;

	// Oscillator
#ifdef __AVR_ATmega8__
	OSCCAL = 0xA2;
#endif

	// Ports
	DDRC = 0x3F;	// LCD
	PORTC = 0x00;
	lcd_init();
	lcd_string(DISP_MAIN, LINE_MAIN);
	lcd_string(DISP_MIDICTR, LINE_BOOT);
	delay_ms(1024);

#ifdef ADC_TEMPO
	DDRB = 0xC1;	// MMC & LCD enable
	PORTB = 0xFE;
#else
	DDRB = 0xC0;	// MMC
	PORTB = 0xFF;
#endif

	DDRD = 0x02;	// Input-Port, außer TxD
	PORTD = 0xFF;	// Pull-Ups aktivieren

#ifdef ADC_TEMPO
	// ADC
	ADMUX = 0x40|5;
	ADCSRA = 0xe7;	// free running, start, prescl 128
#endif

	// UART
	// Hilfsmakro zur UBRR-Berechnung ("Formel" laut Datenblatt)
	#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)
	UBRRH = (uint8_t)(UART_UBRR_CALC(UART_BAUD_RATE,F_CPU)>>8);
	UBRRL = (uint8_t)UART_UBRR_CALC(UART_BAUD_RATE,F_CPU);
#ifndef LED_DISP
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);//|(1<<UDRIE);
#else
	UCSRB = (1<<TXEN);
#endif
	// Midi device silence
//	send_all_off();

	// Read parameters from EEPROM
	if (eeprom_read_byte(EE_VERSION_POS) != 0x27) {	// eeprom empty
		eeprom_write_byte(EE_VERSION_POS, 0x27);
		menu_save();
	}
#ifndef LED_DISP
	else {	// load parameters
		ee_drumch = eeprom_read_byte(EE_DRUMCH_POS);
		if (ee_drumch > 15)	// invalid channel
			ee_drumch = 10;
		ee_lyr = eeprom_read_byte(EE_LYRICS_POS);
#ifdef OLD_CARDSCAN
		ee_ssect = eeprom_read_word(EE_SSECT_POS);
#endif
		ee_mute = eeprom_read_word(EE_MUTE_POS);
		ee_rep = eeprom_read_byte(EE_REPEAT_POS);
		if (ee_rep > 
#ifdef RANDOMSONG
		4
#else
		3
#endif
		) ee_rep = 2;
#ifdef BACKLIGHT
		ee_light = eeprom_read_byte(EE_BLIGHT_POS);
#endif
#ifdef MIDI_MONITOR
		ee_midimon = eeprom_read_byte(EE_MONITOR_POS);
		if (ee_midimon > 3)
			ee_midimon = 1;
#endif
#if defined(CALIBRATE) || defined(MANUAL_CALIBRATE)
		temp = eeprom_read_byte(EE_OSCCAL_POS);
		if (temp >= 128 && temp <= 200)
			OSCCAL = temp;
#endif
	}
#ifdef BACKLIGHT
	if (ee_light)
		PORT_BL &= ~PIN_BL;
#endif
#endif

	//Initialisierung der MMC/SD-Karte
	MMC_hw_init();
	spi_init();
#ifdef __AVR_ATmega168__
#ifndef LED_DISP
#ifdef RC5
	rc5_init(RC5_ALL);
#endif
#endif
#endif
	sei();
	delay_ms(100);
#ifdef RANDOMSONG //__AVR_ATmega168__
	TCCR1B = 1;
#endif
	if (MMC_init()) {
		lcd_string(DISP_OK, LINE_OK);
		flags |= MMCOK_FLAG;
	}
	else {
		lcd_string(DISP_CARDERR, LINE_CARDERR);
#ifdef LED_DISP
		error();
#endif
	}
#ifdef RANDOMSONG //__AVR_ATmega168__
	TCCR1B = 0;
	srand(TCNT1);
	TIFR1 |= 7;
	TCNT1 = 0;
#endif

	// Timer 0: Button Debounce
	// Timer 1: Delta Time Timer
	tempo = 2058;
	OCR1A = 2058;
	// Timer 2: RTC
	OCR2 = RTC_RELOAD_TIMER;

	// Interrupts
#ifdef __AVR_ATmega8__
	// mega8
	TIMSK = 0x51|0x80;	// Tmr 1&2 Compare Match Int. on, Overflow Timer 0 on
#endif
	// mega168
#ifdef __AVR_ATmega168__
//	TIMSK0 = 1;
	TIMSK1 = 2;
	TIMSK2 = 2;
#endif

#ifdef LED_DISP
	PORTC |= 1;
#endif

}


/** Save EEPROM data.
*/
static void menu_save(void) {
#ifndef LED_DISP
	eeprom_write_byte(EE_DRUMCH_POS, ee_drumch);
	eeprom_write_byte(EE_LYRICS_POS, ee_lyr);
#ifdef OLD_CARDSCAN
	eeprom_write_word(EE_SSECT_POS, ee_ssect);
#endif
	eeprom_write_word(EE_MUTE_POS, ee_mute);
	eeprom_write_byte(EE_REPEAT_POS, ee_rep);
#ifdef BACKLIGHT
	eeprom_write_byte(EE_BLIGHT_POS, ee_light);
#endif
#ifdef MIDI_MONITOR
	eeprom_write_byte(EE_MONITOR_POS, ee_midimon);
#endif
#if defined(CALIBRATE) || defined(MANUAL_CALIBRATE)
	eeprom_write_byte(EE_OSCCAL_POS, OSCCAL);
#endif
#endif
}


//----------------------------------------------------------
// UART

/** Send UART byte.
	@param Byte to send
*/
static void sendbyte(u08 byte) {
	loop_until_bit_is_set(UCSRA, UDRE);//while (!(UCSRA & (1<<UDRE)));
	UDR = byte;
}


#ifndef LED_DISP

/** Sends to all channels.
	@param Midi byte to send to all channels
*/
static void send_all_ch(u08 data) {
	u08 i;
	for (i=0; i<16; i++) {
		sendbyte(0xB0+i);
		sendbyte(data);
		sendbyte(0x00);
	}
}


/** All midi off.
	Sends midi all note off command.
*/
static void send_all_off(void) {
	send_all_ch(0x7B); // All Notes Off
	//send_all_ch(0x79); // All Controllers Off
	send_all_ch(0x78); // All Sounds Off
}


/** Wait for UART RX.
	@return Returns received byte
*/
static u08 getdata(void) {
	while (rx_rdp == rx_wrp && state == REC) {
		key_detect();
		if (timeflag&TIME_DISP)
			print_time();
#ifdef MIDI_MONITOR
		if (ee_midimon&1) {
			if (tickcnt >= MAX_TICK) {
				u08 i;
				tickcnt = 0;
				decaycnt++;
				if (decaycnt >= MAX_DECAY) {
					decaycnt = 0;
					for (i=0; i<16; i++)
						if (midisig[i])
							midisig[i]--;
				}
				lcd_setcur(0, 1);
				for (i=0; i<16; i++)
					lcd_data(midisig[i]);
			}
		}
#endif
	}
	u08 b = rx_buf[rx_rdp++];
	rx_rdp &= RX_BUF_SIZE-1;
	return b;
}
#endif


//----------------------------------------------------------
// MMC IO

// Read from MMC
#ifdef BUFFERED_READ

/** Abspielpuffer füllen.
	Liest MMC-Daten und füllt damit den Puffer ganz voll.
*/
static void fill_playbuf(void) {
	if (playbuf_state == PBS_FULL)
		return;
	while ((playbuf_rdp != playbuf_wrp || playbuf_state == PBS_EMPTY)) {
		playbuf[playbuf_wrp++] = mmc_fetch_byte();
		playbuf_wrp &= PLAYBUF_SIZE-1;
		playbuf_state = 0;
	}
	playbuf_state = PBS_FULL;
}

/** Abspielpuffer initialisieren.
*/
static void init_playbuf(void) {
	playbuf_wrp = 0;
	playbuf_rdp = 0;
	playbuf_state = PBS_EMPTY;
}

/** Ein Byte vom Puffer holen.
	Wenn der Puffer leer ist, wird er neu gefüllt.
*/
static u08 read_playbuf(void) {
	u08 temp;

	if (playbuf_state == PBS_EMPTY)
		fill_playbuf();
	temp = playbuf[playbuf_rdp++];
	playbuf_rdp &= PLAYBUF_SIZE-1;
	if (playbuf_rdp == playbuf_wrp)
		playbuf_state = PBS_EMPTY;
	else
		playbuf_state = 0;
	return temp;
}

/** Ein Byte von MMC lesen.
	@return Gelesenes Byte
*/
static u08 fetchbyte(void) {
	if (trk_len)
		trk_len--;
	return read_playbuf();
}

#else

/** Ein Byte von MMC lesen.
	@return Gelesenes Byte
*/
static u08 fetchbyte(void) {
	trk_len--;
	return mmc_fetch_byte();
}
#endif


#ifndef LED_DISP

/** Write byte to MMC.
	@param Byte to write
*/
static void writebyte(u08 b) {
	trk_len++;
	mmc_write_byte(b);
}

/** Write Word to MMC.
	Big Endian.
	@param Word to write
*/
static void mmc_write_word(u16 b) {
	writebyte((u08)(b>>8));
	writebyte((u08)b);
}

/** Write DWord to MMC.
	Big Endian.
	@param DWord to write
*/
static void mmc_write_dword(u32 b) {
	writebyte((u08)(b>>24));
	writebyte((u08)(b>>16));
	writebyte((u08)(b>>8));
	writebyte((u08)b);
}

/** Write string to MMC.
	@param Pointer to string
	@param Length of string
*/
static void writestring(u08 *buf, u08 len) {
	u08 cnt=0;
	for (cnt=0; cnt<len; cnt++)
		mmc_write_byte(buf[cnt]);
}
#endif

/** Read byte from MMC and verify value.
	If not identical, prints error message and stops playing.
	@param Byte to check MMC for
*/
static void checkbyte(u08 b) {
	u08 temp = fetchbyte();
	if (b != temp) {
		state = ERR+DISP_FILEERR;
	}
}


/** Read string from MMC and verify content.
	@param Pointer to string
	@param Length of string
	@return true, if identical
*/
static u08 checkstring(u08 *buf, u08 len) {
	u08 b, cnt=0;

	for (cnt=0; cnt<len; cnt++) {
		b = fetchbyte();
		if (b != (u08)buf[cnt])
			return 0;
	}
	return 1;
}


//----------------------------------------------------------
// MIDI format

#ifndef LED_DISP

/** Write Midi variable quantity to SD.
	@param Binary U32 number
*/
static void write_varlen(u32 value) {
	u32 buffer = value & 0x7f;
	while (value >>= 7) {
		buffer <<= 8;
		buffer |= 0x80;
		buffer += (value & 0x7f);
	}
	while (1) {
		writebyte((u08)buffer);
		if (buffer & 0x80)
			buffer >>= 8;
		else
			break;
	}
}


/** Write delta time to SD.
	Resets delta time to 0.
*/
static void write_time(void) {
	write_varlen(time);
	time = 0;
}
#endif

/** Read Midi variable quantity from SD.
	@return Binary number
*/
static u32 fetch_varlen(void) {
	u32 v;
	u08 c;

	c = fetchbyte();
	v = c;
	if ((v & 0x80) > 0) {
		v &= 0x7F;
		do {
			c = fetchbyte();
			v = (v << 7) + (c & 0x7F);
		} while (c & 0x80);
	}
  	return v;
}


/** Wait for delta time to expire.
	While waiting, check keys/IR and re-fill play buffer.
*/
static void wait_for_deltatime(void) {
	u32 dt;

	dt = fetch_varlen();
	if (time <= dt)
		delta_time = dt - time;
	else {
		delta_time = 0;
		time -= dt;
	}
	if (delta_time) {
		flags |= DT_FLAG;
		while ((flags & DT_FLAG) && state == PLAY) {
			key_detect();
			if (!gl_lyr) {
				if (timeflag&TIME_DISP) {
					print_time();
				}
#ifdef MIDI_MONITOR
				if (ee_midimon&2) {
					if (tickcnt >= MAX_TICK) {
						u08 i;
						tickcnt = 0;
						decaycnt++;
						if (decaycnt >= MAX_DECAY) {
							decaycnt = 0;
							for (i=0; i<16; i++)
								if (midisig[i])
									midisig[i]--;
						}
						lcd_setcur(0, 1);
						for (i=0; i<16; i++)
							lcd_data(midisig[i]);
					}
				}
#endif
			}
#ifdef BUFFERED_READ
			fill_playbuf();
#endif
		}
	}
}


/** Tempo bestimmen.
	Berechnet Zeitbasis zum Abspielen der Delta Times.
	Passt auch die Echtzeitanzeige-Geschwindigkeit an.
*/
static void calc_tempo(void) {
#ifdef ADC_TEMPO
	new_ocr1a = tempo * ad_value / 1024 + (tempo >> 1);
	flags |= OCCHNG_FLAG;
	TCNT2 = 0;	// TODO: Sicheres CTC benutzen!
	OCR2 = RTC_RELOAD_TIMER * ad_value / 1024 + RTC_RELOAD_TIMER / 2;
	lcd_setcur(TEMPO_X, TEMPO_Y);
	if (dspd) {
		lcd_number(ad_value / 5 + 98, 1);
		OCR2 = OCR2>>1;
	}
	else {
		lcd_number(ad_value / 10 + 49, 1);
	}
	lcd_data('%');
#else
	if (dspd) {
		TCNT1 = 0;
		dspd = 0;
	}
	new_ocr1a = tempo - (tempo>>4)*speed;
	flags |= OCCHNG_FLAG;
#ifndef LED_DISP
	if (!gl_lyr && !(ee_midimon&2)) {
		print_tempo();
	}
#endif
	limit_count = RTC_SOFT_COUNT - (RTC_SOFT_COUNT*speed)/(u16)16;
	if (real_time >= limit_count)
		real_time = limit_count-1;
#endif
}


#ifdef CALIBRATE
/** Calibrate the RC internal OSC using MIDI IN data.
*/
static void calibrate_osc(void) {
	u08 old_osccal = OSCCAL;
	u08 delta, delta_min=255, best_osccal=old_osccal, bitcnt;
	u08 maxscan = 1, i, dsum;
	u16 temp;
	union {
		u08 byte[2];
		u16 word;
	} word_u;

	UCSR0B = 0;
	TCCR1B = 0;
	lcd_string(DISP_CAL1, 0);
	lcd_string(DISP_CAL2, 1);

	for (OSCCAL = 131; OSCCAL <= 181; OSCCAL++) {
		i = 0; dsum = 0;
		do {
			delay_ms(2);
			while (!(PIND & 1));
			TCNT1 = 0;
			PCIFR |= 0x04;
			PCMSK2 = 0x01;
			PCICR = 0x04;
			while (PCICR);
			word_u.word = TCNT1;
			if (word_u.byte[0] >= 0x80 && word_u.byte[1])
				bitcnt = word_u.byte[1]-1;
			else
				bitcnt = word_u.byte[1];
			temp = TCNT1/bitcnt;
			if (temp > 256)
				delta = temp-256;
			else
				delta = 256-temp;
			dsum += delta; i++;
		}
		while (i < maxscan);
		if (maxscan == 8)
			delta = (dsum+4)>>3; // Mittelwert
		if (delta < 10)
			maxscan = 8;
		else
			maxscan = 1;
		if (OSCCAL == old_osccal && delta < 6) {
			PCMSK2 = 0;
			UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
			lcd_string(DISP_CAL3, 1); // "Not neccessary"
			delay_ms(1536);
			return;
		}
		else {
			lcd_setcur(20, 0);
			lcd_number((OSCCAL-131)<<1, 0);
			lcd_data('%');
		}
		if (delta < delta_min) {
			delta_min = delta;
			best_osccal = OSCCAL;
		}
	}
	OSCCAL = best_osccal; //+10?
	PCMSK2 = 0;
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	if (delta_min > 6) {
		lcd_string(DISP_CAL4, 1); // "Manual adjust neccessary"
	}
	else {
		lcd_string(DISP_BLANK, 1);
	}
	delay_ms(1536);
}
#endif


//----------------------------------------------------------
// Buttons

/** Entprellen der Tasten.
	ATmega168 benutzt dafür den WDT.
	@param WDT-Modus [1=Timermodus, 0=WDT]
*/
static void debounce(u08 mode) {
	flags &= ~KEY_FLAG;
#ifdef __AVR_ATmega168__
	cli();
	WDTCSR = 0x18;	// Setup WDT-Init
	WDTCSR = 0x40|mode;	// WDIE=1->Interrupt mode; 32ms Timeout
	wdt_reset();
	sei();
#else
	TCNT0 = 0;
	TCCR0 = 5;	// clk/1024
#endif
}


#ifndef LED_DISP

#ifdef DIREKTWAHL
/** Direktwahl des Songs per Zifferntasten.
	@param Taste
*/
inline static void direkt_wahl(u08 k) {
	static u08 zif1=0xff;
	if (file_cnt < 11) {
		if (!k)
			k = 10;
		file_num = k-1;
		state = PLAY;
	}
	else {
		if (zif1 == 0xff) {
			zif1 = k;
			lcd_setcur(DWAHL_X, DWAHL_Y);
			lcd_data('-');
			lcd_data('>');
			lcd_data(k+'0');
		}
		else {
			file_num = k + zif1*10;
			if (file_num <= file_cnt && file_num) {
				file_num--;
				state = PLAY;
			}
			else {
				lcd_setcur(DWAHL_X, DWAHL_Y);
				lcd_data(' ');
				lcd_data(' ');
				lcd_data(' ');
			}
			zif1=0xff;
		}
	}
	if (state == PLAY) {
#ifdef OLD_CARDSCAN
		if (flags & FAT_FLAG)
#endif
		{
			fat_read_filedata(file_num);
			print_filename(NAME_LINE);
			print_filesize();
		}
		start_time();	// Start Time
		flags |= DIRNUM_FLAG; // Verhindert Zufallswiedergabe bei Direktwahl
	}
}
#endif


/** Check buttons.
	Tastenabfrage/IR-Auswertung.
*/
static void key_detect(void) {
	static u08 ok=0, wur=0;
#ifdef KEY_REPEAT
	static u08 kpt=0, krc=0;//!
#endif
	u08 lwur=0;
	u08 k = ((u08)(~KEY_PORT)&KEY_MASK) | ((u08)(~KEY_PORT2)&KEY_MASK2);
	if ((k&KEY_NEXT) && (k&KEY_LAST)) {
		k = RC5_CODE_MENU; lwur=1;
	}
	else if ((k&KEY_LEFT) && (k&KEY_RIGHT)) {
		k = RC5_CODE_DELETE; lwur=1;
	}
#ifdef MIDI_MONITOR
	else if ((k&KEY_LEFT) && (k&KEY_NEXT)) {
		k = RC5_CODE_MON; lwur=1;
	}
#endif
//	else if ((k&KEY_RIGHT) && (k&KEY_LAST)) {
//		k = RC5_CODE_?; lwur=1; }
//	else if ((k&KEY_RIGHT) && (k&KEY_NEXT)) {
//		k = RC5_CODE_?; lwur=1; }
//	else if ((k&KEY_LEFT) && (k&KEY_LAST)) {
//		k = RC5_CODE_?; lwur=1; }
	else if ((k&KEY_STOP) && (k&KEY_NEXT)) {
		k = RC5_CODE_PLAYNEXT; lwur=1;
	}
	else if ((k&KEY_STOP) && (k&KEY_LAST)) {
		k = RC5_CODE_PLAYLAST; lwur=1;
	}
	else if (k&KEY_STOP)
		k = RC5_CODE_STOP;
	else if (k&KEY_REC)
		k = RC5_CODE_REC;
	else if (k&KEY_LEFT)
		k = RC5_CODE_LEFT;
	else if (k&KEY_RIGHT)
		k = RC5_CODE_RIGHT;
	else if (k&KEY_NEXT)
		k = RC5_CODE_NEXT;
	else if (k&KEY_LAST)
		k = RC5_CODE_LAST;
	else if (k&KEY_PLAY)
		k = RC5_CODE_PLAY;

	if (!(flags & KEY_FLAG))
		return;
#ifdef ADC_TEMPO
	if (state == PLAY && TCCR1B && ad_value != ADC) {
		debounce();
		ad_value = ADC;
		calc_tempo();
	}
#endif
#ifdef RC5
	if (rc5.flip != -1) {
		k = rc5.code|0x80;
		if (k != ok || rc5.flip != ((flags>>4)&1) || (flags&IR_FLAG) == 0) {
			ok = k;
			k = 0;
		}
		flags |= IR_FLAG;
		if (rc5.flip)
			flags |= IR_TGL;
		else
			flags &= ~IR_TGL;
		rc5.flip = -1;
		debounce(3); // Längeres Entprellen, weil >60ms Sendeabstand der FB
	}
#endif
#ifdef KEY_REPEAT
	if ((k != ok || (krc<2 && (kpt >= KEY_REP_TIME_INIT 
#ifdef RC5
	|| ((flags&IR_FLAG) && kpt >= KEY_REP_TIME_INIT_IR)
#endif
	)) || (krc>1 && (kpt >= KEY_REP_TIME 
#ifdef RC5
	|| ((flags&IR_FLAG) && kpt >= KEY_REP_TIME_IR)
#endif
	))) && (!wur || (wur && !k)))//!
#else
	if (k != ok && (!wur || (wur && !k)))
#endif
	{
#ifdef KEY_REPEAT
		kpt = 0;
#endif
		if (k != ok
#ifdef RC5
		&& !(flags&IR_FLAG)
#endif
		) {
			debounce(1);
			ok = k;
		}
		if (ok) {
#ifdef KEY_REPEAT
			TCCR2 = 6+8;
			if (krc < 255)
				krc++;
#endif
#ifdef RC5_DISP // Debug - Anzeige RC5-Code der FB
	lcd_setcur(24,0);
	lcd_hex_u08(ok);
#endif
			k = ok & 0x7f;

			if (state == MENU) {
				if (k == RC5_CODE_NEXT) { // Menü blättern auf
					menu_cnt--;
					if (menu_cnt >= MAX_MENU)
						menu_cnt = MAX_MENU-1;
					print_menu();
				}
				else if (k == RC5_CODE_LAST) { // Menü blättern ab
					menu_cnt++;
					if (menu_cnt == MAX_MENU)
						menu_cnt = 0;
					print_menu();
				}
				else if (k == RC5_CODE_LEFT) { // Wert verkleinern
					switch (menu_cnt) {
						case M_DRUMCH:
							if (ee_drumch)
								ee_drumch--;
							break;
						case M_LYR:
							if (ee_lyr)
								ee_lyr = 0;
							break;
#ifdef OLD_CARDSCAN
						case M_SSECT:
							if (ee_ssect)
								ee_ssect--;
							break;
#endif
						case M_MUTE:
							if (ee_mute & (1<<s_ch))
								ee_mute &= ~(1<<s_ch);
							else
								ee_mute |= 1<<s_ch;
							break;
						case M_REP:
							if (ee_rep)
								ee_rep--;
							break;
#ifdef BACKLIGHT
						case M_LIGHT:
							if (ee_light)
								ee_light = 0;
							PORT_BL |= PIN_BL;
							break;
#endif
#ifdef MANUAL_CALIBRATE
						case M_MANCAL:
							if (OSCCAL > 128)
								OSCCAL--;
							break;
#endif
#ifdef MIDI_MONITOR
						case M_MIDIMON:
							if (ee_midimon)
								ee_midimon--;
							break;
#endif
#ifdef USE_RTC
						case M_SETCLOCK:
							gl_sek = 0;
							if (gl_min && krc < FAST_SCROLL)
								gl_min--;
							else {
								if (krc < FAST_SCROLL)
									gl_min = 59;
								if (gl_std)
									gl_std--;
								else
									gl_std = 23;
							}
							set_time();
							break;
						case M_SETDATE:
							if (gl_tag > 1 && krc < FAST_SCROLL)
								gl_tag--;
							else {
								if (krc < FAST_SCROLL)
									gl_tag = TAGE_IM_MONAT[gl_mon-1];
								if (gl_mon > 1)
									gl_mon--;
								else {
									gl_mon = 12;
									gl_jahr--;
								}
							}
							set_date();
							break;
#endif
					}
					print_menu();
				}
				else if (k == RC5_CODE_RIGHT) { // Wert vergrößern
					switch (menu_cnt) {
						case M_DRUMCH:
							if (ee_drumch < 0x0f)
								ee_drumch++;
							break;
						case M_LYR:
							if (!ee_lyr)
								ee_lyr = 1;
							break;
#ifdef OLD_CARDSCAN
						case M_SSECT:
							ee_ssect++;
							break;
#endif
						case M_MUTE:
							s_ch++;
							s_ch &= 0x0f;
							break;
						case M_REP:
							if (ee_rep < 
#ifdef RANDOMSONG
								4
#else
								3
#endif
								) ee_rep++;
							break;
#ifdef BACKLIGHT
						case M_LIGHT:
							if (!ee_light)
								ee_light = 1;
							PORT_BL &= ~PIN_BL;
							break;
#endif
#ifdef CALIBRATE
						case M_CAL:
							calibrate_osc();
							break;
#endif
#ifdef MANUAL_CALIBRATE
						case M_MANCAL:
							if (OSCCAL < 200)
								OSCCAL++;
							break;
#endif
#ifdef MIDI_MONITOR
						case M_MIDIMON:
							if (ee_midimon < 3)
								ee_midimon++;
							break;
#endif
#ifdef USE_RTC
						case M_SETCLOCK:
							gl_sek = 0;
							if (gl_min < 60 && krc < FAST_SCROLL)
								gl_min++;
							else {
								if (krc < FAST_SCROLL)
									gl_min = 0;
								if (gl_std < 24)
									gl_std++;
								else
									gl_std = 0;
							}
							set_time();
							break;
						case M_SETDATE:
							if (gl_tag < TAGE_IM_MONAT[gl_mon-1] && krc < FAST_SCROLL)
								gl_tag++;
							else {
								if (krc < FAST_SCROLL)
									gl_tag = 1;
								if (gl_mon < 12)
									gl_mon++;
								else {
									gl_mon = 1;
									gl_jahr++;
								}
							}
							set_date();
							break;
#endif
					}
					print_menu();
				}
#ifdef USE_RTC
#ifdef RTC_NUMERIC
				else if (k <= 9) { // Ziffern
					u08 temp;
					temp = clockedit;
					if (menu_cnt == M_SETCLOCK) {
						if (temp == 0) {
							lcd_setcur(CLOCK_X+temp+1, LINE_OPTION);
							gl_std=0; gl_min=0; gl_sek=0;
							lcd_data('_');lcd_data(':');lcd_data('_');lcd_data('_');lcd_data(':');lcd_data('_');lcd_data('_');
						}
						lcd_setcur(CLOCK_X+temp, LINE_OPTION);
						lcd_data('0'+k);
						if (temp == 0 || temp == 3 || temp == 6)
							dspd = k*10;
						else
							dspd += k;
						temp++;
						if (temp == 2 || temp == 5)
							temp++;
						if (temp == 3)
							gl_std = dspd;
						else if (temp == 6)
							gl_min = dspd;
						else if (temp == 8) {
							gl_sek = dspd;
							set_time();
						}
						clockedit = temp;
					}
					else if (menu_cnt == M_SETDATE) {
						if (temp == 0) {
							lcd_setcur(DATE_X+temp+1, LINE_OPTION);
							gl_tag=1; gl_mon=1; gl_jahr=7;
							lcd_data('_');lcd_data('.');lcd_data('_');lcd_data('_');lcd_data('.');lcd_data('_');lcd_data('_');
						}
						lcd_setcur(DATE_X+temp, LINE_OPTION);
						lcd_data('0'+k);
						if (temp == 0 || temp == 3 || temp == 6)
							dspd = k*10;
						else
							dspd += k;
						temp++;
						if (temp == 2 || temp == 5)
							temp++;
						if (temp == 3)
							gl_tag = dspd;
						else if (temp == 6)
							gl_mon = dspd;
						else if (temp == 8) {
							gl_jahr = dspd;
							set_date();
						}
						clockedit = temp;
					}
				}
#endif
#endif
				else if (k == RC5_CODE_STOP || k == RC5_CODE_REC) { // Schließen und speichern
					print_main();
					menu_save();
				}
				else if (k == RC5_CODE_MENU) { // Schließen und verwerfen
					print_main();
				}
			}
			else if (file_cnt && state != REC) {

			// Play
				if (state == PLAY) {
					if (k == RC5_CODE_LAST) { // Transponieren abwärts
						if (transpose > -24) {
							transpose--;
							print_transpose();
							send_all_off();
						}
					}
					else if (k == RC5_CODE_NEXT) { // Transponieren aufwärts
						if (transpose < 24) {
							transpose++;
							print_transpose();
							send_all_off();
						}
					}
					else if (k == RC5_CODE_LEFT) { // Langsamer
						if (speed > -15) {
							speed--;
							calc_tempo();
						}
					}
					else if (k == RC5_CODE_RIGHT) { // Schneller
						dspd = 1;
						if (speed < 15) {
							speed++;
							calc_tempo();
						}
					}
					else if (k == RC5_CODE_PLAY) { // Pause
						if (TCCR1B) {
							send_all_off();
							stop_time();	// Pause
							print_pause();
						}
						else {
							start_time();	// Start Time
						}
					}
					else if (k == RC5_CODE_STOP) { // Stopp
						state = STOP;
					}
					else if (k == RC5_CODE_DELETE) { // Tempo normal
						if (speed != 0) {
							speed = 0;
							calc_tempo();
						}
					}
					else if (k == RC5_CODE_MENU) { // Transpose normal
						if (transpose != 0) {
							transpose = 0;
							print_transpose();
							send_all_off();
						}
					}
#ifdef MIDI_MONITOR
					else if (k == RC5_CODE_MON) {   // Monitor bei Play toggeln
						if (ee_midimon&2) {
							ee_midimon &= ~2;
							print_playinfo();
						}
						else {
							ee_midimon |= 2;
							tickcnt = 0;
							decaycnt = 0;
							lcd_string(DISP_BLANK, LINE_TEMPO);
						}
					}
#endif
				}

			// Stop
				else if (state == STOP) {
					if (k == RC5_CODE_PLAY) { // Wiedergabe
						state = PLAY;
					}
					else if (k == RC5_CODE_REC) { // Aufnahme
						state = REC;
					}
#ifdef DIREKTWAHL
					else if (k <= 9) { // Ziffernwahl
						direkt_wahl(k);
					}
#endif
					else if (k == RC5_CODE_DELETE) { // Datei löschen
						state = DELETE;
					}
					else if (k == RC5_CODE_MENU) { // Menü öffnen
						menu_cnt = 0;
						print_menu();
						state = MENU;
					}
					else if (k == RC5_CODE_LAST) { // Blättern rückwärts
						if (!file_num)
							file_num = file_cnt;
						file_num--;
						lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
						lcd_number(file_num+1, 1);
#ifdef OLD_CARDSCAN
						if (flags & FAT_FLAG)
#endif
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
#ifdef OLD_CARDSCAN
						else {
							lcd_setcur(FSIZE_X, FSIZE_Y);
							lcd_hex_u16(file_pos[file_num]);
						}
#endif
					}
					else if (k == RC5_CODE_NEXT) { // Blättern vorwärts
						file_num++;
						if (file_num == file_cnt)
							file_num = 0;
						lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
						lcd_number(file_num+1, 1);
#ifdef OLD_CARDSCAN
						if (flags & FAT_FLAG)
#endif
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
#ifdef OLD_CARDSCAN
						else {
							lcd_setcur(FSIZE_X, FSIZE_Y);
							lcd_hex_u16(file_pos[file_num]);
						}
#endif
					}
					else if (k == RC5_CODE_PLAYNEXT) { // Skip zum nächsten Song
						file_num++;
						if (file_num == file_cnt)
							file_num = 0;
#ifdef OLD_CARDSCAN
						if (flags & FAT_FLAG)
#endif
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
#ifdef OLD_CARDSCAN
						else {
							lcd_setcur(STOPSECT_X, STOPSECT_Y);
							lcd_hex_u16(file_pos[file_num]);
						}
#endif
						lcd_string(DISP_BLANK, 1);
						state = PLAY;
					}
					else if (k == RC5_CODE_PLAYLAST) { // Skip zum vorigen Song
						if (file_num == 0)
							file_num = file_cnt;
						file_num--;
#ifdef OLD_CARDSCAN
						if (flags & FAT_FLAG)
#endif
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
#ifdef OLD_CARDSCAN
						else {
							lcd_setcur(STOPSECT_X, STOPSECT_Y);
							lcd_hex_u16(file_pos[file_num]);
						}
#endif
						lcd_string(DISP_BLANK, 1);
						state = PLAY;
					}
				}
			}
			else {

			// No files on card or state is REC
				if (state == STOP) {
					if (k == RC5_CODE_MENU) { // Menü öffnen
						menu_cnt = 0;
						print_menu();
						state = MENU;
					}
					else if ((flags & MMCOK_FLAG) && k == RC5_CODE_REC) { // Aufnahme
						state = REC;
					}
				}

			// Rec
				else if (state == REC) {
					if (k == RC5_CODE_STOP) {
						state = STOP;
					}
					else if (k == RC5_CODE_REC) {
						state = ABORT_REC;
					}
#ifdef MIDI_MONITOR
					else if (k == RC5_CODE_MON) {   // Monitor bei Rec toggeln
						if (ee_midimon&1) {
							ee_midimon &= ~1;
							lcd_string(DISP_BLANK, LINE_TEMPO);
						}
						else {
							ee_midimon |= 1;
							tickcnt = 0;
							decaycnt = 0;
						}
					}
#endif
				}
			}
			wur = lwur;
		}
		else {
			wur=0;
#ifdef KEY_REPEAT
			krc = 0;
#endif
		}
	}
#ifdef KEY_REPEAT
	if (ok && kpt < 255 && (timeflag&TIME_KPT)) {
		timeflag &= ~TIME_KPT;
		kpt++;
	}
#endif
}
#else

// Check buttons
static void key_detect(void) {
	static u08 ok=0;
	u08 k = ((u08)(~KEY_PORT)&KEY_MASK) | ((u08)(~KEY_PORT2)&KEY_MASK2);
	if (k&KEY_STOP)
		k = RC5_CODE_STOP;
	else if (k&KEY_NEXT)
		k = RC5_CODE_NEXT;
	else if (k&KEY_LAST)
		k = RC5_CODE_LAST;
	else if (k&KEY_PLAY)
		k = RC5_CODE_PLAY;

	if (!(flags & KEY_FLAG))
		return;
	if (k != ok) {
		if (k != ok) {
			debounce(1);
			ok = k;
		}
		if (ok != 0) {
			k = ok & 0x7f;
			if ((k == RC5_CODE_LAST)) {	// One file back
				if (state == STOP && file_cnt) {
					if (file_num == 0)
						file_num = file_cnt;
					file_num--;
#ifdef OLD_CARDSCAN
					if (flags & FAT_FLAG)
#endif
					{
						fat_read_filedata(file_num);
					}
				}
				else if (state == PLAY) {	// Slower
					if (speed > -15) {
						speed--;
						calc_tempo();
					}
				}
			}
			else if ((k == RC5_CODE_NEXT)) {	// Next file
				if (state == STOP && file_cnt) {
					file_num++;
					if (file_num == file_cnt)
						file_num = 0;
#ifdef OLD_CARDSCAN
					if (flags & FAT_FLAG)
#endif
					{
						fat_read_filedata(file_num);
					}
				}
				if (state == PLAY) {	// Faster
					dspd = 1;
					if (speed < 15) {
						speed++;
						calc_tempo();
					}
				}
			}
			else if (k == RC5_CODE_STOP) {	// Stop
				state = STOP;
			}
			else if ((k == RC5_CODE_PLAY) && file_cnt) {	// Play/Pause
				if (state == PLAY) {
					if (TCCR1B) {
						send_all_off();
						stop_time();	// Pause
					}
					else {
						start_time();	// Start Time
					}
				}
				else if (state == STOP && file_cnt) {
					state = PLAY;
				}
			}
		}
	}
}

#endif


#ifndef LED_DISP

/** Anzeige Hauptbildschirm.
	Zeigt Nummer, Anzahl, Dateiname und Größe an.
*/
static void print_main(void) {
	if (file_cnt) {
		lcd_string(DISP_SELECT, LINE_SELECT);
		lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
		lcd_number(file_num+1, 1);
#ifdef OLD_CARDSCAN
		if (flags & FAT_FLAG)
#endif
		{
			fat_read_filedata(file_num);
			print_filename(NAME_LINE);
			print_filesize();
		}
#ifdef OLD_CARDSCAN
		else {
			lcd_setcur(FSIZE_X, FSIZE_Y);
			lcd_hex_u16(file_pos[file_num]);
		}
#endif
		lcd_setcur(FCNT_X, FCNT_Y);
		lcd_number(file_cnt, 1);
	}
	else {
		lcd_string(DISP_MAIN, LINE_MAIN);
		if (flags & MMCOK_FLAG)
			lcd_string(DISP_NOFILE, LINE_NOFILE);
		else
			lcd_string(DISP_CARDERR, LINE_CARDERR);
	}
	state = STOP;
}


/** Anzeige Menü.
*/
static void print_menu(void) {
	lcd_string(DISP_MENU, LINE_MENU);
	lcd_string(DISP_MDRUM+menu_cnt, LINE_OPTION);
#ifdef USE_RTC
#ifdef RTC_NUMERIC
	clockedit = 0;
#endif
#endif
	switch (menu_cnt) {
		case M_DRUMCH:
			lcd_setcur(DRUMCH_X, LINE_OPTION);
			lcd_number(ee_drumch+1, 0);
			break;
		case M_LYR:
			lcd_setcur(LYR_X, LINE_OPTION);
#ifdef COMPACT16x2
			lcd_data(ee_lyr+'0');
#else
			if (!ee_lyr) {
				lcd_data('O');lcd_data('f');lcd_data('f');
			}
			else {
				lcd_data('O');lcd_data('n');
			}
#endif
			break;
#ifdef OLD_CARDSCAN
		case M_SSECT:
			lcd_setcur(SSECT_X, LINE_OPTION);
			lcd_hex_u16(ee_ssect);
			break;
#endif
		case M_MUTE:
			draw_mute();
			break;
		case M_REP:
			lcd_setcur(REP_X, LINE_OPTION);
#ifdef COMPACT16x2
			lcd_data(ee_rep+'0');
#else
			switch (ee_rep) {
			case 0:
				lcd_data('O');lcd_data('f');lcd_data('f'); break;
			case 1:
				lcd_data('S');lcd_data('o');lcd_data('n');lcd_data('g'); break;
			case 2:
				lcd_data('D');lcd_data('i');lcd_data('r');lcd_data(' ');lcd_data('o');lcd_data('n');lcd_data('c');lcd_data('e'); break;
			case 3:
				lcd_data('A');lcd_data('l');lcd_data('l'); break;
#ifdef RANDOMSONG
			case 4:
				lcd_data('R');lcd_data('a');lcd_data('n');lcd_data('d');lcd_data('o');lcd_data('m'); break;
#endif
			}
#endif
			break;
#ifdef BACKLIGHT
		case M_LIGHT:
			lcd_setcur(LIGHT_X, LINE_OPTION);
#ifdef COMPACT16x2
			lcd_data(ee_light+'O');
#else
			if (!ee_light) {
				lcd_data('O');lcd_data('f');lcd_data('f');
			}
			else {
				lcd_data('O');lcd_data('n');
			}
#endif
			break;
#endif
#ifdef CALIBRATE
		case M_CAL:
			break;
#endif
#ifdef MANUAL_CALIBRATE
		case M_MANCAL:
			lcd_setcur(MANCAL_X, LINE_OPTION);
			lcd_number(OSCCAL, 1);
			break;
#endif
#ifdef MIDI_MONITOR
		case M_MIDIMON:
			lcd_setcur(MIDIMON_X, LINE_OPTION);
#ifdef COMPACT16x2
			lcd_data(ee_midimon+'0');
#else
			switch (ee_midimon) {
			case 0:
				lcd_data('O');lcd_data('f');lcd_data('f'); break;
			case 1:
				lcd_data('R');lcd_data('e');lcd_data('c'); break;
			case 2:
				lcd_data('P');lcd_data('l');lcd_data('a');lcd_data('y'); break;
			case 3:
				lcd_data('B');lcd_data('o');lcd_data('t');lcd_data('h'); break;
			}
#endif
			break;
#endif
#ifdef USE_RTC
		case M_SETCLOCK:
			read_time();
			lcd_setcur(CLOCK_X, LINE_OPTION);
			lcd_number(gl_std, 0);
			lcd_data(':');
			lcd_number(gl_min, 0);
			lcd_data(':');
			lcd_number(gl_sek, 0);
			break;
		case M_SETDATE:
			read_time();
			lcd_setcur(DATE_X, LINE_OPTION);
			lcd_number(gl_tag, 0);
			lcd_data('.');
			lcd_number(gl_mon, 0);
			lcd_data('.');
			lcd_number(gl_jahr, 0);
			break;
#endif
		default:
			break;
	}
}


/** Anzeige Transpose.
*/
static void print_transpose(void) {
	if (gl_lyr || (ee_midimon&2))
		return;
	lcd_setcur(TRANS_X, TRANS_Y);
	if (transpose < 0) {
		lcd_data('-');
		lcd_number(-transpose, 0);
	}
	else {
		lcd_data('+');
		lcd_number(transpose, 0);
	}
}


/** Anzeige Muteleiste.
*/
static void draw_mute(void) {
	u08 i;
	lcd_setcur(MUTE_X, MUTE_Y);
	for (i=0; i<16; i++) {
		if (ee_mute & (1<<i)) {
			if (i == s_ch)
				lcd_data('X');
			else
				lcd_data('x');
		}
		else {
			if (i == s_ch)
				lcd_data('O');
			else
				lcd_data('o');
		}
	}
}


/** Special print function
	@param row
	@param col
	@param pointer to string (0-terminated)
*/
/*
static void print(u08 y, u08 x, char *s) {
	lcd_setcur(x, y);
	while (*s)
		lcd_data(*s++);
}
*/

#endif


//----------------------------------------------------------
// Time

#ifndef LED_DISP


/** Zeit am Display anzeigen.
*/
static void print_time(void) {
	timeflag &= ~TIME_DISP;
	lcd_setcur(TIME_X, TIME_Y);
	lcd_number(time_min, 0);
	lcd_data(':');
	lcd_number(time_sec, 0);
}

/** Pause anstatt Zeit anzeigen.
*/
static void print_pause(void) {
	lcd_setcur(TIME_X, TIME_Y);
	lcd_data('P');
	lcd_data('A');
	lcd_data('U');
	lcd_data('S');
	lcd_data('E');
}

/** Geschwindigkeit am Display anzeigen.
*/
static void print_tempo(void)
{
	lcd_setcur(TEMPO_X, TEMPO_Y);
	if (speed < 0) {
		lcd_data('-');
		lcd_nibble(-speed);
	}
	else {
		lcd_data('+');
		lcd_nibble(speed);
	}
}

/** Transpose und Geschwindigkeit am Display anzeigen.
*/
static void print_playinfo(void)
{
	lcd_string(DISP_TEMPO, LINE_TEMPO);
	lcd_setcur(TRANS_X, TRANS_Y);
	if (transpose < 0)
		lcd_data('-');
	else
		lcd_data('+');
	lcd_number(transpose, 0);
	print_tempo();
}
#endif


/** Zeit zurücksetzen.
*/
static void clear_time(void) {
	TCCR2 = 0;
	time_sec = 0;
	time_min = 0;
	real_time = 0;
//	rtime = 0;
	limit_count = RTC_SOFT_COUNT;
}


/** Zeitanzeige starten.
*/
static void start_time(void) {
	print_time();
	TCNT1 = 0;
	TCCR1B = 8+2;
	TCNT2 = 0;
	TCCR2 = 6+8;
}


/** Zeitanzeige stoppen.
*/
static void stop_time(void) {
	TCCR1B = 0;
	TCCR2 = 0;
}


//----------------------------------------------------------
// Main
#ifndef LED_DISP
#ifdef RANDOMSONG

/** Zufallslied berechnen.
	Dabei wird das genannte Lied vermieden.
	@param Vermeiden
	@param Maximale Nummer
	@return Zufallszahl
*/
static u08 random_song(u08 avoid, u08 top) {
	u08 temp;
	if (top < 2)
		return 0;
	do
		temp = rand();
	while (temp == avoid || temp >= top);
	return temp;
}
#endif
#endif


/** Hauptprogramm.
*/
//void main(void) __attribute__ ((naked));
void main2(void) {
union {
	u08 byte[4];
	u16 word[2];
	u32 udword;
} temp;
	u08 ev, len, adp=0, ade=0, lyrx, lyry, trp=0, ch=0, ch_nact=0, i;
	u16 gl_timeset;

	file_pos = (u16 *)sharedmem;
	hw_init();

	if (flags & MMCOK_FLAG) {
/*		u16 s;
		for (s=0; s<0x208+0x69; s++) {
			u16 i;
			mmc_read_sector(s, sharedmem);
			for (i=0; i<512; i++)
				sendbyte(sharedmem[i]);
		}
		lcd_setcur(0,0);
		lcd_hex_u16(0);
		mmc_write_start(0);
		rx_wrp=0;
		rx_rdp=0;
		u32 i=0;
		while (1){
			while (rx_wrp != rx_rdp) {
//				lcd_data(rx_buf[rx_rdp++]);
				mmc_write_byte(rx_buf[rx_rdp++]);
			}
		}
		lcd_setcur(0,0);
		lcd_hex_u16(1);
		while (1);*/
		if (!fat_init()) {
#ifdef LED_DISP
			error();
#else
#ifdef OLD_CARDSCAN
			mmc_count_files(START_SECTOR);
#else
			lcd_string(DISP_CARDERR, LINE_CARDERR);
			while (1);
#endif
#endif
		}
		else {
#ifdef OLD_CARDSCAN
			flags |= FAT_FLAG;
#endif
			fat_count_files();
		}
		file_num = 0;
		print_main();
	}
#ifdef LED_DISP
	else
		error();
#endif

	while (1) {

		if (state == REC) {

#ifndef LED_DISP
				// Rekorder-Task
#ifdef OLD_CARDSCAN
				if (flags & FAT_FLAG) {
					fat_openfile(FILEMODE_WRITE, file_cnt);
					last = fat_filedata.startsect;
				}
				mmc_write_start(last+1);
#else
				fat_openfile(FILEMODE_WRITE, file_cnt);
				mmc_write_start(fat_filedata.startsect+1);
#endif
				OCR1A = 500000UL/240;	// Fixes Tempo festlegen
				trk_len = 490;
				time = 0;
				rx_wrp = 0;	// Puffer leeren
				rx_rdp = 0;
#ifdef MIDI_MONITOR
				for (i=0; i<16; i++) {
					midisig[i] = 0;
					notecnt[i] = 0;
				}
				tickcnt = 0;
				decaycnt = 0;
#endif


				lcd_string(DISP_RECRD, LINE_RECRD);	// Bildschirm anzeigen
				lcd_setcur(RECFNUM_X, RECFNUM_Y);
				lcd_number(file_cnt+1, 1);
				lcd_string(DISP_BLANK, LINE_REC_BLANK);

				clear_time();

//#define BACKUP_REC
#ifndef BACKUP_REC
				while (state == REC) {
					ev = getdata();	// Auf Midi-Event warten
					if (state == STOP || state == ABORT_REC)
						break;
					write_time();
					if (!TCCR1B) {
						start_time();//TCCR1B = 8+2;	// Start Time
						lcd_string(DISP_REC, LINE_REC);
						lcd_setcur(RECFNUM_X, RECFNUM_Y);
						lcd_number(file_cnt+1, 1);
					}
					writebyte(ev);

					if (ev == 0xf0) {	// SysEx
						do {
							i = getdata();
							writebyte(i);
						} while (i != 0xf7);
					}
					else if (ev < 0x80) {	// Running status
#ifdef MIDI_MONITOR
						if ((ade&0xf0) == 0x80) { // NoteOff
							writebyte(getdata());
							if (notecnt[ch])
								notecnt[ch]--;
							if (!notecnt[ch])
								midisig[ch] = 1;
						}
						else if ((ade&0xf0) == 0x90) { // NoteOn
							i = getdata();
							writebyte(i);
							i = (i>>4)+1;
							if (i > 7) i = 7;
							midisig[ch] = i;
							notecnt[ch]++;
						}
						else
#endif
						{
							len = adp-1;
							while (len) {
								writebyte(getdata());
								len--;
							}
						}
					}
					else {
						// MidiEvent
						switch (ev) {
						case 0xf2:
							len = 2;
							break;
						case 0xf3:
							len = 1;
							break;
						default:
							ch = ev & 0x0f;
							switch (ev & 0xf0) {
							case 0xf0:
								len = 0;
								break;
							case 0xc0:	// ProgramChange
							case 0xd0:	// ChannelAftertouch
								len = 1;
								break;
							case 0x80:	// NoteOff
#ifdef MIDI_MONITOR
								writebyte(getdata());
								writebyte(getdata());
								if (notecnt[ch])
									notecnt[ch]--;
								if (!notecnt[ch])
									midisig[ch] = 1;
								len = 0;
								break;
#endif
							case 0x90:	// NoteOn
#ifdef MIDI_MONITOR
							writebyte(getdata());
							i = getdata();
							writebyte(i);
							i = (i>>4)+1;
							if (i > 7) i = 7;
							midisig[ch] = i;
							notecnt[ch]++;
							len = 0;
							break;
#endif
							case 0xa0:	// PolyphonicAftertouch
							case 0xb0:	// ControlModeChange
							case 0xe0:	// PitchWheel
								len = 2;
								break;
							default:
								len = 0;
							}
						}
						adp = len;
						ade = ev;
						while (len) {
							writebyte(getdata());
							len--;
						}
					}
				}

#else //Backup

				while (state == REC) {
					ev = getdata();	// Auf Midi-Event warten
					if (state == STOP || state == ABORT_REC)
						break;
					write_time();
					if (!TCCR1B) {
						start_time();//TCCR1B = 8+2;	// Start Time
						lcd_string(DISP_REC, LINE_REC);
						lcd_setcur(RECFNUM_X, RECFNUM_Y);
						lcd_number(file_cnt+1, 1);
					}
					writebyte(ev);
			
					if (ev == 0xf0) {	// SysEx
						do {
							ev = getdata();
							writebyte(ev);
						} while (ev != 0xf7);
					}
					else if (ev < 0x80) {	// Running status
						len = adp-1;
						while (len > 0) {
							writebyte(getdata());
							len--;
						}
					}
					else {
						// MidiEvent
						switch (ev) {
						case 0xf2:
							len = 2;
							break;
						case 0xf3:
							len = 1;
							break;
						default:
							switch (ev & 0xf0) {
							case 0xf0:
								len = 0;
								break;
							case 0xc0:	// ProgramChange
							case 0xd0:	// ChannelAftertouch
								len = 1;
								break;
							case 0x80:	// NoteOff
							case 0x90:	// NoteOn
							case 0xa0:	// PolyphonicAftertouch
							case 0xb0:	// ControlModeChange
							case 0xe0:	// PitchWheel
								len = 2;
								break;
							default:
								len = 0;
							}
						}
						adp = len;
						while (len > 0) {
							writebyte(getdata());
							len--;
						}
					}
				}
#endif
				if (!TCCR1B || state == ABORT_REC) {
					stop_time();
					mmc_complete_write();
					filemode = FILEMODE_CLOSED; // Datei abmurksen!
				}
				else {
					stop_time();

					mmc_write_dword(0x00ff2f00UL);
					gl_timeset = sect + 1;	// Nächster freier Sektor
					mmc_complete_write();

#ifdef OLD_CARDSCAN
					mmc_write_start(last);
#else
					mmc_write_start(fat_filedata.startsect);
#endif
					writestring("MThd", 4);
					mmc_write_dword(6);
					mmc_write_dword(1);
					mmc_write_word(240);	// Fixes Tempo im Header
					writestring("MTrk", 4);
					mmc_write_dword(trk_len);
					mmc_write_word(255);
					mmc_write_byte(2);
					write_varlen(485);
					mmc_complete_write();

#ifdef OLD_CARDSCAN
					if (flags & FAT_FLAG)
#endif
					{
						fat_filedata.len = trk_len + 20;
						fat_closefile();
					}
#ifdef OLD_CARDSCAN
					else {
						file_pos[file_cnt] = last;
						last = gl_timeset;
					}
#endif
					file_num = file_cnt;
					file_cnt++;
				}

				print_main();
#endif
		}
		else if (state == PLAY) {
				// Wiedergabe starten
#ifndef LED_DISP
#ifdef RANDOMSONG
				if (ee_rep == 4
#ifdef DIREKTWAHL
					&& !(flags & DIRNUM_FLAG)
#endif
					) {
					file_num = random_song(file_num, file_cnt);
					fat_read_filedata(file_num);
					print_filename(NAME_LINE);
				}
#endif
#ifdef DIREKTWAHL
				flags &= ~DIRNUM_FLAG;
#endif
#endif
#ifdef LED_DISP
				PORTC &= ~1;
#endif
				clear_time();
				transpose = 0;
#ifndef LED_DISP
				if (ee_midimon&2) // Anzeige nur wenn keine Midi-Pegelanzeige
					lcd_string(DISP_BLANK, LINE_TEMPO);
				else
					print_playinfo();
#endif
				
#ifdef OLD_CARDSCAN
				if (flags & FAT_FLAG)
#endif
				{
					fat_openfile(FILEMODE_READ, file_num);
				}
#ifdef OLD_CARDSCAN
				else {
					mmc_load_start(file_pos[file_num]);
				}
#endif

#ifdef BUFFERED_READ
				init_playbuf();
#endif
				if (!checkstring("MThd", 4)) {
					mmc_complete_read();
#ifdef LED_DISP
					error();
#endif
					//lcd_string(DISP_FILEERR, LINE_FILEERR);
#ifdef OLD_CARDSCAN
					if (flags & FAT_FLAG)
#endif
						fat_closefile();
					//delay_ms(1000);
					state = ERR+DISP_FILEERR;
					//print_main();
					break;
				}

				fetchbyte();
				checkbyte(0);	// Format 0 prüfen
				checkbyte(0);
				checkbyte(6);
				checkbyte(0);
				checkbyte(0);
				checkbyte(0);
				checkbyte(1);
				temp.byte[1] = fetchbyte();	// Tempo lesen
				temp.byte[0] = fetchbyte();
				gl_timeset = temp.word[0];

				TCNT1 = 0;
				// Tempo berechnen
				if (gl_timeset < 0x8000)	// Delta-Ticks
					tempo = 500000UL/gl_timeset;
				else				// SMPTE-Timeformat
					tempo = 1000000UL/(-(s08)(gl_timeset>>8))/gl_timeset;

				checkstring("MTrk", 4);	// Track lesen

				temp.byte[3] = fetchbyte();	// Länge lesen
				temp.byte[2] = fetchbyte();
				temp.byte[1] = fetchbyte();
				temp.byte[0] = fetchbyte();
				trk_len = temp.udword;
				time = 0;
				speed = 0;
				gl_lyr = false;
#ifdef MIDI_MONITOR
				for (i=0; i<16; i++) {
					midisig[i] = 0;
					notecnt[i] = 0;
				}
				tickcnt = 0;
				decaycnt = 0;
#endif

				lyrx = 0; lyry = 0;

				calc_tempo();
				start_time();	// Start Time

				while (state == PLAY) {
					if (trk_len == 0) {	// Tracklen wird heruntergezählt. Bei 0 = Ende erreicht
						state = STOP;
						break;
					}
					wait_for_deltatime();
					if (state == STOP || state >= ERR)
						break;
					ev = fetchbyte();
					if (ev == 0xFF) {	// META-EVENT
						ev = fetchbyte();
						switch (ev) {
							case 0x2f:	// End Of Track
								checkbyte(0);
								trk_len = 0;
								break;
							case 0x51:	// Tempo Change
								checkbyte(3);
								temp.byte[3] = 0;
								temp.byte[2] = fetchbyte();
								temp.byte[1] = fetchbyte();
								temp.byte[0] = fetchbyte();
								TCNT1 = 0;
								tempo = temp.udword/gl_timeset;
								calc_tempo();
								break;
							//case 0x01:	// Text
							case 0x05:	// Lyrics
								if (ee_lyr) {
									len = 0;
									gl_lyr = true;
									temp.word[0] = fetch_varlen();
									lcd_setcur(lyrx, lyry);
									while (temp.word[0]) {
										temp.word[0]--;
										ev = fetchbyte();
#ifndef LED_DISP
										if (ev >= ' ' && ev <= '~') {
											lcd_data(ev);
											lyrx++;
											if (lyrx == LINESIZE) {
												lyrx = 0;
												lyry++;
												if (lyry == MAX_LINES)
													lyry = 0;
												lcd_string(DISP_BLANK, lyry);
												lcd_setcur(0, lyry);
											}
										}
#endif
									}
									break;
								}
							default:
								temp.word[0] = fetch_varlen();
								while (temp.word[0]) {
									temp.word[0]--;
									fetchbyte();
								}
						}
					}
					else if (ev == 0xf0) {	// SYSEX
						sendbyte(0xf0);
						temp.word[0] = fetch_varlen();
						while (temp.word[0]) {
							temp.word[0]--;
							ev = fetchbyte();
							sendbyte(ev);
						}
					}
					else if (ev >= 0x80) {	// MIDI-EVENT
						trp = false;
						//sendbyte(ev);
						ch_nact = 1;
						switch (ev) {
							case 0xf2:
								len = 2;
								break;
							case 0xf3:
								len = 1;
								break;
							default:
								ch = ev & 0x0f;
								ch_nact = 0;
								switch (ev & 0xf0) {
									case 0xf0:
										len = 0;
										break;
									case 0xc0:	// ProgramChange
									case 0xd0:	// ChannelAftertouch
										len = 1;
										break;
									case 0x80:	// NoteOff
#ifdef MIDI_MONITOR
										if (notecnt[ch])
											notecnt[ch]--;
										if (!notecnt[ch])
											midisig[ch] = 1;
#endif
									case 0x90:	// NoteOn
									case 0xa0:	// PolyphonicAftertouch
										trp = true;
									case 0xb0:	// ControlModeChange
									case 0xe0:	// PitchWheel
										len = 2;
										break;
									default:
#ifdef LED_DISP
										error();
#endif
										//lcd_string(DISP_FILEERR, LINE_FILEERR);
										state = ERR+DISP_FILEERR;
										//stop_time();
										//delay_ms(1000);
										len = 0;
										trk_len = 0;
								}
						}
						adp = len;	// Länge merken (für Running Status)
#ifdef MIDI_MONITOR
						ade = ev;
#endif
						if (ch_nact || !(ee_mute & ((u16)1<<ch))) {
							sendbyte(ev);
							if (trp) {
								i = fetchbyte();
								if (ch != ee_drumch)
									i += transpose;
								sendbyte(i);
								i = fetchbyte();
								sendbyte(i);
#ifdef MIDI_MONITOR
								if ((ade&0xf0) == 0x90) { // NoteOn
									i = (i>>4)+1;
									if (i > 7) i = 7;
									midisig[ch] = i;
									notecnt[ch]++;
								}
#endif
							}
							else {
#ifdef MIDI_MONITOR
								if ((ev&0xf0) == 0xB0) { // ControlMode AllNotesOff/AllSoundsOff auf midisig mappen
									i = fetchbyte();
									sendbyte(i);
									if (i == 0x78 || i == 0x7B) {
										midisig[ch] = 1;
										notecnt[ch] = 0;
									}
									sendbyte(fetchbyte());
								}
								else
#endif
								while (len) {
									sendbyte(fetchbyte());	// Bytes durchschleifen
									len--;
								}
							}
						}
						else {
							while (len) {
								fetchbyte();	// Bytes ignorieren
								len--;
							}
						}
					}
					else {			// Running Status
						if (ch_nact || !(ee_mute & ((u16)1<<ch))) {
							if (trp && ch != ee_drumch)
								sendbyte(ev+transpose);
							else
								sendbyte(ev);
#ifdef MIDI_MONITOR
							if ((ade&0xf0) == 0x90) { // NoteOn
								i = fetchbyte();
								sendbyte(i);
								i = (i>>4)+1;
								if (i > 7) i = 7;
								midisig[ch] = i;
								notecnt[ch]++;
							}
							else if ((ade&0xf0) == 0x80) { // NoteOff
								i = fetchbyte();
								sendbyte(i);
								if (notecnt[ch])
									notecnt[ch]--;
								if (!notecnt[ch])
									midisig[ch] = 1;
							}
							else if ((ade&0xf0) == 0xB0) {
								i = fetchbyte();
								sendbyte(i);
								if (ev == 0x78 || ev == 0x7B) {
									midisig[ch] = 1;
									notecnt[ch] = 0;
								}
							}
							else
#endif
							{
								len = adp - 1;
								while (len) {
									sendbyte(fetchbyte());
									len--;
								}
							}
						}
						else {
							len = adp - 1;
							while (len) {
								fetchbyte();	// Bytes ignorieren
								len--;
							}
						}
					}
				}
				stop_time();
#ifdef OLD_CARDSCAN
				if (flags & FAT_FLAG)
#endif
					fat_closefile();
#ifdef OLD_CARDSCAN
				else
					mmc_complete_read();
#endif
				send_all_off();
#ifdef LED_DISP
				PORTC |= 1;
#endif
				// repeat modes:
				// 0=single song
				// 1=repeat single song
				// 2=all songs
				// 3=repeat all songs
				// 4=random song
				if (trk_len == 0) {
#ifdef LED_DISP
						if (file_num < file_cnt-1) {
							file_num++;
							state = PLAY;
						}
						else {
							file_num = 0;
							state = PLAY;
						}
#else
					if (ee_rep > 1) {
						if (file_num < file_cnt-1) {
							file_num++;
							state = PLAY;
						}
						else if (ee_rep == 3) {
							file_num = 0;
							state = PLAY;
						}
						if (ee_rep == 4) {
							file_num = random_song(file_num, file_cnt);
							state = PLAY;
						}
					}
					else if (ee_rep == 1) {
						state = PLAY;
					}
#endif
				}
#ifndef LED_DISP
				if (state == PLAY) {
					print_main();
					state = PLAY;
				}
				else
					print_main();
#endif
		}
		else if (state == DELETE) {

#ifndef LED_DISP
			lcd_string(DISP_DELETE, LINE_DELETE);	// Bildschirm anzeigen
			file_cnt--;
#ifdef OLD_CARDSCAN
			if (flags & FAT_FLAG) {
				fat_delete_file();
			}
			else {
				mmc_write_start(file_pos[file_cnt]);
				mmc_write_byte(0);
				mmc_complete_write();
			}
#else
			fat_delete_file();
#endif
			if (file_num == file_cnt)
				file_num--;
			delay_ms(1024);
			print_main();
#endif
		}
		else if (state == STOP) {
			gl_lyr = false;
			key_detect();
		}
		else if (state == MENU) {
			key_detect();
		}
		else {
			stop_time();
#ifndef LED_DISP
			lcd_string(state-ERR, 1);
			delay_ms(1024);
			print_main();
#endif
		}
	}
}


//----------------------------------------------------------
// IRQ

/** Timer for delta times.
*/
ISR(SIG_OUTPUT_COMPARE1A) {
	time++;
	if ((flags & DT_FLAG) && time>=delta_time) {
		flags &= ~DT_FLAG;
		time = 0;
/*		rtime += delta_time; // Versuch, die Echtzeit aus den Deltatimes zu errechnen
		while (rtime >= ?rtsek) {
			rtime -= ?rtsek;
			time_sec++;
			if (time_sec == 60) {
				time_sec = 0;
				time_min++;
			}
			timeflag |= TIME_DISP;
		}*/
	}
	if (flags & OCCHNG_FLAG) {
		flags &= ~OCCHNG_FLAG;
		OCR1A = new_ocr1a;
	}
}


/** Timer for real time clock.
*/
#ifdef __AVR_ATmega168__
  ISR(SIG_OUTPUT_COMPARE2A) {
#else
  ISR(SIG_OUTPUT_COMPARE2) {
#endif
#ifndef LED_DISP
#ifdef MIDI_MONITOR
	tickcnt++;
#endif
#ifdef KEY_REPEAT
	timeflag |= TIME_KPT;
#endif
	real_time++;
	if (real_time == limit_count) {
		real_time = 0;
		time_sec++;
		if (time_sec == 60) {
			time_sec = 0;
			time_min++;
		}
		timeflag |= TIME_DISP;
	}
#endif
}


/** Timer for debouncing buttons.
*/
#ifdef __AVR_ATmega168__
ISR(SIG_WATCHDOG_TIMEOUT) {
	wdt_disable();
	flags |= KEY_FLAG;
#ifndef LED_DISP
  #ifdef RC5
	if (flags & IR_FLAG && rc5.flip == -1)
		flags &= ~IR_FLAG;
  #endif
#endif
}
#else
ISR(SIG_OVERFLOW0) {
	TCCR0 = 0;
	flags |= KEY_FLAG;
}
#endif



/** UART RX.
*/
/** akashi
#ifdef __AVR_ATmega168__
  ISR(SIG_USART_RECV) {
#else
  ISR(SIG_UART_RECV) {
#endif
#ifndef LED_DISP
	u08 tmp = UDR;
	if (state != REC || tmp == 0xf8 || tmp == 0xfe)	// Midi Clock und Synch ausblenden
		return;
	rx_buf[rx_wrp++] = tmp;
	rx_wrp &= RX_BUF_SIZE-1;
	if (rx_wrp == rx_rdp) {	// Buffer overflow!
		state = ERR+DISP_BUFERR;
	}
#endif
}
*/

#ifdef CALIBRATE
/** Pinänderung für Kalibrierung.
*/
ISR(SIG_PIN_CHANGE2) {
	if (!(PIND & 1)) {
		NOP;
		TCCR1B = 1;
	}
	else {
		TCCR1B = 0;
		PCICR &= ~0x04;
	}
}
#endif

