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
- Record bleibt am Schluss h舅gen bei USE_RTC
- Midi-Protokoll ist irgendwie anders (Evanescence-Lied)
*/

#define __AVR_ATmega168__	// akashi

#include "mrmidi2.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include "lcd.h"
#include "mmc.h"
#include "spi.h"
#include "delay.h"
#include "fat16.h"
#include "types.h"

#include "rc5.h"

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

#ifndef F_CPU
	#define F_CPU 10000000UL
#endif

// RTC Timer 2
//#define RTC_PRESCALER		256	// for 8MHz
#define RTC_PRESCALER		320	// for 10MHz
#define RTC_RELOAD_TIMER	250
#define RTC_SOFT_COUNT		125
#if F_CPU != (RTC_PRESCALER*RTC_RELOAD_TIMER*RTC_SOFT_COUNT)
	#error RTC_NOT_ADJUSTED
#endif

#define MAX_TICK	5
#define MAX_DECAY	7

// Buttons
//#define KEY_PORT	PIND	// akashi
//#define KEY_PORT2	PINB	// akashi

#define KEY_MASK	0xf8
#define KEY_PLAY	0x80

#define KEY_REP_TIME_INIT	80
#define KEY_REP_TIME		20
#define FAST_SCROLL			30
#define KEY_REP_TIME_INIT_IR	5
#define KEY_REP_TIME_IR			1

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
#define RC5_CODE_MON	0x3E

#define DEFAULT_OSCCAL_POS	32


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
	M_MUTE,
	M_REP,
	M_LIGHT,
	M_MIDIMON,
	M_MANCAL,
	MAX_MENU
};

// Flags
#define KEY_FLAG	0x01
#define DT_FLAG		0x02
#define OCCHNG_FLAG	0x04
#define IR_FLAG		0x08
#define IR_TGL		0x10
#define DIRNUM_FLAG	0x40
#define MMCOK_FLAG	0x80

#define TIME_DISP	0x01
#define TIME_KPT	0x02

// MMC

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
u08 time_min, time_sec, real_time=0, limit_count=RTC_SOFT_COUNT; // RTC
s08 speed, transpose;
u08 file_num=0, file_cnt=0;	// File
u16 *file_pos;
// Menu values
u08 ee_drumch=9, ee_lyr=1, ee_rep=2, ee_midimon=1, menu_cnt, s_ch=0, gl_lyr=false;
u16 ee_mute=0;
u08 timeflag=0; // Multi-purpose Timer 2
u08 midisig[16], notecnt[16], tickcnt, decaycnt;
#define PLAYBUF_SIZE RX_BUF_SIZE
#define PBS_EMPTY 1
#define PBS_FULL 2
#define playbuf rx_buf
#define playbuf_rdp rx_rdp
#define playbuf_wrp rx_wrp
u08 playbuf_state=PBS_EMPTY;

extern struct fat_filedata_st fat_filedata;
extern u32 sect;
extern u08 filemode;

extern uint16_t	gLEDs[8];	// akashi
extern uint16_t	gPianoKeys[8];	// akashi
void error(uint8_t);		// akashi

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
static void send_all_ch(u08 data);
static void send_all_off(void);
static u08 getdata(void);
static void writebyte(u08 b);
static void writestring(u08 *buf, u08 len);
static void write_varlen(u32 value);
static void write_time(void);
static void print_time(void);
static void print_pause(void);
static void print_main(void);
static void print_menu(void);
static void print_transpose(void);
static void print_playinfo(void);
static void print_tempo(void);
static void draw_mute(void);


//---------------------------------------------------

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

	// Ports
	lcd_string(DISP_MAIN, LINE_MAIN);
	lcd_string(DISP_MIDICTR, LINE_BOOT);
	delay_ms(1024);

	// UART
	// Hilfsmakro zur UBRR-Berechnung ("Formel" laut Datenblatt)
	//#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)
	//UBRRH = (uint8_t)(UART_UBRR_CALC(UART_BAUD_RATE,F_CPU)>>8);
	//UBRRL = (uint8_t)UART_UBRR_CALC(UART_BAUD_RATE,F_CPU);
	//UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	// Midi device silence

	// Read parameters from EEPROM
	if (eeprom_read_byte(EE_VERSION_POS) != 0x27) {	// eeprom empty
		eeprom_write_byte(EE_VERSION_POS, 0x27);
		menu_save();
	}
	else {	// load parameters
		ee_drumch = eeprom_read_byte((const uint8_t*)EE_DRUMCH_POS);
		if (ee_drumch > 15)	// invalid channel
			ee_drumch = 10;
		ee_lyr = eeprom_read_byte((const uint8_t*)EE_LYRICS_POS);
		ee_mute = eeprom_read_word((const uint16_t*)EE_MUTE_POS);
		ee_rep = eeprom_read_byte((const uint8_t*)EE_REPEAT_POS);
		if (ee_rep > 4)
			ee_rep = 2;

		ee_midimon = eeprom_read_byte((const uint8_t*)EE_MONITOR_POS);
		if (ee_midimon > 3)
			ee_midimon = 1;
		temp = eeprom_read_byte((const uint8_t*)EE_OSCCAL_POS);
		if (temp >= 128 && temp <= 200)
			OSCCAL = temp;
	}

	//Initialisierung der MMC/SD-Karte
	MMC_hw_init();
	spi_init();
//	rc5_init(RC5_ALL);
	sei();
	delay_ms(100);
	TCCR1B = 1;		// タイマ１プリスケーラを１に設定
	if (MMC_init()) {
		lcd_string(DISP_OK, LINE_OK);
		flags |= MMCOK_FLAG;
	}
	else {
		lcd_string(DISP_CARDERR, LINE_CARDERR);
		// akashi
		for (;;) {
			delay_ms(5000);
		}
	}
	TCCR1B = 0;		// タイマ１プリスケーラを停止
	srand(TCNT1);
	TIFR1 |= 7;		// タイム１割り込み結果をクリア
	TCNT1 = 0;		// タイマ１カウンタを初期化

	// Timer 0: Button Debounce
	// Timer 1: Delta Time Timer
	tempo = 2058;
	OCR1A = 2058;	// 
	// Timer 2: RTC
	OCR2 = RTC_RELOAD_TIMER;

	// Interrupts
	// mega168
	TIMSK1 = 2;
	TIMSK2 |= 2;

}


/** Save EEPROM data.
*/
static void menu_save(void) {
	eeprom_write_byte((uint8_t*)EE_DRUMCH_POS, ee_drumch);
	eeprom_write_byte((uint8_t*)EE_LYRICS_POS, ee_lyr);
	eeprom_write_word((uint16_t*)EE_MUTE_POS, ee_mute);
	eeprom_write_byte((uint8_t*)EE_REPEAT_POS, ee_rep);
	eeprom_write_byte((uint8_t*)EE_MONITOR_POS, ee_midimon);
	eeprom_write_byte((uint8_t*)EE_OSCCAL_POS, OSCCAL);
}


//----------------------------------------------------------
// UART

/** Send UART byte.
	@param Byte to send
*/
static void sendbyte(u08 byte) {
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = byte;
}

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
	}
	u08 b = rx_buf[rx_rdp++];
	rx_rdp &= RX_BUF_SIZE-1;
	return b;
}


//----------------------------------------------------------
// MMC IO

// Read from MMC

/** Abspielpuffer f・len.
	Liest MMC-Daten und f・lt damit den Puffer ganz voll.
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
	Wenn der Puffer leer ist, wird er neu gef・lt.
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
			}
			fill_playbuf();
		}
	}
}


/** Tempo bestimmen.
	Berechnet Zeitbasis zum Abspielen der Delta Times.
	Passt auch die Echtzeitanzeige-Geschwindigkeit an.
*/
static void calc_tempo(void) {
	if (dspd) {
		TCNT1 = 0;
		dspd = 0;
	}

	tempo = tempo * 5 / 4;	// akashi

	new_ocr1a = tempo - (tempo>>4)*speed;
	flags |= OCCHNG_FLAG;
	if (!gl_lyr && !(ee_midimon&2)) {
		print_tempo();
	}
	limit_count = RTC_SOFT_COUNT - (RTC_SOFT_COUNT*speed)/(u16)16;
	if (real_time >= limit_count)
		real_time = limit_count-1;
}

//----------------------------------------------------------
// Buttons

/** Entprellen der Tasten.
	ATmega168 benutzt daf・ den WDT.
	@param WDT-Modus [1=Timermodus, 0=WDT]
*/
static void debounce(u08 mode) {
	flags &= ~KEY_FLAG;
	cli();
	WDTCSR = 0x18;	// Setup WDT-Init
	WDTCSR = 0x40|mode;	// WDIE=1->Interrupt mode; 32ms Timeout
	wdt_reset();
	sei();
}


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
		{
			fat_read_filedata(file_num);
			print_filename(NAME_LINE);
			print_filesize();
		}
		start_time();	// Start Time
		flags |= DIRNUM_FLAG; // Verhindert Zufallswiedergabe bei Direktwahl
	}
}

/** Check buttons.
	Tastenabfrage/IR-Auswertung.
*/
static void key_detect(void) {
	static u08 ok=0, wur=0;
	static u08 kpt=0, krc=0;//!
	u08 lwur=0;

	u08 k = /* ((u08)(~KEY_PORT)&KEY_MASK) | ((u08)(~KEY_PORT2)&KEY_MASK2) */ 0x00;	// akashi

	extern u08 gKey;

	if (gKey) {
		gKey = 0x00;
		k = KEY_STOP;
	}


	if ((k&KEY_NEXT) && (k&KEY_LAST)) {
		k = RC5_CODE_MENU; lwur=1;
	}
	else if ((k&KEY_LEFT) && (k&KEY_RIGHT)) {
		k = RC5_CODE_DELETE; lwur=1;
	}
	else if ((k&KEY_LEFT) && (k&KEY_NEXT)) {
		k = RC5_CODE_MON; lwur=1;
	}
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
		debounce(3); // L舅geres Entprellen, weil >60ms Sendeabstand der FB
	}

	if ((k != ok
		 || (krc < 2 && (kpt >= KEY_REP_TIME_INIT || ((flags&IR_FLAG) && kpt >= KEY_REP_TIME_INIT_IR)))
		 || (krc > 1 && (kpt >= KEY_REP_TIME || ((flags&IR_FLAG) && kpt >= KEY_REP_TIME_IR))))
		&& (!wur || (wur && !k)))
	{
		kpt = 0;
		if (k != ok && !(flags&IR_FLAG)) {
			debounce(1);
			ok = k;
		}

		if (ok) {
			TCCR2 = 6+8;
			if (krc < 255)
				krc++;
			k = ok & 0x7f;

			if (state == MENU) {
				if (k == RC5_CODE_NEXT) { // Men・bl舩tern auf
					menu_cnt--;
					if (menu_cnt >= MAX_MENU)
						menu_cnt = MAX_MENU-1;
					print_menu();
				}
				else if (k == RC5_CODE_LAST) { // Men・bl舩tern ab
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
						case M_MANCAL:
							if (OSCCAL > 128)
								OSCCAL--;
							break;
						case M_MIDIMON:
							if (ee_midimon)
								ee_midimon--;
							break;
					}
					print_menu();
				}
				else if (k == RC5_CODE_RIGHT) { // Wert vergrern
					switch (menu_cnt) {
						case M_DRUMCH:
							if (ee_drumch < 0x0f)
								ee_drumch++;
							break;
						case M_LYR:
							if (!ee_lyr)
								ee_lyr = 1;
							break;
						case M_MUTE:
							s_ch++;
							s_ch &= 0x0f;
							break;
						case M_REP:
							if (ee_rep < 4)
								ee_rep++;
							break;
						case M_MANCAL:
							if (OSCCAL < 200)
								OSCCAL++;
							break;
						case M_MIDIMON:
							if (ee_midimon < 3)
								ee_midimon++;
							break;
					}
					print_menu();
				}
				else if (k == RC5_CODE_STOP || k == RC5_CODE_REC) { // Schlieﾟen und speichern
					print_main();
					menu_save();
				}
				else if (k == RC5_CODE_MENU) { // Schlieﾟen und verwerfen
					print_main();
				}
			}
			else if (file_cnt && state != REC) {

			// Play
				if (state == PLAY) {
					if (k == RC5_CODE_LAST) { // Transponieren abw舐ts
						if (transpose > -24) {
							transpose--;
							print_transpose();
							send_all_off();
						}
					}
					else if (k == RC5_CODE_NEXT) { // Transponieren aufw舐ts
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
				}

			// Stop
				else if (state == STOP) {
					if (k == RC5_CODE_PLAY) { // Wiedergabe
						state = PLAY;
					}
					else if (k == RC5_CODE_REC) { // Aufnahme
						state = REC;
					}
					else if (k <= 9) { // Ziffernwahl
						direkt_wahl(k);
					}
					else if (k == RC5_CODE_DELETE) { // Datei lchen
						state = DELETE;
					}
					else if (k == RC5_CODE_MENU) { // Men・fnen
						menu_cnt = 0;
						print_menu();
						state = MENU;
					}
					else if (k == RC5_CODE_LAST) { // Bl舩tern r・kw舐ts
						if (!file_num)
							file_num = file_cnt;
						file_num--;
						lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
						lcd_number(file_num+1, 1);
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
					}
					else if (k == RC5_CODE_NEXT) { // Bl舩tern vorw舐ts
						file_num++;
						if (file_num == file_cnt)
							file_num = 0;
						lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
						lcd_number(file_num+1, 1);
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
					}
					else if (k == RC5_CODE_PLAYNEXT) { // Skip zum n臘hsten Song
						file_num++;
						if (file_num == file_cnt)
							file_num = 0;
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
						lcd_string(DISP_BLANK, 1);
						state = PLAY;
					}
					else if (k == RC5_CODE_PLAYLAST) { // Skip zum vorigen Song
						if (file_num == 0)
							file_num = file_cnt;
						file_num--;
						{
							fat_read_filedata(file_num);
							print_filename(NAME_LINE);
							print_filesize();
						}
						lcd_string(DISP_BLANK, 1);
						state = PLAY;
					}
				}
			}
			else {

			// No files on card or state is REC
				if (state == STOP) {
					if (k == RC5_CODE_MENU) { // Men・fnen
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
				}
			}
			wur = lwur;
		}
		else {
			wur=0;
			krc = 0;
		}
	}
	if (ok && kpt < 255 && (timeflag&TIME_KPT)) {
		timeflag &= ~TIME_KPT;
		kpt++;
	}
}

/** Anzeige Hauptbildschirm.
	Zeigt Nummer, Anzahl, Dateiname und Gre an.
*/
static void print_main(void) {
	if (file_cnt) {
		lcd_string(DISP_SELECT, LINE_SELECT);
		lcd_setcur(STOPFNUM_X, STOPFNUM_Y);
		lcd_number(file_num+1, 1);
		{
			fat_read_filedata(file_num);
			print_filename(NAME_LINE);
			print_filesize();
		}
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


/** Anzeige Men・
*/
static void print_menu(void) {
	lcd_string(DISP_MENU, LINE_MENU);
	lcd_string(DISP_MDRUM+menu_cnt, LINE_OPTION);
	switch (menu_cnt) {
		case M_DRUMCH:
			lcd_setcur(DRUMCH_X, LINE_OPTION);
			lcd_number(ee_drumch+1, 0);
			break;
		case M_LYR:
			lcd_setcur(LYR_X, LINE_OPTION);
			if (!ee_lyr) {
				lcd_data('O');lcd_data('f');lcd_data('f');
			}
			else {
				lcd_data('O');lcd_data('n');
			}
			break;
		case M_MUTE:
			draw_mute();
			break;
		case M_REP:
			lcd_setcur(REP_X, LINE_OPTION);
			switch (ee_rep) {
			case 0:
				lcd_data('O');lcd_data('f');lcd_data('f'); break;
			case 1:
				lcd_data('S');lcd_data('o');lcd_data('n');lcd_data('g'); break;
			case 2:
				lcd_data('D');lcd_data('i');lcd_data('r');lcd_data(' ');lcd_data('o');lcd_data('n');lcd_data('c');lcd_data('e'); break;
			case 3:
				lcd_data('A');lcd_data('l');lcd_data('l'); break;
			case 4:
				lcd_data('R');lcd_data('a');lcd_data('n');lcd_data('d');lcd_data('o');lcd_data('m'); break;
			}
			break;
		case M_MANCAL:
			lcd_setcur(MANCAL_X, LINE_OPTION);
			lcd_number(OSCCAL, 1);
			break;
		case M_MIDIMON:
			lcd_setcur(MIDIMON_X, LINE_OPTION);
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
			break;
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

//----------------------------------------------------------
// Time

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

/** Zeit zur・ksetzen.
*/
static void clear_time(void) {
	TCCR2 = 0;
	time_sec = 0;
	time_min = 0;
	real_time = 0;
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
void main2(void) {
union {
	u08 byte[4];
	u16 word[2];
	u32 udword;
} temp;
	u08 ev, len, adp=0, ade=0, lyrx, lyry, trp=0, ch=0, ch_nact=0, i;
	u16 gl_timeset;
    bool isEqualLEDsAndKeyboard;

	file_pos = (u16 *)sharedmem;
	hw_init();

	if (flags & MMCOK_FLAG) {
		if (!fat_init()) {
			lcd_string(DISP_CARDERR, LINE_CARDERR);
			return;			// akashi
			error(0xee);	// akashi
		}
		else {
			fat_count_files();
		}
		file_num = 0;
		print_main();
	}

	while (1) {
		if (state == REC) {
				// Rekorder-Task
				fat_openfile(FILEMODE_WRITE, file_cnt);
				mmc_write_start(fat_filedata.startsect+1);
				OCR1A = 500000UL/240;	// Fixes Tempo festlegen
				trk_len = 490;
				time = 0;
				rx_wrp = 0;	// Puffer leeren
				rx_rdp = 0;
				for (i=0; i<16; i++) {
					midisig[i] = 0;
					notecnt[i] = 0;
				}
				tickcnt = 0;
				decaycnt = 0;

				lcd_string(DISP_RECRD, LINE_RECRD);	// Bildschirm anzeigen
				lcd_setcur(RECFNUM_X, RECFNUM_Y);
				lcd_number(file_cnt+1, 1);
				lcd_string(DISP_BLANK, LINE_REC_BLANK);

				clear_time();

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
					} else if (ev < 0x80) {	// Running status
						if ((ade&0xf0) == 0x80) { // NoteOff
							writebyte(getdata());
							if (notecnt[ch])
								notecnt[ch]--;
							if (!notecnt[ch])
								midisig[ch] = 1;
						}else if ((ade&0xf0) == 0x90) { // NoteOn
							i = getdata();
							writebyte(i);
							i = (i>>4)+1;
							if (i > 7) i = 7;
							midisig[ch] = i;
							notecnt[ch]++;
						} else {
							len = adp-1;
							while (len) {
								writebyte(getdata());
								len--;
							}
						}
					} else {
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
								writebyte(getdata());
								writebyte(getdata());
								if (notecnt[ch])
									notecnt[ch]--;
								if (!notecnt[ch])
									midisig[ch] = 1;
								len = 0;
								break;
							case 0x90:	// NoteOn
							writebyte(getdata());
							i = getdata();
							writebyte(i);
							i = (i>>4)+1;
							if (i > 7) i = 7;
							midisig[ch] = i;
							notecnt[ch]++;
							len = 0;
							break;
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

				if (!TCCR1B || state == ABORT_REC) {
					stop_time();
					mmc_complete_write();
					filemode = FILEMODE_CLOSED; // Datei abmurksen!
				}
				else {
					stop_time();

					mmc_write_dword(0x00ff2f00UL);
					gl_timeset = sect + 1;	// N臘hster freier Sektor
					mmc_complete_write();

					mmc_write_start(fat_filedata.startsect);
					writestring((u08*)"MThd", 4);
					mmc_write_dword(6);
					mmc_write_dword(1);
					mmc_write_word(240);	// Fixes Tempo im Header
					writestring((u08*)"MTrk", 4);
					mmc_write_dword(trk_len);
					mmc_write_word(255);
					mmc_write_byte(2);
					write_varlen(485);
					mmc_complete_write();

					{
						fat_filedata.len = trk_len + 20;
						fat_closefile();
					}
					file_num = file_cnt;
					file_cnt++;
				}

				print_main();
		} else if (state == PLAY) {
			// Wiedergabe starten
			if (ee_rep == 4 && !(flags & DIRNUM_FLAG)) {
				file_num = random_song(file_num, file_cnt);
				fat_read_filedata(file_num);
				print_filename(NAME_LINE);
			}
			flags &= ~DIRNUM_FLAG;
			clear_time();
			transpose = 0;
			if (ee_midimon&2) // Anzeige nur wenn keine Midi-Pegelanzeige
				lcd_string(DISP_BLANK, LINE_TEMPO);
			else
				print_playinfo();
			{
				fat_openfile(FILEMODE_READ, file_num);
			}
			init_playbuf();
			if (!checkstring((u08*)"MThd", 4)) {
				mmc_complete_read();
				fat_closefile();
				state = ERR+DISP_FILEERR;
				error(0xcc);	// akashi
				break;
			}
			fetchbyte();
			checkbyte(0);	// Format 0 pr・en
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
			checkstring((u08*)"MTrk", 4);	// Track lesen
			temp.byte[3] = fetchbyte();	// L舅ge lesen
			temp.byte[2] = fetchbyte();
			temp.byte[1] = fetchbyte();
			temp.byte[0] = fetchbyte();
			trk_len = temp.udword;
			time = 0;
			speed = 0;
//			speed = -4;		// a little fast
//			speed = -5;		// a little slow
			gl_lyr = false;
			for (i = 0; i < 16; i++) {
				midisig[i] = 0;
				notecnt[i] = 0;
			}
			tickcnt = 0;
			decaycnt = 0;
			lyrx = 0; lyry = 0;
			calc_tempo();
			start_time();	// Start Time
			while (state == PLAY) {
				
				// LEDとキーボードが一致しているか調べる。
				isEqualLEDsAndKeyboard = true;
				for (i = 0; i < 8; i++) {
					if (gLEDs[i] != gPianoKeys[i]) {
						isEqualLEDsAndKeyboard = false;
						break;
					}	
				}
				if (!isEqualLEDsAndKeyboard) {
					if (TCCR1B)
						stop_time();
					continue;
				}
				start_time();
				
				if (trk_len == 0) {	// Tracklen wird heruntergez臧lt. Bei 0 = Ende erreicht
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
					case 0x05:	// Lyrics
						if (ee_lyr) {
							len = 0;
							gl_lyr = true;
							temp.word[0] = fetch_varlen();
							lcd_setcur(lyrx, lyry);
							while (temp.word[0]) {
								temp.word[0]--;
								ev = fetchbyte();
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
				} else if (ev == 0xf0) {	// SYSEX
					sendbyte(0xf0);
					temp.word[0] = fetch_varlen();
					while (temp.word[0]) {
						temp.word[0]--;
						ev = fetchbyte();
						sendbyte(ev);
					}
				} else if (ev >= 0x80) {	// MIDI-EVENT
					trp = false;
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
							if (notecnt[ch])
								notecnt[ch]--;
							if (!notecnt[ch])
								midisig[ch] = 1;
						case 0x90:	// NoteOn
						case 0xa0:	// PolyphonicAftertouch
							trp = true;
						case 0xb0:	// ControlModeChange
						case 0xe0:	// PitchWheel
							len = 2;
							break;
						default:
							state = ERR+DISP_FILEERR;
							len = 0;
							trk_len = 0;
						}
					}
					adp = len;	// L舅ge merken (f・ Running Status)
					ade = ev;
					if (ch_nact || !(ee_mute & ((u16)1 << ch))) {
						sendbyte(ev);
						if (trp) {
							i = fetchbyte();
							if (ch != ee_drumch)
								i += transpose;
							sendbyte(i);
							i = fetchbyte();
							sendbyte(i);
							if ((ade&0xf0) == 0x90) { // NoteOn
								i = (i>>4)+1;
								if (i > 7) i = 7;
								midisig[ch] = i;
								notecnt[ch]++;
							}
						} else {
							if ((ev&0xf0) == 0xB0) { // ControlMode AllNotesOff/AllSoundsOff auf midisig mappen
								i = fetchbyte();
								sendbyte(i);
								if (i == 0x78 || i == 0x7B) {
									midisig[ch] = 1;
									notecnt[ch] = 0;
								}
								sendbyte(fetchbyte());
							} else
								while (len) {
									sendbyte(fetchbyte());	// Bytes durchschleifen
									len--;
								}
							}
						} else {
							while (len) {
								fetchbyte();	// Bytes ignorieren
								len--;
							}
						}
					} else {			// Running Status
						if (ch_nact || !(ee_mute & ((u16)1 << ch))) {
							if (trp && ch != ee_drumch)
								sendbyte(ev + transpose);
							else
								sendbyte(ev);
							if ((ade&0xf0) == 0x90) { // NoteOn
								i = fetchbyte();
								sendbyte(i);
								i = (i >> 4)+1;
								if (i > 7) i = 7;
								midisig[ch] = i;
								notecnt[ch]++;
							} else if ((ade & 0xf0) == 0x80) { // NoteOff
								i = fetchbyte();
								sendbyte(i);
								if (notecnt[ch])
									notecnt[ch]--;
								if (!notecnt[ch])
									midisig[ch] = 1;
							} else if ((ade & 0xf0) == 0xB0) {
								i = fetchbyte();
								sendbyte(i);
								if (ev == 0x78 || ev == 0x7B) {
									midisig[ch] = 1;
									notecnt[ch] = 0;
								}
							} else {
								len = adp - 1;
								while (len) {
									sendbyte(fetchbyte());
									len--;
								}
							}
						} else {
							len = adp - 1;
							while (len) {
								fetchbyte();	// Bytes ignorieren
								len--;
							}
						}
					}
				}
				stop_time();
				fat_closefile();
				send_all_off();
				// repeat modes:
				// 0=single song
				// 1=repeat single song
				// 2=all songs
				// 3=repeat all songs
				// 4=random song
				if (trk_len == 0) {
					if (ee_rep > 1) {
						if (file_num < file_cnt-1) {
							file_num++;
							state = PLAY;
						} else if (ee_rep == 3) {
							file_num = 0;
							state = PLAY;
						}
						if (ee_rep == 4) {
							file_num = random_song(file_num, file_cnt);
							state = PLAY;
						}
					} else if (ee_rep == 1) {
						state = PLAY;
					}
				}
				if (state == PLAY) {
					print_main();
					state = PLAY;
				} else
					print_main();
		} else if (state == DELETE) {
			lcd_string(DISP_DELETE, LINE_DELETE);	// Bildschirm anzeigen
			file_cnt--;
			fat_delete_file();
			if (file_num == file_cnt)
				file_num--;
			delay_ms(1024);
			print_main();
		} else if (state == STOP) {
			gl_lyr = false;
			key_detect();
			file_num = random_song(file_num, file_cnt);	// akashi
			state = PLAY;	// akashi
		} else if (state == MENU) {
			key_detect();
		} else {
			stop_time();
			lcd_string(state-ERR, 1);
			delay_ms(1024);
			print_main();
		}
	}
}

//----------------------------------------------------------
// IRQ

// Timer for delta times.
// Timer/Counter1 Compare Match A
ISR(SIG_OUTPUT_COMPARE1A) {

	// 時刻をインクリメントする。
	time++;

	// DT_FLAG が有効かつ、時刻が、Δt 以上の場合、
	if ((flags & DT_FLAG) && time>=delta_time) {
		// DT_FLAG を無効とする。
		flags &= ~DT_FLAG;		
		// 時刻を初期化する。
		time = 0;
	}

	// 比較レジスタ変更が有効の場合、
	if (flags & OCCHNG_FLAG) {
		// 比較レジスタ変更を無効とする。
		flags &= ~OCCHNG_FLAG;
		// 比較レジスタを変更する。
		OCR1A = new_ocr1a;
	}
}

// Timer for real time clock.
// Timer/Counter2 Compare Match A
ISR(SIG_OUTPUT_COMPARE2A) {
	tickcnt++;
	timeflag |= TIME_KPT;
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
}


/** Timer for debouncing buttons.
*/
ISR(SIG_WATCHDOG_TIMEOUT) {
	wdt_disable();
	flags |= KEY_FLAG;
	if (flags & IR_FLAG && rc5.flip == -1)
		flags &= ~IR_FLAG;
}

/** UART RX.
*/
/** akashi
ISR(SIG_USART_RECV) {
	u08 tmp = UDR;
	if (state != REC || tmp == 0xf8 || tmp == 0xfe)	// Midi Clock und Synch ausblenden
		return;
	rx_buf[rx_wrp++] = tmp;
	rx_wrp &= RX_BUF_SIZE-1;
	if (rx_wrp == rx_rdp) {	// Buffer overflow!
		state = ERR+DISP_BUFERR;
	}
}
*/

