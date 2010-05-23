/*
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;		LCD-Routinen		;;
;;		============		;;
;;		(c)Simon			;;
;;	4-Bit-Interface			;;
;;	DB4-DB7:	PC0-PC3		;;
;;	RS:		PC4				;;
;;	E:		PC5				;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
*/
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include "types.h"
#include "delay.h"


// Port
#define LCD PORTC
#ifdef ADC_TEMPO
#define LCD2 PORTB
#define EN_PIN 0
#else
#define EN_PIN 5
#endif





prog_uchar DISP_TEXT[TEXTCNT][LINESIZE] = {
#ifndef COMPACT16x2
"                        ",
"Mr.MIDI2 by Simon L.    ",
"Ultimate Edition        ",
//"Booting...              ",
"Scanning...    File #   ",
"MMC ERROR: Toggle Power ",
"FILE DATA ERROR: Stopped",
"File:###/###  Size:###kB",
//"Playing...     File #   ",
//"Pause          File #   ",
"Ready for Rec  File #   ",
"Recording...   File #   ",
"INPUT BUFFER RAN FULL!  ",
"Tempo:   Trans:         ",
"NO FILES ON CARD        ",
"Deleting file...        ",
#ifdef CALIBRATE
"Calibrate OSC           ",
"Send MIDI signals to IN!",
"Not neccessary!         ",
"Use manual calibration! ",
#endif
"o=exit,<>=edit,UP/DN=sel",
"1. Drum Channel:        ",
"2. Lyric Display:       ",
#ifdef OLD_CARDSCAN
"x. Start Sector:        ",
#endif
"3. Mute oooooooooooooooo",
"4. Repeat:              ",
#ifdef BACKLIGHT
"x. Backlight:           ",
#endif
#ifdef MIDI_MONITOR
"5. MIDI Monitor:        ",
#endif
#ifdef MANUAL_CALIBRATE
"6. Manual OSCCAL:       ",
#endif
#ifdef CALIBRATE
"7. Calibrate OSC: >>    ",
#endif
#ifdef USE_RTC
"8. Set Time: 00:00:00   ",
"9 .Set Date: 01.01.07   ",
#endif
#else
"                ",
"MrMidi2 by Simon",
"Ultimate Edition",
//"Booting...      ",
"Scanning.. F:   ",
"CARD ERROR      ",
"FILE DATA ERROR ",
"F:###/###  ###kB",
//"Playing... F:   ",
//"Pause      F:   ",
"Ready Rec. F:   ",
"Recording. F:   ",
"INPUTBUFFER FULL",
"S:   T:         ",
"NO FILES ON CARD",
"Deleting file...",
#ifdef CALIBRATE
"Calibrate OSC   ",
"Send MIDI data! ",
"Not neccessary! ",
"Use manual calib",
#endif
"[]=exit, <>=edit",
"1.Drumchannel=  ",
"2.Lyrics:       ",
#ifdef OLD_CARDSCAN
"x.SSector:      ",
#endif
"3.Channelmute:  ",
"4.Repeatmode:   ",
#ifdef BACKLIGHT
"x.Backlight:    ",
#endif
#ifdef MIDI_MONITOR
"5.Monitor:      ",
#endif
#ifdef MANUAL_CALIBRATE
"6.Manual Cal:   ",
#endif
#ifdef CALIBRATE
"7.Calibrate: >> ",
#endif
#ifdef USE_RTC
"8.Time: 00:00:00",
"9.Date: 01.01.07",
#endif
#endif
};



void lcd_enable(void);

// Sendet ein Datenbyte an das LCD
void lcd_data(u08 d) {
/*
	LCD = (d>>4)|0x10;
	lcd_enable();
	LCD = (d&0x0f)|0x10;
	lcd_enable();
	delay_us(50);
*/
}

// Sendet einen Befehl an das LCD
void lcd_command(u08 d) {
/*
	LCD = (d>>4);
	lcd_enable();
	LCD = (d&0x0f);
	lcd_enable();
	delay_us(50);
*/
}

// Erzeugt den Enable-Puls
void lcd_enable(void) {
/*
#ifdef ADC_TEMPO
	LCD2 |= 1 << EN_PIN;
	delay_us(1);
	LCD2 &= ~(1 << EN_PIN);
#else
	LCD |= 1 << EN_PIN;
	delay_us(1);
	LCD &= ~(1 << EN_PIN);
#endif
*/
}

// Sendet den Befehl zur Löschung des Displays
void lcd_clear(void) {
/*
	lcd_command(LCD_CLEAR);
*/
}

// Gibt einen String aus
void lcd_string(u08 idx, u08 row) {
/*
	u08 i;
	lcd_setcur(0, row);
	for (i=0; i<LINESIZE; i++)
		lcd_data((u08)pgm_read_byte( &(DISP_TEXT[idx][i]) ));
*/
}

// Setzt den Cursor
void lcd_setcur(u08 x, u08 y) {
/*
	if (y==1)
		x += 64;
	lcd_command(LCD_DDRAM+x);
*/
}

// Schreibt die Customchars
static void lcd_initcgram(void) {
/*
	u08 i, k;
	for (i=0; i<8; i++) {
		for (k=0; k<8; k++) {
			lcd_command(LCD_CGRAM+k+(i<<3));
			if ((7-k) < i)
				lcd_data(0x1f);
			else
				lcd_data(0x00);
		}
	}
*/
}

// Hexadezimale Ausgabe Nibble
void lcd_nibble(u08 d) {
/*
	if (d > 9)
		lcd_data(d+'A'-10);
	else
		lcd_data(d+'0');
*/
}

// Hexadezimale Ausgabe u08
void lcd_hex_u08(u08 d) {
/*
	lcd_nibble(d>>4);
	lcd_nibble(d&0x0f);
*/
}

// Hexadezimale Ausgabe u16
void lcd_hex_u16(u16 d) {
/*
	lcd_hex_u08((u08)(d>>8));
	lcd_hex_u08((u08)d);
*/
}

/*// Hexadezimale Ausgabe u32
void lcd_hex_u32(u32 d) {
	lcd_hex_u08((u08)(d>>24));
	lcd_hex_u08((u08)(d>>16));
	lcd_hex_u08((u08)(d>>8));
	lcd_hex_u08((u08)d);
}
*/
// Dezimale Zahlen 0..255
void lcd_number(u08 d, u08 n) {
/*	u08 zehner=0, hunderter=0;
	while (d >= 100) {
		d -= 100;
		hunderter++;
	}
	while (d >= 10) {
		d -= 10;
		zehner++;
	}
	if (hunderter || n)
		lcd_data(hunderter+'0');
	lcd_data(zehner+'0');
	lcd_data(d+'0');
*/
}

// Dezimale Zahlen 0..9999
void lcd_number_k(u16 d) {
/*
	u08 zehner=0, hunderter=0, tausender=0;
	while (d >= 1000) {
		d -= 1000;
		tausender++;
	}
	while (d >= 100) {
		d -= 100;
		hunderter++;
	}
	while (d >= 10) {
		d -= 10;
		zehner++;
	}
	lcd_data(tausender+'0');
	lcd_data(hunderter+'0');
	lcd_data(zehner+'0');
	lcd_data(d+'0');
*/
}

// Initialisierung: Muss ganz am Anfang des Programms aufgerufen werden
void lcd_init(void) {
/*
	delay_ms(250);
	LCD = 0x03;
	lcd_enable();
	delay_ms(5);
	lcd_enable();
	delay_ms(5);
	lcd_enable();
	delay_ms(5);
	LCD = 0x02;
	lcd_enable();
	delay_ms(5);
	lcd_command(0x28);
	lcd_command(0x0c);
	lcd_command(0x04);
	lcd_command(LCD_CLEAR);
	delay_ms(5);
	lcd_initcgram();
	delay_ms(1);
*/
}
