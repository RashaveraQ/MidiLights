#ifndef _LCD_H_
#define _LCD_H_

#include "types.h"
#include "mrmidi2.h"

// Text
enum display_strings_e{
	DISP_BLANK,
	DISP_MAIN,
	DISP_MIDICTR,
//	DISP_BOOT,
	DISP_OK,
	DISP_CARDERR,
	DISP_FILEERR,
	DISP_SELECT,
//	DISP_PLAY,
//	DISP_PAUSE,
	DISP_RECRD,
	DISP_REC,
	DISP_BUFERR,
	DISP_TEMPO,
	DISP_NOFILE,
	DISP_DELETE,
#ifdef CALIBRATE
	DISP_CAL1,
	DISP_CAL2,
	DISP_CAL3,
	DISP_CAL4,
#endif
	DISP_MENU,
	DISP_MDRUM,
	DISP_MLYR,
#ifdef OLD_CARDSCAN
	DISP_MSSECT,
#endif
	DISP_MMUTE,
	DISP_MREP,
#ifdef BACKLIGHT
	DISP_MLIGHT,
#endif
#ifdef MIDI_MONITOR
	DISP_MIDISIG,
#endif
#ifdef MANUAL_CALIBRATE
	DISP_MMANCAL,
#endif
#ifdef CALIBRATE
	DISP_MCAL,
#endif
#ifdef USE_RTC
	DISP_SETCLOCK,
	DISP_SETDATE,
#endif
	MAX_DISP_ELEM
};

#define TEXTCNT	MAX_DISP_ELEM	//27

//#define COMPACT16x2

#ifdef COMPACT16x2
	#define LINESIZE	16
	#define MAX_LINES	2

	#define LINE_MAIN 0
	#define LINE_BOOT 1
	#define LINE_OK 1
	#define LINE_CARDERR 1
	#define LINE_FILEERR 1
	#define TEMPO_X 2
	#define TEMPO_Y 1
	#define LINE_PLAY 0
	#define LINE_TEMPO 1
	#define PLAYFNUM_X 13
	#define PLAYFNUM_Y 0
	#define DWAHL_X 12
	#define DWAHL_Y 0
	#define STOPFNUM_X 2
	#define STOPFNUM_Y 1
	#define STOPSECT_X 12
	#define STOPSECT_Y 1
	#define LINE_PAUSE 0
	#define LINE_MAIN 0
	#define LINE_SELECT 1
	#define NAME_LINE 0
	#define FCNT_X 6
	#define FCNT_Y 1
	#define FSIZE_X 11
	#define FSIZE_Y 1
	#define LINE_NOFILE 1
	#define LINE_MENU 1
	#define LINE_OPTION 0
	#define DRUMCH_X 14
	#define LYR_X 9
	#define SSECT_X 10
	#define REP_X 13
	#define LIGHT_X 12
	#define MANCAL_X 13
    #define MIDIMON_X 13
	#define CLOCK_X 8
	#define DATE_X 8
	#define TRANS_X 7
	#define TRANS_Y 1
	#define MUTE_X 0
	#define MUTE_Y 1
	#define TIME_X 11
	#define TIME_Y 1
	#define LINE_RECRD 0
	#define RECFNUM_X 13
	#define RECFNUM_Y 0
	#define LINE_REC_BLANK 1
	#define LINE_REC 0
	#define LINE_DELETE 1
	#define LINE_BUFERR 1
	#define BOOTFNUM_X 13
	#define BOOTFNUM_Y 1
	#define BOOTSECT_X 12
	#define BOOTSECT_Y 0
#else
	#define LINESIZE	24
	#define MAX_LINES	2

	#define LINE_MAIN 0
	#define LINE_BOOT 1
	#define LINE_OK 1
	#define LINE_CARDERR 1
	#define LINE_FILEERR 1
	#define TEMPO_X 6
	#define TEMPO_Y 1
	#define LINE_PLAY 0
	#define LINE_TEMPO 1
	#define PLAYFNUM_X 21
	#define PLAYFNUM_Y 0
	#define DWAHL_X 20
	#define DWAHL_Y 0
	#define STOPFNUM_X 5
	#define STOPFNUM_Y 1
	#define STOPSECT_X 14
	#define STOPSECT_Y 1
	#define LINE_PAUSE 0
	#define LINE_MAIN 0
	#define LINE_SELECT 1
	#define NAME_LINE 0
	#define FCNT_X 9
	#define FCNT_Y 1
	#define FSIZE_X 19
	#define FSIZE_Y 1
	#define LINE_NOFILE 1
	#define LINE_MENU 1
	#define LINE_OPTION 0
	#define DRUMCH_X 17
	#define LYR_X 18
	#define SSECT_X 17
	#define REP_X 11
	#define LIGHT_X 14
	#define MANCAL_X 19
    #define MIDIMON_X 17
	#define CLOCK_X 15
	#define DATE_X 15
	#define TRANS_X 15
	#define TRANS_Y 1
	#define MUTE_X 8
	#define MUTE_Y 0
	#define TIME_X 19
	#define TIME_Y 1
	#define LINE_RECRD 0
	#define RECFNUM_X 21
	#define RECFNUM_Y 0
	#define LINE_REC_BLANK 1
	#define LINE_REC 0
	#define LINE_DELETE 1
	#define LINE_BUFERR 1
	#define BOOTFNUM_X 21
	#define BOOTFNUM_Y 1
	#define BOOTSECT_X 20
	#define BOOTSECT_Y 0
#endif

#ifndef LED_DISP
// Commands
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_MODE 0x04
#define LCD_ON 0x08
#define LCD_SHIFT 0x10
#define LCD_FUNCSET 0x20
#define LCD_CGRAM 0x40
#define LCD_DDRAM 0x80

// Functions
void lcd_data(u08 d);
void lcd_command(u08 c);
void lcd_clear(void);
void lcd_string(u08 idx, u08 row);
void lcd_setcur(u08 x, u08 y);
void lcd_init(void);
void lcd_nibble(u08 d);
void lcd_hex_u08(u08 d);
void lcd_hex_u16(u16 d);
//void lcd_hex_u32(u32 d);
void lcd_number(u08 d, u08 n);
void lcd_number_k(u16 d);
#endif

#endif
