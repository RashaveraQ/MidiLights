#include <avr/io.h>
#include "spi.h"
#include "types.h"

// RTC-Typen
#define RTC4513
//#define DUMMY

// Port
#define CLOCK_PORT		PORTB
#define CLOCK_CS_PIN	0x40

#ifdef RTC4513
	#define CLOCK_PIN		PINB
	#define CLOCK_DDR		DDRB
	#define CLOCK_DATA_PIN	0x10
	#define CLOCK_SCK_PIN	0x20
#endif

// Register
#ifdef RTC4513
#define MODE_WRITE	0x03
#define MODE_READ	0x0c

enum {
	REG_S1,
	REG_S10,
	REG_MI1,
	REG_MI10,
	REG_H1,
	REG_H10,
	REG_D1,
	REG_D10,
	REG_MO1,
	REG_MO10,
	REG_Y1,
	REG_Y10,
	REG_W,
	REG_CD,
	REG_CE,
	REG_CF
};
#endif

#ifdef DUMMY
#define CMD_CLOCKSTART	0x10
#define CMD_CLOCKHALT	0x11
#define CMD_CLOCK24H	0x18
#define CMD_CLOCKREAD	0x20
#define CMD_CLOCKSET	0x21
#endif

// Vars
struct time_st {
	u08 gl_sek, gl_min, gl_std, gl_tag, gl_mon, gl_jahr;
} gl_time;

	#define gl_sek gl_time.gl_sek
	#define gl_min gl_time.gl_min
	#define gl_std gl_time.gl_std
	#define gl_tag gl_time.gl_tag
	#define gl_mon gl_time.gl_mon
	#define gl_jahr gl_time.gl_jahr

const u08 TAGE_IM_MONAT[12] = {
	31,29,31,30,31,30,31,31,30,31,30,31
};

// Funktionen
inline static void clock_cs_low(void) {
	CLOCK_PORT &= ~CLOCK_CS_PIN;
}


inline static void clock_cs_high(void) {
#ifdef RTC4513
	CLOCK_PORT &= ~(CLOCK_DATA_PIN | CLOCK_SCK_PIN);
#endif
	CLOCK_PORT |= CLOCK_CS_PIN;
}


/*
inline static u08 clock_calcwtag(void) {
	u08 mon=gl_mon, jahr=gl_jahr, jh=20;
	if (mon < 3) {
		jahr--;
		mon += 12;
	}
	if (jahr >= 100) {
		jh = 21;
		jahr -= 100;
	}
	mon = gl_tag + (u08)(mon+1)*(u08)13/(u08)5 + jahr + (jahr>>2) + (jh>>2) + 5;
	jh <<= 1;
	while (jh > mon)
		mon += 7;
	mon -= jh;
	while (mon > 6)
		mon -= 7;
	return mon;
}
*/

#ifdef RTC4513
static void write_nibble(u08 b) {
	u08 i;
	CLOCK_DDR |= CLOCK_DATA_PIN;
	for (i=0; i<4; i++) {
		if (b&1)
			CLOCK_PORT |= CLOCK_DATA_PIN;
		else
			CLOCK_PORT &= ~CLOCK_DATA_PIN;
		b >>= 1;
		NOP;
		CLOCK_PORT |= CLOCK_SCK_PIN;
		NOP; NOP; NOP;
		CLOCK_PORT &= ~CLOCK_SCK_PIN;
		NOP;
	}
}

static u08 read_nibble(void) {
	u08 i, temp=0;
	CLOCK_DDR &= ~CLOCK_DATA_PIN;
	for (i=0x01; i<0x10; i<<=1) {
		CLOCK_PORT |= CLOCK_SCK_PIN;
		NOP; NOP;
		if (CLOCK_PIN & CLOCK_DATA_PIN)
			temp |= i;
		NOP;
		CLOCK_PORT &= ~CLOCK_SCK_PIN;
		NOP;
	}
	return temp;
}

static void write_data(u08 adr, u08 b) {
	clock_cs_high();
	write_nibble(MODE_WRITE);
	write_nibble(adr);
	write_nibble(b);
}

static u08 read_data(u08 adr) {
	clock_cs_high();
	write_nibble(MODE_READ);
	write_nibble(adr);
	return read_nibble();
}

static u08 div_u08(u08 *z, u08 d) {
	u08 e=0;
	while (*z > d) {
		*z -= d;
		e++;
	}
	return e;
}
#endif
	

void read_time(void)
{
#ifdef RTC4513
	u08 temp, i, dia;
	do {
		dia = 0;
		temp = read_data(REG_S1);
		i = read_nibble();
		temp += 10*(i&0x07);
		gl_sek = temp;
		temp = read_nibble();
		i = read_nibble();
		if (i&0x08) dia = 1;
		temp += 10*(i&0x07);
		gl_min = temp;
		temp = read_nibble();
		i = read_nibble();
		if (i&0x08) dia = 1;
		temp += 10*(i&0x03);
		gl_std = temp;
		temp = read_nibble();
		i = read_nibble();
		if (i&0x08) dia = 1;
		temp += 10*(i&0x03);
		gl_tag = temp;
		temp = read_nibble();
		i = read_nibble();
		if (i&0x08) dia = 1;
		temp += 10*(i&0x01);
		gl_mon = temp;
		temp = read_nibble();
		temp += 10*read_nibble();
		gl_jahr = temp;
		clock_cs_low();
	}
	while (dia);
#endif
	
#ifdef DUMMY
	u08 temp=0xff, i, *p;
	clock_cs_low();
	spi_io(CMD_CLOCKREAD);
	p = (u08 *)gl_time.gl_sek;
	for (i=0; i<sizeof(gl_time); i++)
		*p++ = spi_io(temp);
	clock_cs_high();
#endif
}


void set_time(void)
{
#ifdef RTC4513
	u08 temp, i;
	write_data(REG_CF, 0x07); // Reset
	clock_cs_low();
	write_data(REG_W, 0); // Init
	write_nibble(0x02);
	write_nibble(0x0f);
	write_nibble(0x07);
	temp = gl_sek;
	i = div_u08(&temp, 10);
	write_nibble(temp);
	write_nibble(i);
	temp = gl_min;
	i = div_u08(&temp, 10);
	write_nibble(temp);
	write_nibble(i);
	temp = gl_std;
	i = div_u08(&temp, 10);
	write_nibble(temp);
	write_nibble(i);
	clock_cs_low();
	write_data(REG_CF, 0x04);
	clock_cs_low();
	write_data(REG_CD, 0x02);
	clock_cs_low();
#endif
	
#ifdef DUMMY
	u08 i, *p;
	clock_cs_low();
	spi_io(CMD_CLOCKHALT);
	clock_cs_high();
	clock_cs_low();
	spi_io(CMD_CLOCK24H);
	clock_cs_high();
	clock_cs_low();
	spi_io(CMD_CLOCKSET);
	p = (u08 *)gl_time.gl_sek;
	for (i=0; i<sizeof(gl_time); i++)
		spi_io(*p++);
//	spi_io(clock_calcwtag());
	spi_io(0);
	clock_cs_high();
	clock_cs_low();
	spi_io(CMD_CLOCKSTART);
	clock_cs_high();
#endif
}

void set_date(void)
{
#ifdef RTC4513
	u08 temp, i;
	temp = gl_tag;
	i = div_u08(&temp, 10);
	write_data(REG_D1, temp);
	write_nibble(i);
	temp = gl_mon;
	i = div_u08(&temp, 10);
	write_nibble(temp);
	write_nibble(i);
	temp = gl_jahr;
	i = div_u08(&temp, 10);
	write_nibble(temp);
	write_nibble(i);
	clock_cs_low();
#endif
}
