/*
 * a.lp_mp3 - Open Source Atmel AVR based MP3 Player
 * Copyright (c) 2003-2004 K. John '2B|!2B' Crispin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA02111-1307USA
 *
 * Feedback, Bugs.... mail john{AT}phrozen.org
 *
 */ 
 
#include <avr/io.h>
#include <avr/interrupt.h>

#include "types.h"

void spi_init(void){
	// setup spi intrface
/*
#ifdef __AVR_ATmega8__
	sbi(DDRB,PB2);
	sbi(DDRB,PB3);
	cbi(PORTB,PB5);
	sbi(DDRB,PB5);
#endif
#ifdef __AVR_ATmega168__
	sbi(DDRB,PB2);
	sbi(DDRB,PB3);
	cbi(PORTB,PB5);
	sbi(DDRB,PB5);
#endif

#ifdef __AVR_ATmega32__
	sbi(DDRB, PB5);	// set MOSI a output
	sbi(DDRB, PB4);	// SS must be output for Master mode to work
	sbi(DDRB, PB7);	// set SCK as output
	cbi(PORTB, PB7);// set SCK lo
#endif
*/
	sbi(DDRB, PB5);	// set MOSI a output
	sbi(DDRB, PB4);	// SS must be output for Master mode to work
	sbi(DDRB, PB7);	// set SCK as output
	cbi(PORTB, PB7);// set SCK lo

	outp(((1<<MSTR)|(1<<SPE)|(1<<SPR1)), SPCR );	// enable SPI interface (8MHz/64=125kHz)
	// MSTR(マスタ/スレーブ選択) '0':スレーブ,'1':マスタ
	// SPE(SPIイネーブル)
	// SPR1,SPR0 クロック・レート選択
	SPSR |= 1;	// SPI2X -> 250kHz SPI-Clock
	// 10MHzなので、SPIのクロックレートは、312.5kHzとなる。
};

u08 spi_io(u08 data){
	outp(data, SPDR);
	while((inp(SPSR)&(1<<SPIF)) == 0x00){};
	return inp(SPDR);
};
	
	
