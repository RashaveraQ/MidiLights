#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 10000000UL
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

uint16_t	gData[8];
uint8_t		gLedPowerBit = 0;

ISR(TIMER0_OVF_vect)
{
	PORTA = 0;	// LEDの電源OFF

	PORTC = ~(0xFF & gData[gLedPowerBit]);
	PORTD = 0x01 | (0xE0 & ~(0xE0 & (gData[gLedPowerBit] >> 3)));

	gLedPowerBit++;
	if (gLedPowerBit > 7) {
		gLedPowerBit = 0;
	}

	PORTA = 1 << gLedPowerBit;	// 指定のLEDの電源ON
}

ISR(USART0_RX_vect)
{
	uint8_t d = UDR0;
	gData[d % 8] |= 1 << ((d / 8) % 11);
}

int main(void)
{
	// LEDの電源スイッチ
	DDRA = 0xFF;
	PORTA = 0;

	// 未接続(未使用)ピンは、ノイズ耐性向上のため'0'出力(GND接続状態)とします。
	DDRB = 0xFF;
	PORTB = 0;

	// LEDの制御スイッチ('0'出力で点灯、'1'出力で消灯であり、最初は消灯させるので'1'出力とします。)
	DDRC = 0xFF;
	PORTC = 0xFF;

	// D0は、MIDI入力
	// D1-D4は、未接続
	// D5-D7は、LEDの制御スイッチ
	DDRD = 0xFE;
	PORTD = 0xE1;

	// タイマ設定
	TCCR0B = 0x03;	// プリスケーラは、64
	//TCCR0B = 0x04;	// プリスケーラは、256
	//TCCR0B = 0x05;	// プリスケーラは、1024
	TIMSK0 = 0x01;	// タイマ０オーバーフロー割り込み許可

	UBRR0 = 19;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
	UCSR0B = 0x90;	// 受信および受信完了割り込み許可

	sei();
/*
	for (int i = 0; i < 8; i++) {
		gData[i] = 1 << 10;
	}

	for (;;) {
		_delay_ms(10);
	}

*/
	for (;;) {
		for (int i = 0; i < 8; i++) {
			gData[i] = 0;
		}
/*		for (int i = 0; i < 10; i++) {
			long r = random();
			gData[r % 8] |= 1 << ((r / 8)% 11);
		}
*/
		_delay_ms(1000);
	}


/*
	PORTA = 4;
	for (;;) {
		PORTC = 0x7F;
		_delay_ms(1);
		PORTC = 0xFF;
		_delay_ms(11);
	}
*/
/*	for (;;) {
		for (int i = 0; i < 8; i++) {
			PORTA = 1 << i;

			for (int j = 1; j < 3; j++ ) {
				PORTD = 0xE1 & ~(1 << (j + 5));
				_delay_ms(50);
			}
			PORTD = 0xE1;

			for (int j = 0; j < 8; j++ ) {
				PORTC = ~(1 << j);
				_delay_ms(50);
			}
			PORTC = 0xFF;

			for (int j = 0; j < 1; j++ ) {
				PORTD = 0xE1 & ~(1 << (j + 5));
				_delay_ms(50);
			}
			PORTD = 0xE1;
		}

		PORTA = 0;
		PORTC = 0x00;
		PORTD = 0x01;
		for (int l = 0; l < 100; l++) {
			for (int i = 0; i < 8; i++) {
				PORTA = 1 << i;
				_delay_ms(1);
			}
		}
		PORTA = 0;
		PORTC = 0xFF;
		PORTD = 0xE1;
	}
*/
}
