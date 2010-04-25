#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 10000000UL
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

uint16_t	gData[8];

ISR(TIMER0_OVF_vect)
{
	static uint8_t	sLedPowerBit = 0;

	PORTA = 0;	// LEDの電源OFF

	PORTC = ~(0xFF & gData[sLedPowerBit]);
	PORTD = 0x01 | (0xE0 & ~(0xE0 & (gData[sLedPowerBit] >> 3)));

	sLedPowerBit++;
	if (sLedPowerBit > 7) {
		sLedPowerBit = 0;
	}

	PORTA = 1 << sLedPowerBit;	// 指定のLEDの電源ON
}

ISR(USART0_RX_vect)
{
	static uint8_t	operand = 0;
	static uint8_t	ignore_count = 0;

	uint8_t d = UDR0;

	if (ignore_count) {
		ignore_count--;
		return;
	}

	switch (operand) {
	case 0:	// 未定
		switch (d & 0xF0) {
		case 0x90:	// ノートオン
		case 0x80:	// ノートオフ
			//gData[4] = 0xFFFF;
		case 0xB0:	// コントロールチェンジ or モード・チェンジ
		case 0xC0:	// プログラムチェンジ
			operand = 0xF0 & d;
			break;
		case 0xF0:	// システム・リアルタイム・メッセージ
			if (d == 0xF0) {
				operand = d;
			}
			break;
		}
		break;
	case 0x90:	// ノートオン
	case 0x80:	// ノートオフ
		// データバイトにて指定するノートナンバーとは、
		// 最も低い音を0、最も高い音を127と割り当てた音の高さのことである。
		// 中央ハにはノートナンバー60が割り当てられ、
		// 88鍵盤のグランドピアノで出せる音域は
		// ノートナンバー21～108と割り当てられる
		d -= 21;	// LEDの配線を修正後用
		//d -= 18;	// 暫定LEDの配線を直す前


		uint8_t idx = d / 11;
		uint8_t data = 1 << (d % 11);
		switch (operand) {
		case 0x90:	// ノートオン
			gData[idx] |= data;
			break;
		case 0x80:	// ノートオフ
			gData[idx] &= ~data;
			break;
		}
		ignore_count = 1;
		operand = 0;
		break;
	case 0xB0:	// コントロールチェンジ or モード・チェンジ
		// オール・ノート・オフ
		if (d == 0x7B) {
			for (int i = 0; i < 8; i++) {
				gData[i] = 0;
			}
		}
		// break しない
	case 0xC0:	// プログラムチェンジ
		ignore_count = 1;
		operand = 0;
		break;
	case 0xF0:	// システム・リアルタイム・メッセージ
		// 終了メッセージ
		if (d == 0xF7) {
			operand = 0;
		}
		break;
	}
}

int main(void)
{
	cli();

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
	TCCR0B = 0x01;	// プリスケーラは、1
	TIMSK0 = 0x01;	// タイマ０オーバーフロー割り込み許可

	UBRR0 = 19;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
	UCSR0B = 0x90;	// 受信および受信完了割り込み許可

	sei();

	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < 88; i++) {
			uint8_t idx = i / 11;
			uint8_t data = 1 << (i % 11);
			gData[idx] |= data;
			_delay_ms(4);
		}
		for (int i = 0; i < 88; i++) {
			uint8_t idx = i / 11;
			uint8_t data = 1 << (i % 11);
			gData[idx] &= ~data;
			_delay_ms(4);
		}
	}

	for (;;) {
		for (int i = 0; i < 8; i++) {
			gData[i] = 0;
		}
		_delay_ms(1000);
	}
}
