#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 10000000UL
#include <util/delay.h>
#include <stdint.h>
//#include <stdlib.h>

#include "lcd.h"
#include "mmc.h"
#include "spi.h"
#include "fat16.h"
#include "rc5.h"

uint16_t	gData[8];

// タイマー通知
ISR(TIMER2_OVF_vect)
{
	static uint8_t	sLedPowerBit = 0;

	PORTA = 0;	// LEDの電源OFF

//	PORTD = 0x03 | ~(gData[sLedPowerBit] << 1);
	PORTD &= 0x03;
	PORTD |= 0xFC & (~(gData[sLedPowerBit] << 1));
	PORTB &= 0xFD;
	PORTB |= 0x02 & ~(gData[sLedPowerBit] << 1);
	PORTC = (0x03 & ~(gData[sLedPowerBit] >> 7)) | (0xC0 & ~(gData[sLedPowerBit] >> 3));

	PORTA = 1 << sLedPowerBit;	// 指定のLEDの電源ON

	sLedPowerBit++;
	if (sLedPowerBit > 7) {
		sLedPowerBit = 0;
	}
}

// USART0, Rx Complete 
ISR(USART0_RX_vect)
{
	//gData[1] = 0x1;
	static uint8_t	operand = 0;
	static uint8_t  operand_bak = 0;
	static int8_t	note = -1;
	static uint8_t	ignore_count = 0;

	if (ignore_count) {
		ignore_count--;
		return;
	}

	uint8_t d = UDR0;

retry:
	switch (operand) {
	case 0:	// 未定
		switch (d & 0xF0) {
		case 0x90:	// ノートオン
		case 0x80:	// ノートオフ
			note = -1;
			// break しない
		case 0xB0:	// コントロールチェンジ or モード・チェンジ
		case 0xC0:	// プログラムチェンジ
			operand_bak = operand = 0xF0 & d;
			break;
		case 0xF0:	// システム・リアルタイム・メッセージ
			switch (d) {
			case 0xF0:
				operand = d;
				break;
			}
			break;

		default:	// ランニングステータス
			switch (operand_bak) {
			case 0x90:
			case 0x80:
				if ((d & 0x80) == 0x00) {
					operand = operand_bak;
					goto retry;
				}
				break;
			}
		}
		break;
	case 0x90:	// ノートオン
	case 0x80:	// ノートオフ
		// データバイトにて指定するノートナンバーとは、
		// 最も低い音を0、最も高い音を127と割り当てた音の高さのことである。
		// 中央ハにはノートナンバー60が割り当てられ、
		// 88鍵盤のグランドピアノで出せる音域は
		// ノートナンバー21〜108と割り当てられる

		if (note == -1) {
			if (d < 21 || 108 < d) {
				ignore_count = 1;
				operand = 0;
				break;
			}
			note = d - 21;
			break;
		}

		if (d == 0 || d > 127) {
			operand = 0x80;
		}

		uint8_t idx = note / 11;
		uint16_t data = 1 << (note % 11);
		switch (operand) {
		case 0x90:	// ノートオン
			gData[idx] |= data;
			break;
		case 0x80:	// ノートオフ
			gData[idx] &= ~data;
			break;
		}
		note = -1;
		operand = 0;
		break;
	case 0xB0:	// コントロールチェンジ or モード・チェンジ
		// オール・ノート・オフ
		switch (d) {
		case 0x78:
		case 0x79:
		case 0x7B:
		case 0x7C:
		case 0x7D:
		case 0x7E:
		case 0x7F:
			for (int i = 0; i < 8; i++) {
				gData[i] = 0;
			}
			break;
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

volatile int gFlag;

// USART0, Tx Complete 
ISR(USART0_TX_vect)
{
	gData[0] = 0x1;
	gFlag = 0;
}

// USART0 Data Register Empty 
ISR(USART0_UDRE_vect)
{
}

// Pin Change Interrupt Request 1 
ISR(PCINT1_vect)
{
	gData[0]++;
}

void send(uint8_t data)
{
	// 送信完了していない限り、繰り返す。
//	while ((UCSR0A & (1 << TXC0)) == 0x00);

	// 送信データ・レジスタが空きでない限り、繰り返す。
	while ((UCSR0A & (1 << UDRE0)) == 0x00);

	// 送信データをセット。
	UDR0 = data;
//	recv(data);
}

void note_on(uint8_t note)
{
	send(0x90);
	send(note + 21);
	send(100);
}

void note_off()
{
	send(0xB0);
	send(0x7B);
}

void error(uint8_t err) {
	for (;;) {
		gData[0] = err;
		_delay_ms(100);
		gData[0] = 0;
		_delay_ms(100);
	}	
}

int main(void)
{
	cli();

	// LEDの電源スイッチ
	DDRA = 0xFF;
	PORTA = 0;

	// 未接続(未使用)ピンは、ノイズ耐性向上のため'0'出力(GND接続状態)とします。
	// MMCのため、SS(PB4),MOSI(PB5),MISO(PB6),SCK(PB7)を0
	DDRB = 0xBE;
	PORTB = 0x03;

	// LEDの制御スイッチ('0'出力で点灯、'1'出力で消灯であり、最初は消灯させるので'1'出力とします。)
	DDRC = 0xFF;
	PORTC = 0xC3;

	// D0は、MIDI入力
	// D1-D7は、LEDの制御スイッチ
	DDRD = 0xFE;
	PORTD = 0xFF;

	// タイマ設定
	TCCR2B = 0x01;	// プリスケーラは、1
	TIMSK2 = 0x01;	// タイマ２オーバーフロー割り込み許可
	
	UBRR0 = 19;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
//	UCSR0B = 0xB8;	// 送受信および受信完了送信空き割り込み許可
	UCSR0B = 0x98;	// 送受信および受信完了割り込み許可

	// MMC/SDカード用の初期化処理
//	MMC_hw_init();
//	spi_init();
	//rc5_init(RC5_ALL);

	sei();
/*
	if (MMC_init()) {
		lcd_string(DISP_OK, LINE_OK);
		flags |= MMCOK_FLAG;
	} else {
		// mmc error
	}

	if (flags & MMCOK_FLAG) {
		if (!fat_init()) {
			error(1);
		}
		fat_count_files();
		fat_read_filedata(file_num);
	}
*/
	for (int8_t i = 0; i < 88; i++) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gData[idx] |= data;
		_delay_ms(3);
	}

	for (int8_t i = 0; i < 88; i++) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gData[idx] &= ~data;
		_delay_ms(3);
	}

	for (int8_t i = 87; i >= 0; i--) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gData[idx] |= data;
		_delay_ms(3);
	}

	for (int8_t i = 87; i >= 0; i--) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gData[idx] &= ~data;
		_delay_ms(3);
	}

	// ピン変化割り込み(赤外線リモコン受信許可)
	PCICR = 1 << PCIE1;		// ポートB
	PCMSK1 = 1 << PCINT8;	

	for (;;) {
/*
		for (int8_t i = 0; i < 88; i++) {
			note_on(i);
			_delay_ms(10);
			note_off();
		}

		for (int8_t i = 87; i >= 0; i--) {
			uint8_t idx = i / 11;
			uint16_t data = 1 << (i % 11);
			gData[idx] |= data;
			_delay_ms(3);
		}

		for (int8_t i = 87; i >= 0; i--) {
			uint8_t idx = i / 11;
			uint16_t data = 1 << (i % 11);
			gData[idx] &= ~data;
			_delay_ms(3);
		}
*/
		_delay_ms(5000);
	}
}
