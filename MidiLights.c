#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 10000000UL
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

#include "lcd.h"
#include "mmc.h"
#include "spi.h"
#include "delay.h"
#include "fat16.h"
#include "types.h"
#include "rc5.h"

void rc5_TIMER0_OVF_vect();
void rc5_INT0_vect();
void rc5_init2();

void main2(void);
void rc5_exit();

extern u16 file_cnt;

uint16_t	gLEDs[8];
uint16_t    gPianoKeys[8];
bool gIsPracticeMode;		// 練習モード

u08 gKey = 0x00;
uint8_t gRc5checking = 0;

// 外部割り込み
ISR(INT1_vect)
{
	gKey = 0x01;

/*	static u32 data = 0;
	static u08 nbit = 0;
	static u16 cmd = 0;

	// LEDの電源スイッチ
	DDRA = 0xFF;
	PORTA = 0;

	TIMSK0 &= ~1;		// タイマ０オーバーフロー割り込み禁止

	// リモコン測定中
	if (gRc5checking) {
		nbit++;
		data <<= 1;

		// タイマカウンタの確認
		if (TCNT0 > 13) {
			data |= 0x0001;
		}

		// 
		if (nbit == 16) {
			cmd = (u16)(0x0000ffff & data);
		}

		// 
		if (nbit == 50) {
			// 正しいコマンドの場合、
			//if (cmd == 0x4004) {
				gData[0] = 0xff & (data >> 24);
				gData[1] = 0xff & (data >> 16);
				gData[2] = 0xff & (data >> 8);
				gData[3] = 0xff & data;
			//} else {
			//	gData[0] = gData[1] = gData[2] = gData[3] = 0xff;
			//}

			// リモコン測定完了
			nbit = 0;
			gRc5checking = 0;
			TCCR0B = 0x01;	// プリスケーラは、1
		}
	} else {
		nbit = 0;
		TCCR0B = 0x05;		// プリスケーラは、1024
		gRc5checking = 1;	// リモコン測定中とする。
	}
	

	TCNT0 = 0;	// タイマーカウント０
	TIFR0 = 1;	// タイマ/カウンタ０オーバーフロー割り込み結果をクリア
	TIMSK0 = 1;	// タイマ/カウンタ０オーバーフロー割り込み許可
*/
}

// タイマー通知
ISR(TIMER0_OVF_vect)
{
/*	TIMSK0 &= ~1;		// タイマ０オーバーフロー割り込み禁止

	// リモコン測定中
	if (gRc5checking) {
		// リモコン測定完了
		gRc5checking = 0;
		TCCR0B = 0x01;	// プリスケーラは、1
	} else {
*/
		static uint8_t	sLedPowerBit = 0;

		PORTA = 0;	// LEDの電源OFF

		PORTC = 0xFF & ~(gLEDs[sLedPowerBit]);
		PORTD &= 0x1F;
		PORTD |= 0xE0 & (~(gLEDs[sLedPowerBit] >> 3));

		//gData[2] = rc5.code;
		//gData[3] = rc5.addr;

		PORTA = 1 << sLedPowerBit;	// 指定のLEDの電源ON

		sLedPowerBit++;
		if (sLedPowerBit > 7) {
			sLedPowerBit = 0;
		}
/*	}

	TIMSK0 = 1;		// タイマ/カウンタ０オーバーフロー割り込み許可
*/
}

// USART0, Rx Complete 
ISR(USART0_RX_vect)
{
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
		// ノートナンバー21～108と割り当てられる

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
			gLEDs[idx] |= data;
			break;
		case 0x80:	// ノートオフ
			gLEDs[idx] &= ~data;
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
				gLEDs[i] = 0;
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

// USART1, Rx Complete
ISR(USART1_RX_vect)
{
	static uint8_t	operand = 0;
	static uint8_t  operand_bak = 0;
	static int8_t	note = -1;
	static uint8_t	ignore_count = 0;

	if (ignore_count) {
		ignore_count--;
		return;
	}

	uint8_t d = UDR1;

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
		// ノートナンバー21～108と割り当てられる

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
			gPianoKeys[idx] |= data;
			break;
			case 0x80:	// ノートオフ
			gPianoKeys[idx] &= ~data;
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
				gPianoKeys[i] = 0;
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

// USART0, Tx Complete 
ISR(USART0_TX_vect)
{
}

// USART0 Data Register Empty 
ISR(USART0_UDRE_vect)
{
}

// USART1, Tx Complete
ISR(USART1_TX_vect)
{
}

// USART1 Data Register Empty
ISR(USART1_UDRE_vect)
{
}

void error(uint8_t err) {
	sei();
	for (u08 i = 0; i < 5; i++) {
		gLEDs[0] = err;
		delay_ms(500);
		gLEDs[0] = 0xff;
		delay_ms(100);
		gLEDs[0] = 0;
		delay_ms(100);
	}	
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

int main(void)
{
	cli();

	// LEDの電源スイッチ
	DDRA = 0xFF;
	PORTA = 0;

	// 未接続(未使用)ピンは、ノイズ耐性向上のため'0'出力(GND接続状態)とします。
	// MMCのため、SS(PB4),MOSI(PB5),MISO(PB6),SCK(PB7)を0
	DDRB = 0xBF;
	PORTB = 0xF0;

	// LEDの制御スイッチ('0'出力で点灯、'1'出力で消灯であり、最初は消灯させるので'1'出力とします。)
	DDRC = 0xFF;
	PORTC = 0xFF;

	// D0は、MIDI入力。D1は、MIDI出力。D3は、赤外線リモコン受信用外部割り込み
	// D5-D7は、LEDの制御スイッチ
	DDRD = 0xF2;
	PORTD = 0xEF;

	// タイマ設定
	TCCR0B = 0x01;	// プリスケーラは、1
	TIMSK0 = 0x01;	// タイマ２オーバーフロー割り込み許可
	
	UBRR0 = 19;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 10MHz
	UBRR1 = 19;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 10MHz
//	UBRR0 = 15;		// MIDIのボーレートは、31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 8MHz
//	UCSR0B = 0xB8;	// 送受信および受信完了送信空き割り込み許可
	UCSR0B = 0x98;	// 送受信および受信完了割り込み許可
	UCSR1B = 0x90;	// 受信および受信完了割り込み許可

	MMC_hw_init();
	spi_init();
//	rc5_init(RC5_ALL);

	gIsPracticeMode = false;

	sei();
	for (int8_t i = 0; i < 88; i++) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gLEDs[idx] |= data;
		_delay_ms(3);
	}

	for (int8_t i = 0; i < 88; i++) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gLEDs[idx] &= ~data;
		_delay_ms(3);
	}

	for (int8_t i = 87; i >= 0; i--) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gLEDs[idx] |= data;
		_delay_ms(3);
	}

	for (int8_t i = 87; i >= 0; i--) {
		uint8_t idx = i / 11;
		uint16_t data = 1 << (i % 11);
		gLEDs[idx] &= ~data;
		_delay_ms(3);
	}
	// ピン変化割り込み(赤外線リモコン受信許可)
	EICRA = 0x08;	// 外部割り込み１ INT1 が HIGH → LOW
	EIFR = 0x02;	// 割り込みペンディングをクリア
	EIMSK |= 2;		// 外部割り込み１許可

	main2();

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
