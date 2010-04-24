#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

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

	for (;;) {

		for (int i = 0; i < 8; i++) {
			PORTA = 1 << i;

			for (int j = 0; j < 3; j++ ) {
				PORTD = 0xE1 & ~(1 << (j + 5));
				_delay_ms(50);
			}
			PORTD = 0xE1;

			for (int j = 0; j < 8; j++ ) {
				PORTC = ~(1 << j);
				_delay_ms(50);
			}
			PORTC = 0xFF;

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
}
