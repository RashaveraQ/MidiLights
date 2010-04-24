#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	// LED�̓d���X�C�b�`
	DDRA = 0xFF;
	PORTA = 0;

	// ���ڑ�(���g�p)�s���́A�m�C�Y�ϐ�����̂���'0'�o��(GND�ڑ����)�Ƃ��܂��B
	DDRB = 0xFF;
	PORTB = 0;

	// LED�̐���X�C�b�`('0'�o�͂œ_���A'1'�o�͂ŏ����ł���A�ŏ��͏���������̂�'1'�o�͂Ƃ��܂��B)
	DDRC = 0xFF;
	PORTC = 0xFF;

	// D0�́AMIDI����
	// D1-D4�́A���ڑ�
	// D5-D7�́ALED�̐���X�C�b�`
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
