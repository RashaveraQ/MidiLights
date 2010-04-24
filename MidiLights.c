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
	PORTA = 0;	// LED�̓d��OFF

	PORTC = ~(0xFF & gData[gLedPowerBit]);
	PORTD = 0x01 | (0xE0 & ~(0xE0 & (gData[gLedPowerBit] >> 3)));

	gLedPowerBit++;
	if (gLedPowerBit > 7) {
		gLedPowerBit = 0;
	}

	PORTA = 1 << gLedPowerBit;	// �w���LED�̓d��ON
}

ISR(USART0_RX_vect)
{
	uint8_t d = UDR0;
	gData[d % 8] |= 1 << ((d / 8) % 11);
}

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

	// �^�C�}�ݒ�
	TCCR0B = 0x03;	// �v���X�P�[���́A64
	//TCCR0B = 0x04;	// �v���X�P�[���́A256
	//TCCR0B = 0x05;	// �v���X�P�[���́A1024
	TIMSK0 = 0x01;	// �^�C�}�O�I�[�o�[�t���[���荞�݋���

	UBRR0 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
	UCSR0B = 0x90;	// ��M����ю�M�������荞�݋���

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
