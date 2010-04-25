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

	PORTA = 0;	// LED�̓d��OFF

	PORTC = ~(0xFF & gData[sLedPowerBit]);
	PORTD = 0x01 | (0xE0 & ~(0xE0 & (gData[sLedPowerBit] >> 3)));

	sLedPowerBit++;
	if (sLedPowerBit > 7) {
		sLedPowerBit = 0;
	}

	PORTA = 1 << sLedPowerBit;	// �w���LED�̓d��ON
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
	case 0:	// ����
		switch (d & 0xF0) {
		case 0x90:	// �m�[�g�I��
		case 0x80:	// �m�[�g�I�t
			//gData[4] = 0xFFFF;
		case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
		case 0xC0:	// �v���O�����`�F���W
			operand = 0xF0 & d;
			break;
		case 0xF0:	// �V�X�e���E���A���^�C���E���b�Z�[�W
			if (d == 0xF0) {
				operand = d;
			}
			break;
		}
		break;
	case 0x90:	// �m�[�g�I��
	case 0x80:	// �m�[�g�I�t
		// �f�[�^�o�C�g�ɂĎw�肷��m�[�g�i���o�[�Ƃ́A
		// �ł��Ⴂ����0�A�ł���������127�Ɗ��蓖�Ă����̍����̂��Ƃł���B
		// �����n�ɂ̓m�[�g�i���o�[60�����蓖�Ă��A
		// 88���Ղ̃O�����h�s�A�m�ŏo���鉹���
		// �m�[�g�i���o�[21�`108�Ɗ��蓖�Ă���
		d -= 21;	// LED�̔z�����C����p
		//d -= 18;	// �b��LED�̔z���𒼂��O


		uint8_t idx = d / 11;
		uint8_t data = 1 << (d % 11);
		switch (operand) {
		case 0x90:	// �m�[�g�I��
			gData[idx] |= data;
			break;
		case 0x80:	// �m�[�g�I�t
			gData[idx] &= ~data;
			break;
		}
		ignore_count = 1;
		operand = 0;
		break;
	case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
		// �I�[���E�m�[�g�E�I�t
		if (d == 0x7B) {
			for (int i = 0; i < 8; i++) {
				gData[i] = 0;
			}
		}
		// break ���Ȃ�
	case 0xC0:	// �v���O�����`�F���W
		ignore_count = 1;
		operand = 0;
		break;
	case 0xF0:	// �V�X�e���E���A���^�C���E���b�Z�[�W
		// �I�����b�Z�[�W
		if (d == 0xF7) {
			operand = 0;
		}
		break;
	}
}

int main(void)
{
	cli();

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
	TCCR0B = 0x01;	// �v���X�P�[���́A1
	TIMSK0 = 0x01;	// �^�C�}�O�I�[�o�[�t���[���荞�݋���

	UBRR0 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
	UCSR0B = 0x90;	// ��M����ю�M�������荞�݋���

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
