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

//	PORTD = 0x03 | ~(gData[sLedPowerBit] << 1);
	PORTD &= 0x03;
	PORTD |= 0xFC & (~(gData[sLedPowerBit] << 1));
	PORTB = 0x02 & ~(gData[sLedPowerBit] << 1);
	PORTC = (0x03 & ~(gData[sLedPowerBit] >> 7)) | (0xC0 & ~(gData[sLedPowerBit] >> 3));

	PORTA = 1 << sLedPowerBit;	// �w���LED�̓d��ON

	sLedPowerBit++;
	if (sLedPowerBit > 7) {
		sLedPowerBit = 0;
	}
}



void recv(uint8_t d)
{
	//gData[1] = 0x1;

	static uint8_t	operand = 0;
	static int8_t	note = -1;
	static uint8_t	ignore_count = 0;

	UDR0 = d;

	if (ignore_count) {
		ignore_count--;
		return;
	}

	switch (operand) {
	case 0:	// ����
		switch (d & 0xF0) {
		case 0x90:	// �m�[�g�I��
		case 0x80:	// �m�[�g�I�t
			note = -1;
			// break ���Ȃ�
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

		if (note == -1) {
			if (d < 21 || 108 < d) {
				ignore_count = 1;
				operand = 0;
				break;
			}
			note = d - 21;
			break;
		}

		if (d == 0) {
			operand = 0x80;
		}

		uint8_t idx = note / 11;
		uint16_t data = 1 << (note % 11);
		switch (operand) {
		case 0x90:	// �m�[�g�I��
			gData[idx] |= data;
			break;
		case 0x80:	// �m�[�g�I�t
			gData[idx] &= ~data;
			break;
		}
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

ISR(USART0_RX_vect)
{
	uint8_t d = UDR0;
	recv(d);
}


volatile int gFlag;

ISR(USART0_TX_vect)
{
	gData[0] = 0x1;
	gFlag = 0;
}

ISR(USART0_UDRE_vect)
{
	gData[0] = 0x2;
	gFlag = 0;
}

void send(uint8_t data)
{
	// ���M�������Ă��Ȃ�����A�J��Ԃ��B
//	while ((UCSR0A & (1 << TXC0)) == 0x00);

	// ���M�f�[�^�E���W�X�^���󂫂łȂ�����A�J��Ԃ��B
//	while ((UCSR0A & (1 << UDRE0)) == 0x00);
	_delay_ms(1);

	// ���M�f�[�^���Z�b�g�B
	recv(data);
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

	// LED�̓d���X�C�b�`
	DDRA = 0xFF;
	PORTA = 0;

	// ���ڑ�(���g�p)�s���́A�m�C�Y�ϐ�����̂���'0'�o��(GND�ڑ����)�Ƃ��܂��B
	DDRB = 0xFF;
	PORTB = 0x02;

	// LED�̐���X�C�b�`('0'�o�͂œ_���A'1'�o�͂ŏ����ł���A�ŏ��͏���������̂�'1'�o�͂Ƃ��܂��B)
	DDRC = 0xFF;
	PORTC = 0xC3;

	// D0�́AMIDI����
	// D1-D7�́ALED�̐���X�C�b�`
	DDRD = 0xFE;
	PORTD = 0xFD;

	// �^�C�}�ݒ�
	TCCR0B = 0x01;	// �v���X�P�[���́A1
	TIMSK0 = 0x01;	// �^�C�}�O�I�[�o�[�t���[���荞�݋���

	UBRR0 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
	UCSR0B = 0x98;	// ����M����ю�M�������荞�݋���

	sei();

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

	for (;;) {
/*for (int8_t i = 0; i < 88; i++) {
			note_on(i);
			_delay_ms(1);
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

		note_off();
*/
		_delay_ms(5000);
	}
}
