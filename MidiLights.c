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
bool gIsPracticeMode;		// ���K���[�h

u08 gKey = 0x00;
uint8_t gRc5checking = 0;

// �O�����荞��
ISR(INT1_vect)
{
	gKey = 0x01;

/*	static u32 data = 0;
	static u08 nbit = 0;
	static u16 cmd = 0;

	// LED�̓d���X�C�b�`
	DDRA = 0xFF;
	PORTA = 0;

	TIMSK0 &= ~1;		// �^�C�}�O�I�[�o�[�t���[���荞�݋֎~

	// �����R�����蒆
	if (gRc5checking) {
		nbit++;
		data <<= 1;

		// �^�C�}�J�E���^�̊m�F
		if (TCNT0 > 13) {
			data |= 0x0001;
		}

		// 
		if (nbit == 16) {
			cmd = (u16)(0x0000ffff & data);
		}

		// 
		if (nbit == 50) {
			// �������R�}���h�̏ꍇ�A
			//if (cmd == 0x4004) {
				gData[0] = 0xff & (data >> 24);
				gData[1] = 0xff & (data >> 16);
				gData[2] = 0xff & (data >> 8);
				gData[3] = 0xff & data;
			//} else {
			//	gData[0] = gData[1] = gData[2] = gData[3] = 0xff;
			//}

			// �����R�����芮��
			nbit = 0;
			gRc5checking = 0;
			TCCR0B = 0x01;	// �v���X�P�[���́A1
		}
	} else {
		nbit = 0;
		TCCR0B = 0x05;		// �v���X�P�[���́A1024
		gRc5checking = 1;	// �����R�����蒆�Ƃ���B
	}
	

	TCNT0 = 0;	// �^�C�}�[�J�E���g�O
	TIFR0 = 1;	// �^�C�}/�J�E���^�O�I�[�o�[�t���[���荞�݌��ʂ��N���A
	TIMSK0 = 1;	// �^�C�}/�J�E���^�O�I�[�o�[�t���[���荞�݋���
*/
}

// �^�C�}�[�ʒm
ISR(TIMER0_OVF_vect)
{
/*	TIMSK0 &= ~1;		// �^�C�}�O�I�[�o�[�t���[���荞�݋֎~

	// �����R�����蒆
	if (gRc5checking) {
		// �����R�����芮��
		gRc5checking = 0;
		TCCR0B = 0x01;	// �v���X�P�[���́A1
	} else {
*/
		static uint8_t	sLedPowerBit = 0;

		PORTA = 0;	// LED�̓d��OFF

		PORTC = 0xFF & ~(gLEDs[sLedPowerBit]);
		PORTD &= 0x1F;
		PORTD |= 0xE0 & (~(gLEDs[sLedPowerBit] >> 3));

		//gData[2] = rc5.code;
		//gData[3] = rc5.addr;

		PORTA = 1 << sLedPowerBit;	// �w���LED�̓d��ON

		sLedPowerBit++;
		if (sLedPowerBit > 7) {
			sLedPowerBit = 0;
		}
/*	}

	TIMSK0 = 1;		// �^�C�}/�J�E���^�O�I�[�o�[�t���[���荞�݋���
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
	case 0:	// ����
		switch (d & 0xF0) {
		case 0x90:	// �m�[�g�I��
		case 0x80:	// �m�[�g�I�t
			note = -1;
			// break ���Ȃ�
		case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
		case 0xC0:	// �v���O�����`�F���W
			operand_bak = operand = 0xF0 & d;
			break;
		case 0xF0:	// �V�X�e���E���A���^�C���E���b�Z�[�W
			switch (d) {
			case 0xF0:
				operand = d;
				break;
			}
			break;

		default:	// �����j���O�X�e�[�^�X
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

		if (d == 0 || d > 127) {
			operand = 0x80;
		}

		uint8_t idx = note / 11;
		uint16_t data = 1 << (note % 11);
		switch (operand) {
		case 0x90:	// �m�[�g�I��
			gLEDs[idx] |= data;
			break;
		case 0x80:	// �m�[�g�I�t
			gLEDs[idx] &= ~data;
			break;
		}
		note = -1;
		operand = 0;
		break;
	case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
		// �I�[���E�m�[�g�E�I�t
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
		case 0:	// ����
		switch (d & 0xF0) {
			case 0x90:	// �m�[�g�I��
			case 0x80:	// �m�[�g�I�t
			note = -1;
			// break ���Ȃ�
			case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
			case 0xC0:	// �v���O�����`�F���W
			operand_bak = operand = 0xF0 & d;
			break;
			case 0xF0:	// �V�X�e���E���A���^�C���E���b�Z�[�W
			switch (d) {
				case 0xF0:
				operand = d;
				break;
			}
			break;

			default:	// �����j���O�X�e�[�^�X
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

		if (d == 0 || d > 127) {
			operand = 0x80;
		}

		uint8_t idx = note / 11;
		uint16_t data = 1 << (note % 11);
		switch (operand) {
			case 0x90:	// �m�[�g�I��
			gPianoKeys[idx] |= data;
			break;
			case 0x80:	// �m�[�g�I�t
			gPianoKeys[idx] &= ~data;
			break;
		}
		note = -1;
		operand = 0;
		break;
		case 0xB0:	// �R���g���[���`�F���W or ���[�h�E�`�F���W
		// �I�[���E�m�[�g�E�I�t
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
	// ���M�������Ă��Ȃ�����A�J��Ԃ��B
//	while ((UCSR0A & (1 << TXC0)) == 0x00);

	// ���M�f�[�^�E���W�X�^���󂫂łȂ�����A�J��Ԃ��B
	while ((UCSR0A & (1 << UDRE0)) == 0x00);

	// ���M�f�[�^���Z�b�g�B
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

	// LED�̓d���X�C�b�`
	DDRA = 0xFF;
	PORTA = 0;

	// ���ڑ�(���g�p)�s���́A�m�C�Y�ϐ�����̂���'0'�o��(GND�ڑ����)�Ƃ��܂��B
	// MMC�̂��߁ASS(PB4),MOSI(PB5),MISO(PB6),SCK(PB7)��0
	DDRB = 0xBF;
	PORTB = 0xF0;

	// LED�̐���X�C�b�`('0'�o�͂œ_���A'1'�o�͂ŏ����ł���A�ŏ��͏���������̂�'1'�o�͂Ƃ��܂��B)
	DDRC = 0xFF;
	PORTC = 0xFF;

	// D0�́AMIDI���́BD1�́AMIDI�o�́BD3�́A�ԊO�������R����M�p�O�����荞��
	// D5-D7�́ALED�̐���X�C�b�`
	DDRD = 0xF2;
	PORTD = 0xEF;

	// �^�C�}�ݒ�
	TCCR0B = 0x01;	// �v���X�P�[���́A1
	TIMSK0 = 0x01;	// �^�C�}�Q�I�[�o�[�t���[���荞�݋���
	
	UBRR0 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 10MHz
	UBRR1 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 10MHz
//	UBRR0 = 15;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / (16 * BAUD)) - 1; for 8MHz
//	UCSR0B = 0xB8;	// ����M����ю�M�������M�󂫊��荞�݋���
	UCSR0B = 0x98;	// ����M����ю�M�������荞�݋���
	UCSR1B = 0x90;	// ��M����ю�M�������荞�݋���

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
	// �s���ω����荞��(�ԊO�������R����M����)
	EICRA = 0x08;	// �O�����荞�݂P INT1 �� HIGH �� LOW
	EIFR = 0x02;	// ���荞�݃y���f�B���O���N���A
	EIMSK |= 2;		// �O�����荞�݂P����

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
