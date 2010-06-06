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

void main2(void);
extern u16 file_cnt;

uint16_t	gData[8];

// �^�C�}�[�ʒm
ISR(TIMER2_OVF_vect)
{
	static uint8_t	sLedPowerBit = 0;

	PORTA = 0;	// LED�̓d��OFF

	PORTC = 0xFF & ~(gData[sLedPowerBit]);
	PORTD &= 0x1F;
	PORTD |= 0xE0 & (~(gData[sLedPowerBit] >> 3));

//	gData[2] = rc5.code;
//	gData[3] = rc5.addr;
//	gData[4] = rc5.flip;

	PORTA = 1 << sLedPowerBit;	// �w���LED�̓d��ON

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
			gData[idx] |= data;
			break;
		case 0x80:	// �m�[�g�I�t
			gData[idx] &= ~data;
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
				gData[i] = 0;
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

void error(uint8_t err) {
	sei();
	for (u08 i = 0; i < 5; i++) {
		gData[0] = err;
		delay_ms(500);
		gData[0] = 0xff;
		delay_ms(100);
		gData[0] = 0;
		delay_ms(100);
	}	
}

u08 gKey = 0x00;

// Pin Change Interrupt Request 3
ISR(PCINT3_vect)
{
	gKey = 0x01;
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

	// D0�́AMIDI���́BD1�́AMIDI�o�́BD2�́A�ԊO�������R����M�p�O�����荞��
	// D5-D7�́ALED�̐���X�C�b�`
	DDRD = 0xFA;
	PORTD = 0xE7;

	// �^�C�}�ݒ�
	TCCR2B = 0x01;	// �v���X�P�[���́A1
	TIMSK2 = 0x01;	// �^�C�}�Q�I�[�o�[�t���[���荞�݋���
	
	UBRR0 = 19;		// MIDI�̃{�[���[�g�́A31.25Kbps  UBRRn = (fosc / 16 * BAUD) - 1
//	UCSR0B = 0xB8;	// ����M����ю�M�������M�󂫊��荞�݋���
	UCSR0B = 0x98;	// ����M����ю�M�������荞�݋���

	MMC_hw_init();
	spi_init();
	//rc5_init(RC5_ALL);

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
	// �s���ω����荞��(�ԊO�������R����M����)
	PCICR = 1 << PCIE3;		// �|�[�gD
	PCMSK3 = 1 << PCINT26;	

//	for (;;) {	
//		delay_ms(1000);
//	}

	// MMC/SD�J�[�h�p�̏���������
//	delay_ms(1024);	// SD�J�[�h�����肷��̂�҂B

/*
	u08 res = MMC_init();
	if (res == 1) {
		if (!fat_init()) {
			error(0xaa);
		}
		fat_count_files();
		fat_read_filedata(0);
	} else {
		// mmc error
		error(res);
	}

	gData[1] = (u08)(0x00ff & file_cnt);
	gData[2] = (u08)((0xff00 & file_cnt) >> 8);	
	error(0xc3);
*/
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
