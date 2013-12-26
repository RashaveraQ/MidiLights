#include <avr/io.h>
#include <avr/interrupt.h>
#include "rc5.h"


extern uint16_t	gLEDs[8];	// akashi

#define __AVR_ATmega168__	// akashi

#ifndef RC5_INT
	#define RC5_INT      RC5_INT0
#endif  /* RC5_INT */
  
#ifndef RC5_PRESCALE
	#define RC5_PRESCALE 256
#endif  /* RC5_PRESCALE */

/* ******************************************************************************** */

rc5_t rc5;

/* ******************************************************************************** */

#ifndef F_CPU
	#define F_CPU 10000000
#endif /* !F_CPU */

/* �s for a whole bit of RC5 (first & second part) */
#define RC5_BIT_US   (64*27)

// 68 
#define RC5_TICKS \
        ((uint8_t) ((uint32_t) (F_CPU / 1000 * RC5_BIT_US / 1000 / RC5_PRESCALE)))
        
// 11.333
#define RC5_DELTA \
        (RC5_TICKS / 6)
        
typedef union 
{
	uint16_t w;
	uint8_t  b[2];
} code_t;

static code_t code;
static uint8_t rc5_addr;

/* Number of Bits received so far */
/* Number of Interrupts occured so far */
static uint8_t nbits;	// ���܂܂ł̂Ƃ����M�����r�b�g��
static uint8_t nint;	// ���܂܂ł̂Ƃ��늄�荞�݉�

extern uint8_t gRc5checking;

/* ******************************************************************************** */

void rc5_init2()
{
#if (RC5_PRESCALE==1024)
	TCCR0A = 0;
	TCCR0B = (1 << CS02) | (1 << CS00);
#elif   (RC5_PRESCALE==256)
	TCCR0A = 0;				// ���샂�[�h:�m�[�}��
	TCCR0B = (1 << CS02);	// �v���X�P�[��:1024
#elif   (RC5_PRESCALE==64)
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);
#else
  #error This RC5_PRESCALE is not supported
#endif /* RC5_PRESCALE */
}
        
void rc5_init (uint8_t addr)
{
	nint  = 0;
	nbits = 0;
	rc5.flip = -1;
	rc5_addr = addr;

        /* INTx on falling edge */
        /* clear pending INTx */
        /* enable INTx interrupt */
	EICRA = 0x02;	// �O�����荞�݂O INT0 �� HIGH �� LOW
//	EICRA = 0x03;	// �O�����荞�݂O INT0 �� LOW �� HIGH
	EIFR = 0x01;	// ���荞�݃y���f�B���O���N���A
	EIMSK |= 1;		// �O�����荞�݂O����
}

// akashi
// �����R���m�F���[�h���I�����ALED�_�����[�h�ɕύX����B
void rc5_exit()
{
	// LED�_�����[�h�ɕύX����B
	gRc5checking = 0;	// �����R���m�F�����I�t
	TCCR0B = 0x01;	// �v���X�P�[���́A1
	TIMSK0 = 0x01;	// �^�C�}�O�I�[�o�[�t���[���荞�݋���
}


/* ******************************************************************************** */
// �^�C�}�O�I�[�o�[�t���[���荞�ݏ���
//ISR(TIMER0_OVF_vect)
void rc5_TIMER0_OVF_vect()
{
	TIMSK0 &= ~1;		// �^�C�}�O�I�[�o�[�t���[���荞�݋֎~

	uint8_t _nbits = nbits;
	code_t _code = code;

	// ��M�����r�b�g����26�̏ꍇ�A
	if (26 == _nbits) {
		_nbits++;		// ��M�����r�b�g�����C���N�������g
		_code.w <<= 1;	// �R�[�h������1�r�b�g�V�t�g����B
	}

	// ��M�����r�b�g����27�̏ꍇ���Acode �̉��ʃo�C�g��0x30�ȏォ�A
	// rc5.flip �� �O�𒴂���ꍇ�A
	if (27 == _nbits && _code.b[1] >= 0x30 /* AGC == 3 */
        && 0 > rc5.flip)
	{
		uint8_t _rc5_code;
		uint8_t _rc5_addr;
		/* we do the bit manipulation stuff by hand, because of code size */
		// �ԊO�������R���̃R�[�h�ƃA�h���X���擾
		_rc5_code = _code.b[0] & 0x3f; /* 0b00111111 : #0..#5 */
		_code.w <<= 2;
		_rc5_addr = _code.b[1] & 0x1f; /* 0b00011111 : #6..#10 */

		// 
		if (rc5_addr & 0x80 || rc5_addr == _rc5_addr) {
			rc5.code = _rc5_code;
			rc5.addr = _rc5_addr;
			signed char flip = 0;
			if (_code.b[1] & 0x20) /* 0b00100000 : #11 */
			{
				flip = 1;
			}
			rc5.flip = flip;

			// �����R���m�F���[�h���I�����ALED�_�����[�h�ɕύX����B
			rc5_exit();
		}
	}

	nint = 0;
	nbits = 0;

	/* INTx on falling edge */
	/* clear pending INTx */
	/* enable INTx interrupt */
//	EICRA = 0x02;	// �O�����荞�݂O INT0 �� HIGH �� LOW
	EICRA = 0x03;	// �O�����荞�݂O INT0 �� LOW �� HIGH
	EIFR = 0x01;	// ���荞�݃y���f�B���O���N���A
	EIMSK |= 1;		// �O�����荞�݂O����
}

/* ******************************************************************************** */

// �O�����荞��
//ISR(INT0_vect)
void rc5_INT0_vect()
{
	code_t _code = code;
	uint8_t _nint = nint;

	uint8_t tcnt0 = TCNT0;	// �^�C�}/�J�E���^�l���擾�B
	TCNT0 = 0;				// �^�C�}/�J�E���^�l���������B

	if (0 == _nint) {
		/* INTx on both edges */
		EICRA = 0x01;		// INT0 �� Low��High, High��Low
		TIFR0 = 1;			// �^�C�}/�J�E���^�O�I�[�o�[�t���[���荞�݌��ʂ��N���A
		TIMSK0 = 1;			// �^�C�}/�J�E���^�O�I�[�o�[�t���[���荞�݋���
		_code.w = 0;		// 
	} else {
		/* Number of bits of the just elapsed period */
		uint8_t n = 1;

		/* Bits received so far */
		uint8_t _nbits = nbits;

		/* is TCNT0 close to RC5_TICKS or RC5_TICKS/2 ? */
		if (tcnt0 > RC5_TICKS + RC5_DELTA) {			// over 79  -> ng
			goto invalid;
		} else if (tcnt0 < RC5_TICKS/2 - RC5_DELTA) {	// under 23 -> ng
			goto invalid;
		} else if (tcnt0 > RC5_TICKS - RC5_DELTA) {		// over 57 -> ok
			n = 2;
		} else if (tcnt0 > RC5_TICKS/2 + RC5_DELTA) {	// over 45 -> ng
			goto invalid;
		}

		gLEDs[0]++;	// akashi

		/* store the just received 1 or 2 bits */
		do {
			_nbits++;
			if (_nbits & 1) {
				_code.w <<= 1;
				_code.b[0] |= _nint & 1;
			}
		} while (--n);

		if (0) {
			invalid:
			gLEDs[1]++;	// akashi
			/* disable INTx, run into Overflow0 */
			EIMSK &= ~1;	// �O�����荞�݋֎~
			_nbits = 0;

			// �����R���m�F���[�h���I�����ALED�_�����[�h�ɕύX����B
			rc5_exit();
		}

		nbits = _nbits;
	}

	code = _code;
	nint = 1 + _nint;
}
