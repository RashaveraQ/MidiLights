#include <avr/io.h>
#include <avr/interrupt.h>
#include "rc5.h"

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

/* µs for a whole bit of RC5 (first & second part) */
#define RC5_BIT_US   (64*27)

#define RC5_TICKS \
        ((uint8_t) ((uint32_t) (F_CPU / 1000 * RC5_BIT_US / 1000 / RC5_PRESCALE)))
        
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
static uint8_t nbits;
static uint8_t nint;

/* ******************************************************************************** */
        
void rc5_init (uint8_t addr)
{
	nint  = 0;
	nbits = 0;
	rc5.flip = -1;
	rc5_addr = addr;
        
#if (RC5_PRESCALE==1024)
	TCCR0A = 0;
	TCCR0B = (1 << CS02) | (1 << CS00);
#elif   (RC5_PRESCALE==256)
	TCCR0A = 0;
	TCCR0B = (1 << CS02);
#elif   (RC5_PRESCALE==64)
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);
#else
  #error This RC5_PRESCALE is not supported
#endif /* RC5_PRESCALE */
        
        /* INTx on falling edge */
        /* clear pending INTx */
        /* enable INTx interrupt */
#if (RC5_INT == RC5_INT0)               
	EICRA = 0x02;
	EIFR = 0x01;
	EIMSK |= 1;
#elif (RC5_INT == RC5_INT1)             
	EICRA = 0x02<<2;
	EIFR = 0x01<<1;
	EIMSK |= 1<<1;
#else
  #error please define RC5_INT
#endif /* RC5_INT */
}

/* ******************************************************************************** */

ISR(SIG_OVERFLOW0)
{
	TIMSK0 &= ~1;

	uint8_t _nbits = nbits;
	code_t _code = code;

	if (26 == _nbits) {
		_nbits++;
		_code.w <<= 1;
	}

	if (27 == _nbits && _code.b[1] >= 0x30 /* AGC == 3 */
        && 0 > rc5.flip)
	{
		uint8_t _rc5_code;
		uint8_t _rc5_addr;
		/* we do the bit manipulation stuff by hand, because of code size */
		_rc5_code = _code.b[0] & 0x3f; /* 0b00111111 : #0..#5 */
		_code.w <<= 2;
		_rc5_addr = _code.b[1] & 0x1f; /* 0b00011111 : #6..#10 */

		if (rc5_addr & 0x80 || rc5_addr == _rc5_addr) {
			rc5.code = _rc5_code;
			rc5.addr = _rc5_addr;
			signed char flip = 0;
			if (_code.b[1] & 0x20) /* 0b00100000 : #11 */
				flip = 1;
			rc5.flip = flip;
		}
	}

	nint = 0;
	nbits = 0;

	/* INTx on falling edge */
	/* clear pending INTx */
	/* enable INTx interrupt */
#if (RC5_INT == RC5_INT0)               
	EICRA = 0x02;
	EIFR = 0x01;
	EIMSK |= 1;
#elif (RC5_INT == RC5_INT1)             
	EICRA = 0x02<<2;
	EIFR = 0x01<<1;
	EIMSK |= 1<<1;
#endif
}

/* ******************************************************************************** */

#if (RC5_INT == RC5_INT0)               
ISR(SIG_INTERRUPT0)
#elif (RC5_INT == RC5_INT1)             
ISR(SIG_INTERRUPT1)
#endif /* RC5_INT */
{
        code_t _code = code;
        uint8_t _nint = nint;
        
        uint8_t tcnt0 = TCNT0;
        TCNT0 = 0;
        
        if (0 == _nint)
        {
                /* INTx on both edges */
#if (RC5_INT == RC5_INT0)
  				EICRA = 0x01;
#elif (RC5_INT == RC5_INT1)             
  				EICRA = 0x01<<2;
#endif /* RC5_INT */
        
                TIFR0 = 1;
                TIMSK0 = 1;
                _code.w = 0;
        }
        else
        {
                /* Number of bits of the just elapsed period */
                uint8_t n = 1;
         
                /* Bits received so far */
                uint8_t _nbits = nbits;
        
                /* is TCNT0 close to RC5_TICKS or RC5_TICKS/2 ? */
                if (tcnt0 > RC5_TICKS + RC5_DELTA)
                        goto invalid;
                else if (tcnt0 < RC5_TICKS/2 - RC5_DELTA)
                        goto invalid;
                else if (tcnt0 > RC5_TICKS - RC5_DELTA)
                        n = 2;
                else if (tcnt0 > RC5_TICKS/2 + RC5_DELTA)
                        goto invalid;
                
                /* store the just received 1 or 2 bits */
                do
                {
                        _nbits++;
                        if (_nbits & 1)
                        {
                                _code.w <<= 1;
                                _code.b[0] |= _nint & 1;
                        }
                } 
                while (--n);
                
                if (0)
                {
                        invalid:
                        
                        /* disable INTx, run into Overflow0 */
#if (RC5_INT == RC5_INT0)               
  						EIMSK &= ~1;
#elif (RC5_INT == RC5_INT1)             
  						EIMSK &= ~2;
#endif /* RC5_INT */

                        _nbits = 0;
                }
                
                nbits = _nbits;
        }

        code = _code;
        nint = 1+_nint;
}
