#ifndef _RC5_H_
#define _RC5_H_

#include <inttypes.h>

#define RC5_INT0 0
#define RC5_INT1 1

#define RC5_ALL 0xff

typedef struct
{
	uint8_t code;
	uint8_t addr;
	volatile signed char flip;
} rc5_t;

extern rc5_t rc5;
extern void rc5_init (uint8_t addr);

#endif /* _RC5_H_ */
