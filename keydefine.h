/*
 * keydefine.h
 *
 * Created: 2013/12/27 21:34:28
 *  Author: akashi
 */ 


#ifndef KEYDEFINE_H_
#define KEYDEFINE_H_

// Buttons
//#define KEY_PORT	PIND	// akashi
//#define KEY_PORT2	PINB	// akashi

#define KEY_MASK	0xf8
#define KEY_PLAY	0x80

#define KEY_REP_TIME_INIT	80
#define KEY_REP_TIME		20
#define FAST_SCROLL			30
#define KEY_REP_TIME_INIT_IR	5
#define KEY_REP_TIME_IR			1

#define KEY_MASK2	0x03

#define KEY_STOP	0x08
#define KEY_REC		0x10
#define KEY_NEXT	0x20
#define KEY_LAST	0x40
#define KEY_LEFT	0x02
#define KEY_RIGHT	0x01

// States
#define STOP	0
#define PLAY	1
#define REC		2
#define ABORT_REC	3
#define DELETE	4
#define MENU	5
#define ERR		0x80

#define FILE_NUM_START_NOTE 6	// ファイル番号開始ノート

#endif /* KEYDEFINE_H_ */