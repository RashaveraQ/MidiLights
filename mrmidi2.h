#ifndef _MM2_H_
#define _MM2_H_


// Config

#define SW_VERSION	0x28


#define ULTIMATE

#define BUFFERED_READ	// Gepuffertes Einlesen von Mididaten bei der Wiedergabe (erhöht Timingspräzision)
#define RC5				// IR-Remote-Unerstützung aktivieren

#define MIDI_MONITOR	// MIDI-Monitor verfügbar
#define KEY_REPEAT		// Taste gedrückt halten ist wie am PC
#define DIREKTWAHL		// Wahl eines Songs per Zifferntasten
#define RANDOMSONG		// Zufallswiedergabe verfügbar
#define MANUAL_CALIBRATE// Kalibrierung des Oszillators im Menü

#define OLD_BL			// Alter 1024-Word-Bootloader im Chip

#endif

