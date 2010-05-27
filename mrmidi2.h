#ifndef _MM2_H_
#define _MM2_H_


// Config

#define SW_VERSION	0x28


//#define FULL
#define ULTIMATE
//#define NORMAL
//#define LED_DISP		// Kompletter Wegfall aller Display-/Aufnahmefunktionen


#ifdef FULL
	#define BUFFERED_READ	// Gepuffertes Einlesen von Mididaten bei der Wiedergabe (erhöht Timingspräzision)
	#define RC5				// IR-Remote-Unerstützung aktivieren
	
	#define BACKLIGHT		// LCD-Hintergrundbeleuchtung im Menü einstellbar
	#define MIDI_MONITOR	// MIDI-Monitor verfügbar
	#define KEY_REPEAT		// Taste gedrückt halten ist wie am PC
	#define DIREKTWAHL		// Wahl eines Songs per Zifferntasten
	#define RANDOMSONG		// Zufallswiedergabe verfügbar
	#define MANUAL_CALIBRATE// Kalibrierung des Oszillators im Menü
	#define CALIBRATE		// Automatische Kalibrierroutine
	
	//#define RC5_DISP		// Debug Displayanzeige
	
	//#define OLD_CARDSCAN	// Raw-Scan der Karte, wenn kein FAT16 gefunden wurde
	//#define NOSECTORSEARCH	// Cluster werden nicht aus FAT allokiert (fragmentierte Karten nicht erlaubt)
	#define USE_RTC		// Echtzeituhr angeschlossen
	#define RTC_NUMERIC	// Eingabe der Zeit per Ziffentasten
#endif

#ifdef ULTIMATE
	#define BUFFERED_READ	// Gepuffertes Einlesen von Mididaten bei der Wiedergabe (erhöht Timingspräzision)
	#define RC5				// IR-Remote-Unerstützung aktivieren
	
	//#define BACKLIGHT		// LCD-Hintergrundbeleuchtung im Menü einstellbar
	#define MIDI_MONITOR	// MIDI-Monitor verfügbar
	#define KEY_REPEAT		// Taste gedrückt halten ist wie am PC
	#define DIREKTWAHL		// Wahl eines Songs per Zifferntasten
	#define RANDOMSONG		// Zufallswiedergabe verfügbar
	#define MANUAL_CALIBRATE// Kalibrierung des Oszillators im Menü
	//#define CALIBRATE		// Automatische Kalibrierroutine
	
	//#define RC5_DISP		// Debug Displayanzeige
	
	//#define OLD_CARDSCAN	// Raw-Scan der Karte, wenn kein FAT16 gefunden wurde
	//#define NOSECTORSEARCH	// Cluster werden nicht aus FAT allokiert (fragmentierte Karten nicht erlaubt)
	//#define USE_RTC		// Echtzeituhr angeschlossen
	//#define RTC_NUMERIC	// Eingabe der Zeit per Ziffentasten
#endif

#ifdef NORMAL
	#define BUFFERED_READ	// Gepuffertes Einlesen von Mididaten bei der Wiedergabe (erhöht Timingspräzision)
	#define RC5				// IR-Remote-Unerstützung aktivieren
	
	#define BACKLIGHT		// LCD-Hintergrundbeleuchtung im Menü einstellbar
	//#define MIDI_MONITOR	// MIDI-Monitor verfügbar
	//#define KEY_REPEAT		// Taste gedrückt halten ist wie am PC
	#define DIREKTWAHL		// Wahl eines Songs per Zifferntasten
	#define RANDOMSONG		// Zufallswiedergabe verfügbar
	//#define MANUAL_CALIBRATE// Kalibrierung des Oszillators im Menü
	//#define CALIBRATE		// Automatische Kalibrierroutine
	
	//#define RC5_DISP		// Debug Displayanzeige
	
	#define OLD_CARDSCAN	// Raw-Scan der Karte, wenn kein FAT16 gefunden wurde
	//#define NOSECTORSEARCH	// Cluster werden nicht aus FAT allokiert (fragmentierte Karten nicht erlaubt)
	//#define USE_RTC		// Echtzeituhr angeschlossen
	//#define RTC_NUMERIC	// Eingabe der Zeit per Ziffentasten
#endif

#ifdef LED_DISP
	#define BUFFERED_READ	// Gepuffertes Einlesen von Mididaten bei der Wiedergabe (erhöht Timingspräzision)
#endif

#define OLD_BL			// Alter 1024-Word-Bootloader im Chip



#endif

