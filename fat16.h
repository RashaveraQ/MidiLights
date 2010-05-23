#include "mmc.h"
#include "lcd.h"
#include "types.h"

struct fat_filedata_st {
	u32 len;
	u32 startsect;
	u16 rootsect;
	u08 rootsectoffs;
	u08 entries; // LFN => entries > 1
	u08 name[LINESIZE];
};

#define FILEMODE_CLOSED 0
#define FILEMODE_READ 1
#define FILEMODE_WRITE 2

// functions
u08 fat_init();
void print_filename(u08 y);
void print_filesize(void);
void fat_count_files(void);
u08 fat_read_filedata(u08 nr);	// new file data in global struct
void fat_delete_file(void);
void fat_new_file(void);	// new file data in global struct
u08 fat_openfile(u08 mode, u08 nr);
u08 fat_closefile(void);
u08 fat_getnextsector(void);
