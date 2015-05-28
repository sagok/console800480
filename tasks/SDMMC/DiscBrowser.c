/*
 * DiscBrowser.c
 *
 *  Created on: 27.02.2015
 *      Author: sagok
 */

/* FatFs includes component */
#include "ff_gen_drv.h"

#include "RTC.h"
#include "Fonts/fonts.h"

/***********************************************************************
 *
 ***********************************************************************/

FRESULT scan_files (char* path)        /* Start node to be scanned (also used as work area) */
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */
	FILINFO FileInfo;


#if _USE_LFN
    static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
			    f_stat (fn,&FileInfo);					/* Get file status */

		//	    fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
		//	    fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);

                printf("%s/%s	%02d:%02d:%u %02d.%02d.%02d \n", path, fn, FileInfo.fdate & 0x1F,(FileInfo.fdate>>5) & 0x0F, 1980+((FileInfo.fdate>>9) & 0x7F), (FileInfo.ftime>>11) & 0x1F ,(FileInfo.ftime>>5) & 0x3F ,FileInfo.ftime & 0x1F);
            }
        }
        f_closedir(&dir);
    }

    return res;
}

/***********************************************************************
 *  Read a text file and display it
 ***********************************************************************/

//int main (void)
FRESULT Read_file_on_display (FATFS* FatFs, const TCHAR* path)
{
    FIL fil;       /* File object */
    char line[82]; /* Line buffer */
    FRESULT fr;    /* FatFs return code */


    /* Open a text file */
    fr = f_open(&fil,path, FA_READ);
    if (fr) return (int)fr;

    /* Read all lines and display it */
    while (f_gets(line, sizeof line, &fil))
        printf(line);

    printf("\n");
    /* Close the file */
    f_close(&fil);

    return FR_OK;
}

/***********************************************************************
 *
 ***********************************************************************/
FRESULT Read_Random_Bin_Fromfile(FIL* fp, DWORD adr, TCHAR* Tch, UINT size)
{
	FRESULT res;
	uint32_t bytesread;

	f_lseek(fp,adr);
	res = f_read(fp, Tch, size, (void *)&bytesread);

	return	res;
}

/***********************************************************************
 *
 ***********************************************************************/
FRESULT set_timestamp (
    char *obj,     /* ”казатель на им€ файла */
    int year,
    int month,
    int mday,
    int hour,
    int min,
    int sec
)
{
    FILINFO fno;
    fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
    fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);
    return f_utime(obj, &fno);
}
/***********************************************************************
 *
 ***********************************************************************/
FRESULT Write_to_logFile(FIL* fp, TCHAR* Tch, BYTE size)
{
	FRESULT 	res;
	uint32_t 	byteswritten;
	uint8_t		Hours,Minutes,Seconds;
	uint8_t			Date,Month,Year;

extern uint8_t 	aShowTime[50];
extern uint8_t 	aShowDate[50];

	RTC_GetTime(&Hours, &Minutes,&Seconds);
	RTC_GetDate(&Date,&Month,&Year);

	printf("[RTC] %u %u %u %u %u %u\n",Hours,Minutes,Seconds,Date,Month,Year);

	res = f_open(fp, "log.txt",  FA_OPEN_ALWAYS | FA_WRITE);		// создадим файл если его нет
	 if(res != FR_OK)
	 {
		printf("[SD] 'log.txt' file open error (%d)\n",res);
		return (FR_INT_ERR);
	 } else {
	 f_lseek(fp,f_size(fp));										// го в конец файла.
	 f_write(fp, aShowDate, 11, (void *)&byteswritten);
	 f_write(fp, aShowTime, 9, (void *)&byteswritten);
	 f_write(fp, Tch, size, (void *)&byteswritten);
	 f_close(fp);
	 set_timestamp("log.txt",2015,4,14,13,20,10);

	return (FR_OK);

	}
}

/***********************************************************************
 *
 ***********************************************************************/
FRESULT Write_to_HexFile(FIL* fp, TCHAR* Tch, BYTE size)
{
	uint32_t 	byteswritten;

	 f_write(fp, Tch, 1, (void *)&byteswritten);

	return (FR_OK);

}

/***********************************************************************
 *
 ***********************************************************************/
FRESULT OpenFontFile(FIL* FFile, sFONT* Font){
	char		byteread[5];
	uint8_t		error;
	FIL 		LogFileOnSD;

//scan_files("fonts");
f_chdir("/fonts");											// сменим директорию
   if(f_open(FFile, "cour36b.FFN", FA_READ) != FR_OK)		//"Arial36B.FFN"
	{
		printf("[OpenFontFile] :not open the cour36b.FFN file\n");
		f_chdir("/");											// сменим директорию
		Write_to_logFile(&LogFileOnSD, (char *)"[OpenFontFile] :not open the FontFile\n",sizeof("[OpenFontFile] :not open the FontFile\n")-1);	// добавим в лог.
		return FR_NO_FILE;
	} else {
		printf("[OpenFontFile] :open the cour36b.FFN file\n");
		error = f_gets(&byteread[0],5,FFile);
		if (error == 0) {f_close(FFile); return FR_NOT_READY;}

		Font->Width = (uint16_t)byteread[0]<<8 | (uint16_t)byteread[1];
		Font->Height = (uint16_t)byteread[2]<<8 | (uint16_t)byteread[3];
		Font->table =(uint8_t *)"cour36b.fbn";
		f_close(FFile);
	}
   if (f_open(FFile, "cour36b.FBN", FA_READ) != FR_OK)   return FR_NO_FILE;
   return FR_OK;
}
/***********************************************************************
 *
 ***********************************************************************/
FRESULT CloseFontFile(FIL* FFile){

	f_close(FFile);
	f_chdir("/");
return FR_OK;
}
