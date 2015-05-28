/*
 * DiscBrowser.h
 *
 *  Created on: 27.02.2015
 *      Author: sagok
 */

#ifndef DISCBROWSER_H_
#define DISCBROWSER_H_

FRESULT scan_files (char* path);        														// Start node to be scanned (also used as work area)
FRESULT Read_file_on_display (FATFS* FatFs, const TCHAR* path);
FRESULT Read_Random_Bin_Fromfile(FIL* fp, DWORD adr, TCHAR* Tch, UINT size);
FRESULT set_timestamp (char *obj, int year, int month, int mday, int hour, int min, int sec);	//изменяет метку времени (timestamp) у файла или директории.
FRESULT Write_to_logFile(FIL* fp, TCHAR* Tch, BYTE size);
FRESULT Write_to_HexFile(FIL* fp, TCHAR* Tch, BYTE size);
FRESULT OpenFontFile(FIL* FFile, sFONT* Font);
FRESULT CloseFontFile(FIL* FFile);

#endif /* DISCBROWSER_H_ */
