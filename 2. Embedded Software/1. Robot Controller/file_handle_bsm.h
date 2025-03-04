/*
 * file_handle_bsm.h
 *
 *  Created on: Jun 9, 2022
 *      Author: bsm6656
 */

#ifndef INC_FILE_HANDLE_BSM_H_
#define INC_FILE_HANDLE_BSM_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"

extern FATFS fs;
extern FIL fil;
extern FILINFO filnfo;
extern FRESULT fresult;
extern UINT br, bw;


extern FRESULT Mount_SD(const TCHAR* path);
extern void unMount_SD(const TCHAR* path);

extern FRESULT Scan_SD();		//	Scan dir and check existing files
extern FRESULT Write_File(char* data);	//	Write data to the opened file
extern FRESULT Create_File();	//	Create file
extern FRESULT Open_File();
extern FRESULT Write_openedFile(char* data);
extern FRESULT Close_File();
#endif /* INC_FILE_HANDLE_BSM_H_ */
