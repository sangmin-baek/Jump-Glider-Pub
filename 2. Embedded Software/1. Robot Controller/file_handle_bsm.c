/*
 * file_handle_bsm.c
 *
 *  Created on: Jun 9, 2022
 *      Author: bsm6656
 */
#include "file_handle_bsm.h"

FATFS fs;
FIL fil;
FILINFO filnfo;
FRESULT fresult;
UINT br, bw;

char Filename[8];

/*void Mount_SD(const TCHAR* path){
	fresult = f_mount(&fs, path, 1);

}*/

FRESULT Mount_SD(const TCHAR* path){
	fresult = f_mount(&fs, path, 1);
	return fresult;
}

void unMount_SD(const TCHAR* path){
	fresult = f_mount(NULL, path, 1);
}

FRESULT Open_File(){	//	Write data to the opened file
	fresult = f_open(&fil, Filename, FA_OPEN_APPEND | FA_WRITE);
	if(fresult == FR_OK){
		return fresult;

	}
}
FRESULT Write_openedFile(char* data){	//	Write data to the opened file
	fresult = f_write(&fil, data, strlen(data), &bw);

	return fresult;


}

FRESULT Close_File(){	//	Write data to the opened file
	fresult = f_close(&fil);

	return fresult;


}

FRESULT Write_File(char* data){	//	Write data to the opened file
	fresult = f_open(&fil, Filename, FA_OPEN_APPEND | FA_WRITE);
	if(fresult == FR_OK){
		fresult = f_write(&fil, data, strlen(data), &bw);
		if(fresult == FR_OK){
			fresult = f_close(&fil);
			return fresult;
		}
	}
}

FRESULT	Create_File(){	//	Create file

	uint i=0;
	char buff[256];
	do{
		sprintf(Filename,"%d.txt\n",i);
		fresult = f_stat(Filename, &filnfo);
		i++;
	}while(fresult == FR_OK);

	fresult = f_open(&fil, Filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if(fresult == FR_OK){
		sprintf(buff,"NewFile\r\n");
		f_write(&fil, buff, strlen(buff), &bw);

		//fresult = f_puts("NewFile\n", &fil);
		fresult = f_close(&fil);
		if(fresult == FR_OK){
			return fresult;
		}
	}
}

