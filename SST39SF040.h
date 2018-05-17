/*
 * SST39SF040.h
 *
 *  Created on: Dec 9, 2015
 *      Author: amir
 */

#ifndef SST39SF040_H_
#define SST39SF040_H_

#define SECTOR_LENGTH 0x1000
#define WRITE_DELAY 50
#define READ_DELAY 50


void flash_setup(void);
void flash_write_lowV(uint32_t address, char data, float volts);
void flash_write(uint32_t address, char data);
void flash_erase_sector(uint32_t address);
void flash_erase_chip(void);
char flash_read(uint32_t address);
void set_flash_voltage(float vin);
void DAC_setup(void);


#endif /* SST39SF040_H_ */

