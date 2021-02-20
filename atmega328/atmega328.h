#ifndef __ATMEGA2561_H__ //__ESP32_H__
#define __ATMEGA2561_H__ //__ESP32_H__

#include <avr/io.h> //remove
#include <avr/interrupt.h> //remove
#include <avr/pgmspace.h> //remove
//#include "../MMC_SD.h" //remove
//#include <Arduino.h>

//extern uint16_t SectorsPerClust;
//extern uint16_t FirstDataSector;
//extern uint8_t FAT32_Enable;

//struct FileInfoStruct FileInfo;
//struct direntry PictureInfo;


#define CLR_CS     (PORTB &= ~(0x01)) //pin #8  - Chip Enable Signal //GPIO13
#define SET_CS     (PORTB |=  (0x01)) //pin #8  - Chip Enable Signal 
#define CLR_RS     (PORTB &= ~(0x02)) //pin #9  - Data/Instruction //?????? maybe GPIO12 (D/C)
#define SET_RS     (PORTB |=  (0x02)) //pin #9  - Data/Instruction
#define CLR_WR	   (PORTB &= ~(0x04)) //pin #10 - Read/Write //GPIO14
#define SET_WR	   (PORTB |=  (0x04)) //pin #10 - Read/Write
#define CLR_RD	   (PORTB &= ~(0x08)) //pin #11 - //GPIO27
#define SET_RD	   (PORTB |=  (0x08)) //pin #11 - 
#define CLR_RESET  (PORTB &= ~(0x10)) //pin #12 - Reset //GPIO23
#define SET_RESET  (PORTB |=  (0x10)) //pin #12 - Reset



extern void clearScreen(void);

void displayHome(void);
//void pictureSlideShow();
void writeColor(uint16_t color);
void writeCommand(uint8_t command);
void writeData(uint8_t data);
void clrCS(void);
void setCS(void);
void clrRS(void);
void setRS(void);
void clrWR(void);
void setWR(void);
void clrRD(void);
void setRD(void);
void clrRESET(void);
void setRESET(void);
void uPInit(char* interfaceType);
uint8_t readData(void);

#endif /* __ATMEGA2561_H__ */ //__ESP32_H__
