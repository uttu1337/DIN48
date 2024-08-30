/*
 * DIN48.c
 *
 *  Created on: 28 июн. 2024 г.
 *      Author: USER
 */
#include "DIN48.h"
#include "uart_lib.h"
#include "xprintf.h"
#include "mik32_hal_gpio.h"
#include <stdint.h>
#include <memory.h>
#include <stdlib.h>
#define GET_STATE 152 // 0x98
#define KAGDILA 0
#define SET_STATE 153 // 0x99
#define GET_VER  0x28  // Запрос версии
#define RCV_VER  0x29 // Ответ версии
#define GET_CRC  0x2C  //  Запрос КС
#define RCV_CRC  0x2D  // Ответ КС
#define CRC32_POLY   0x04C11DB7
#define CRC32_POLY_R 0xEDB88320
#define GPIO_A ((GPIO_TypeDef*)GPIO_0_BASE_ADDRESS )
#define GPIO_B ((GPIO_TypeDef*)GPIO_1_BASE_ADDRESS )
#define GPIO_C ((GPIO_TypeDef*)GPIO_2_BASE_ADDRESS )

GPIO_InitTypeDef GPIO_Init_A, GPIO_Init_B, GPIO_Init_C;

uint32_t vernum = 1010000; // Версия
static uint32_t crc32_table[256];
static uint32_t crc32r_table[256];
uint16_t test;

uint16_t readPins16(GPIO_TypeDef* GPIO) {
	uint16_t state = 0;
	for (int i = 0; i < 16; i++) {
		if (HAL_GPIO_ReadPin(GPIO, (1 << i)) == GPIO_PIN_HIGH) { // если в пине логическая единица
			state |= (1 << i); // то ставим 1
		}
		else state &= ~(1 << i); // иначе ставим 0
	}
	return state;
}

uint16_t readPins8(GPIO_TypeDef* GPIO) {
	uint16_t state = 0;
	for (int i = 0; i < 8; i++) {
		if (HAL_GPIO_ReadPin(GPIO, (1 << i)) == GPIO_PIN_HIGH) { // если в пине логическая единица
			state |= (1 << i); // то ставим 1
		}
		else state &= ~(1 << i); // иначе ставим 0
	}
	return state;
}

uint32_t crc32_byte(uint32_t init_crc, uint8_t *buf, int len)
{
        uint32_t v;
        uint32_t crc;
        crc = ~init_crc;
        while(len > 0) {
                v = *buf++;
                crc = ( crc >> 8 ) ^ crc32r_table[( crc ^ (v ) ) & 0xff];
                len --;
        }
        return ~crc;
}

void crc32_init(void) { // см. CRC32 IEEE 802.3
	int i, j;
	uint32_t c, cr;
	for (i = 0; i < 256; ++i)
	{
		cr = i;
		c = i << 24;
		for (j = 8; j > 0; --j)
		{
	    	c = c & 0x80000000 ? (c << 1) ^ CRC32_POLY : (c << 1);
	        cr = cr & 0x00000001 ? (cr >> 1) ^ CRC32_POLY_R : (cr >> 1);
		}
	   crc32_table[i] = c;
	   crc32r_table[i] = cr;
	}
}
uint8_t crcCalc(uint8_t * buffer, uint16_t len)
{
	uint8_t  i;
	uint8_t crc8=0;
    uint8_t temp8=0;

	for (i=0;i<len;i++)
	{

               temp8=buffer[i];
               crc8=crc8^temp8;
	}
	return crc8;

}

unsigned char checkCrc(uint8_t * buff, unsigned char count,unsigned char crc)
{

  unsigned char calcCrc;

  calcCrc=crcCalc(buff,count);
  if (crc==calcCrc)
  return 1;
  return 0;
}
volatile uint8_t cnt = 0;
bool readPrc(UART_TypeDef* uart, uint8_t* buf) {
	UART_ClearRxFifo(uart); // Предварительно очищаем RX
	UART_WaitReceiving(uart); // Ждём байт
	buf[cnt] = UART_ReadByte(uart);

	cnt +=1;
	if (buf[0] == 0xEF) { // проверка стартового байта
		if (cnt > 3) {

			if (cnt == buf[2] + 5) { // прочитали весь пакет
				if (checkCrc(buf, buf[2]+5-1, buf[buf[2]+5-1])) { // проверяем КС
					cnt = 0;
					return true; // сообщаем что пакет готов к отправке
				}
			}
		}
	}
	else cnt = 0;
	return false;
}

uint16_t* formPrc(UART_TypeDef* uart, uint8_t* readbuf, uint16_t* writebuf, uint32_t crc) {
	volatile int i = 0;
	volatile int j;
	size_t len;
	volatile uint8_t addr = 0xAA;
	volatile uint64_t din;
	uint8_t rs_crc_out = 0;		// обнуляем crc
	unsigned char version[50];
	uint16_t pins;
	uint16_t Vers = vernum / 1000000;
	uint16_t SubVers = vernum / 10000 - Vers * 100;
	uint8_t tmp = 4;
	char ver[4];
		switch (readbuf[3]) {
					case KAGDILA:	// DIN_KAGDILA
						writebuf[0] = 0xEF;
						writebuf[1] = addr;
						writebuf[3] = 0x00;
						writebuf[2] = 0;
						break;

					case GET_VER: // Версия ПО;
						len = sizeof(__DATE__) + sizeof(Vers) + sizeof (SubVers);
						writebuf[0] = 0xEF;
						writebuf[1] = addr;
						writebuf[2] = len;
						writebuf[3] = RCV_VER;
						writebuf[4] = Vers + '0';
						writebuf[5] = '.';
						writebuf[6] = SubVers + '0';
						writebuf[7] = ' ';
						for (i = 0; __DATE__[i] != '\0'; i++) {
							writebuf[i+8] = __DATE__[i];
						}
						break;

					case GET_CRC: // Запрос КС
						writebuf[0] = 0xEF;
						writebuf[1] = addr;
						writebuf[2] = 4;
						writebuf[3] = RCV_CRC;
						writebuf[4] = (crc & 0x000000ff); // запись 32 битного значения по байту
						writebuf[5] = (crc & 0x0000ff00) >> 8;
						writebuf[6] = (crc & 0x00ff0000) >> 16;
						writebuf[7] = (crc & 0xff000000) >> 24;
						break;

					case GET_STATE:	// DIN_GET_STATE
						writebuf[0] = 0xEF;
						writebuf[1] = addr;
						writebuf[2] = 6;
						writebuf[3] = 0x98;
						pins = readPins16(GPIO_A);
						writebuf[4] = (pins & 0x00ff);
						writebuf[5] = (pins & 0xff00) >> 8;
						pins = readPins16(GPIO_B);
						writebuf[6] = (pins & 0x00ff);
						writebuf[7] = (pins & 0xff00) >> 8;
						pins = readPins8(GPIO_C);
						writebuf[8] = (pins & 0xff);
						writebuf[9] = 0x00;
						break;

					default:
						break;
		}
		for (i=0;i<(writebuf[2]+4);i++) { // считаем crc
					rs_crc_out ^= writebuf[i];
				}
					writebuf[writebuf[2]+4] = rs_crc_out;
	return writebuf;

}
