#pragma once
#include "uart_lib.h"
#include "xprintf.h"
#include <stdint.h>

/*
 *Считывает состояние пинов в 16 разрядных портах (P0, P1)
 *GPIO - указатель на структуру порта GPIO
 */

uint16_t readPins16(GPIO_TypeDef* GPIO);

/*
 *Считывает состояние пинов в 8 разрядных портах (P2)
 *GPIO - указатель на структуру порта GPIO
 */

uint16_t readPins8(GPIO_TypeDef* GPIO);

/*
 * Инициализация алгоритма crc32
 */
void crc32_init();

/*
 * Проверка контрольной суммы
 * buff - указатель на проверямую посылку
 * count - кол-во символов
 * crc - контрольная сумма
 */

unsigned char checkCrc(uint8_t * buff, unsigned char count,unsigned char crc);

/*
 * Вычисление crc32
 * init_crc - начальное значение crc
 * buf - указатель на буфер данных для которого вычисляем crc
 * len - кол-во данных (длина массива)
 */

uint32_t crc32_byte(uint32_t init_crc, uint8_t *buf, int len);

/* Расчёт контрольной суммы
 * buffer - указатель на, собственнно, саму посылку, откуда считать контрольную сумма
 * len - длина
 */

uint8_t crcCalc(uint8_t * buffer, uint16_t length);

/* Чтение посылки
 * uart - указатель для доступа к UART
 * buf - указатель на принимающий буфер
 * counter - счётчик
 */

bool readPrc(UART_TypeDef* uart, uint8_t* readbuf);

/* Обработка посылки
 * uart - указатель для доступа к UART
 * readbuf - указатель на массив для чтения
 * writebuf - указатель на массив для записи
 * crc - 32 битное значение контрольной суммы
 * Отправка производится функцией UART_Write из библиотеки
 */

uint16_t* formPrc(UART_TypeDef* uart, uint8_t* readbuf, uint16_t* writebuf, uint32_t crc);

