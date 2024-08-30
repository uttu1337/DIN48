#include <otp.h>
#include <gpio.h>
#include <timer32.h>
#include "riscv_csr_encoding.h"
#include "scr1_csr_encoding.h"
#include "mcu32_memory_map.h"
#include <power_manager.h>
#include "pad_config.h"
#include <gpio_irq.h>
#include <epic.h>
#include <csr.h>
#include "uart_lib.h"
#include "xprintf.h"
#include "mik32_hal_rtc.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_gpio.h"
#include "DIN48.h"
#include <memory.h>

#define MAXSIZE 260
#define ROM_START 0x01000000 // базовый адрес EEPROM
#define ROM_SIZE  0x0000FA00 // размер EEPROM

uint8_t readbuf[MAXSIZE];
uint16_t writebuf[MAXSIZE];
uint32_t crc;
bool ready = false;

static void SystemClock_Config(unsigned long oscillator);

extern unsigned long __TEXT_START__;

void Error(void)
{
	while (1);
}

void trap_handler() {
	if ( EPIC->RAW_STATUS & (1<<EPIC_GPIO_IRQ_INDEX))
	{
		GPIO_0->OUTPUT ^= (0b1)<<(9);
		GPIO_IRQ->CLEAR = (1 << 7);
		EPIC->CLEAR = (1<<EPIC_GPIO_IRQ_INDEX);
	}
	else if (EPIC->RAW_STATUS & (1<<EPIC_BATTERY_NON_GOOD))
	{
		EPIC->CLEAR = (1<<EPIC_BATTERY_NON_GOOD);
	}
}

void main() {
	// interrupt vector init
	write_csr(mtvec, &__TEXT_START__);

	SystemClock_Config(PCC_OSCILLATORTYPE_OSC32K | PCC_OSCILLATORTYPE_OSC32M);

	PM->CLK_APB_P_SET =   PM_CLOCK_APB_P_GPIO_0_M
						| PM_CLOCK_APB_P_GPIO_1_M
						| PM_CLOCK_APB_P_GPIO_2_M
						| PM_CLOCK_APB_P_GPIO_IRQ_M
						| PM_CLOCK_APB_P_TIMER32_1_M
						;
	PM->CLK_APB_M_SET =   PM_CLOCK_APB_M_PAD_CONFIG_M
						| PM_CLOCK_APB_M_WU_M
						| PM_CLOCK_APB_M_PM_M
						| PM_CLOCK_APB_M_EPIC_M
						| PM_CLOCK_APB_M_RTC_M
						| PM_CLOCK_APB_M_OTP_CONTROLLER_M
						;
	PM->CLK_AHB_SET     |= 	  PM_CLOCK_AHB_SPIFI_M
			;

	// RS485 init
    UART_Init(UART_0, 138, UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_M_8BIT_M, 0, 0); // 138 - делитель для 230400 бод
    // VCP (terminal output)
    UART_Init(UART_1, 138, UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_M_8BIT_M, 0, 0); // 138 - делитель для 230400 бод

    xprintf("\r\n\r\nDIN48 Test App Started\r\n");


    // LEDs innit
	GPIO_0->DIRECTION_OUT =  1<<(9);
	GPIO_0->DIRECTION_OUT =  1<<(10);
	// RS485 management pins init
	GPIO_0->DIRECTION_OUT =  1<<(0); // DE
	GPIO_1->DIRECTION_OUT =  1<<(14);// RO

	// extint on user key setup begin
	GPIO_1->DIRECTION_IN =  1<<(15);

	// interrupt generation setup
	GPIO_IRQ->LINE_MUX = 3 << (7*4);	// 7th line <- Mode=3
	GPIO_IRQ->EDGE = 1 << 7;			// EDGE mode
	GPIO_IRQ->LEVEL_CLEAR = 1 << 7;		// falling edge
	GPIO_IRQ->ENABLE_SET = 1 << 7;		// enable 7th line

	// interrupt reception setup
	EPIC->MASK_LEVEL_SET = 1 << EPIC_GPIO_IRQ_INDEX;
	EPIC->MASK_LEVEL_CLEAR = 1 << EPIC_BATTERY_NON_GOOD;

	// global interrupt enable
	set_csr(mstatus, MSTATUS_MIE);
    set_csr(mie, MIE_MEIE);

	GPIO_0->CLEAR = (0b1)<<(9);
	xprintf("Green LED ON\r\n");

	xprintf("RS485 Transmit and receive enable\r\n");
	GPIO_0->SET =  1<<(0); // DE ON
	GPIO_1->CLEAR =  1<<(14);// RO ON

	xprintf("Red & Green LED OFF\r\n");
	GPIO_0->OUTPUT ^= (0b1)<<(10);
	GPIO_0->OUTPUT ^= (0b1)<<(9);
	void HAL_IRQ_EnableInterrupts();

	xprintf("Initiating CRC calculation\r\n");
	GPIO_0->OUTPUT ^= (0b1)<<(10);
	crc32_init();
	crc = crc32_byte(0,(void*)EEPROM_BASE_ADDRESS,EEPROM_SIZE);
	GPIO_0->OUTPUT ^= (0b1)<<(10);

	while (1)
	{
		/*GPIO_0->OUTPUT ^= (0b1)<<(10);

		for (volatile int i = 0; i < 10000; i++); // после чтения мигаем красным диодом
		GPIO_0->OUTPUT ^= (0b1)<<(10);*/


		if (readPrc(UART_0, readbuf)) { // чтение посылки)

			/*GPIO_0->OUTPUT ^= (0b1)<<(9);
			for (volatile int i = 0; i < 10000; i++); // после отправки мигаем зеленым диодом
			GPIO_0->OUTPUT ^= (0b1)<<(9);*/



		memset(writebuf, 0, (sizeof(writebuf)/sizeof(writebuf[0]) ));
		UART_Write(UART_0,	formPrc(UART_0, readbuf, writebuf, crc), writebuf[2]+5); // отправка посылки
		}

		ready = 0;
	}
}

static void SystemClock_Config(unsigned long oscillator)
{
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = oscillator;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 128;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_OSC32K;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

void xputc(char chr)
{
	UART_WriteByte(UART_0, chr);
	UART_WaitTransmission(UART_0);
}


