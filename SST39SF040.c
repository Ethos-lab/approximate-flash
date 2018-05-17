/*
 * SST39SF040.c
 *
 *  Created on: Dec 9, 2015
 *      Author: amir
 */

#include "em_dac.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"

#include "SST39SF040.h"

#define NOP() __asm__ __volatile__ ("nop\n\t" ::)

#define DATA_MASK 0xFF

// Set the Flash data lines as outputs
#define flash_set_data_output()                       \
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 5, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0); \
  GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0)

// Set the Flash data lines as inputs
#define flash_set_data_input()                     \
  GPIO_PinModeSet(gpioPortD, 0, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 2, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 3, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 4, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 5, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 6, gpioModeInput, 0); \
  GPIO_PinModeSet(gpioPortD, 7, gpioModeInput, 0)

/* Put address in address line:
 * D8|D15-13|A14-12|E3-0|C7-C0
*/
#define set_flash_address(address)                                           \
  GPIO_PortOutSetVal(gpioPortC, (address) & 0xFF, 0xFF);                     \
  GPIO_PortOutSetVal(gpioPortE, ((address) >> 8) & 0xF, 0xF);                \
  GPIO_PortOutSetVal(gpioPortA, (((address) >> 12) & 0x7) << 12, 0x7 << 12); \
  GPIO_PortOutSetVal(gpioPortD, (((address) >> 15) & 0x7) << 13, 0x7 << 13); \
  GPIO_PortOutSetVal(gpioPortD, (((address) >> 18) & 0x1) << 8, 0x1 << 8)

#define FLASH_OE_OFFSET 10
#define set_flash_OE(val) GPIO_PortOutSetVal(gpioPortB, ((val) & 0x1) << FLASH_OE_OFFSET, 0x1 << FLASH_OE_OFFSET)

#define FLASH_CE_OFFSET 12
#define set_flash_CE(val) GPIO_PortOutSetVal(gpioPortB, ((val) & 0x1) << FLASH_CE_OFFSET, 0x1 << FLASH_CE_OFFSET)

#define FLASH_WE_OFFSET 9
#define set_flash_WE(val) GPIO_PortOutSetVal(gpioPortB, ((val) & 0x1) << FLASH_WE_OFFSET, 1 << FLASH_WE_OFFSET)

#define flash_start_state() set_flash_CE(1); set_flash_OE(1); set_flash_WE(1); set_flash_address(0x7FFFF); flash_set_data_input()

#define flash_internal_write(addr, data)            \
	set_flash_address(addr);                        \
	flash_delay(WRITE_DELAY); /* Erase fails without this */ \
	set_flash_WE(0);                                \
	GPIO_PortOutSetVal(gpioPortD, data, DATA_MASK); \
	NOP();                                          \
	set_flash_WE(1);                                \
	NOP();                                          \
	NOP()

void flash_delay(unsigned int ticks) {
	volatile unsigned int cnt = 0;
	while((++cnt) != ticks)
		;
}

void flash_write_lowV(uint32_t address, char data, float volts) {
	/* Change data line to output */
	  flash_set_data_output();

	  /* Just keep OE high */
	  set_flash_CE(0);

	  // SW0
	  flash_internal_write(0x5555, 0xAA);
	  flash_internal_write(0x2AAA, 0x55);
	  flash_internal_write(0x5555, 0xA0);

	  // Go to the supplied voltage and do the actual write
	  set_flash_voltage(volts);
	  flash_internal_write(address, data);

	  set_flash_CE(1);

	  // TBP = 20us ~ 1000 ticks
	  flash_delay(1000);

	  // Return to nominal voltage
	  set_flash_voltage(5.0);

	  // Force initial conditions
	  flash_start_state();
}

void flash_write(uint32_t address, char data){
  flash_write_lowV(address, data, 5.0);
}

/*
 * Erase a sector of 4K byte where address "address" is located.
 */
void flash_erase_sector(uint32_t address){
  flash_set_data_output();
  set_flash_CE(0);

  flash_internal_write(0x5555, 0xAA);
  flash_internal_write(0x2AAA, 0x55);
  flash_internal_write(0x5555, 0x80);
  flash_internal_write(0x5555, 0xAA);
  flash_internal_write(0x2AAA, 0x55);
  flash_internal_write(address, 0x30);

  set_flash_CE(1);

  // Wait 25ms ~1,250,000 ticks @ 48MHz
  flash_delay(1250000);

  // Force initial conditions
  flash_start_state();
}

void flash_erase_chip(void){
  flash_set_data_output();
  set_flash_CE(0);

  flash_internal_write(0x5555, 0xAA);
  flash_internal_write(0x2AAA, 0x55);
  flash_internal_write(0x5555, 0x80);
  flash_internal_write(0x5555, 0xAA);
  flash_internal_write(0x2AAA, 0x55);
  flash_internal_write(0x5555, 0x10);

  set_flash_CE(1);

  // Wait 100ms ~5,000,000 ticks @ 48MHz
  flash_delay(5000000);

  // Force initial conditions
  flash_start_state();
}



char flash_read(uint32_t address) {
  // Set pins connected to data lines to inputs
  flash_set_data_input();
  set_flash_address(address); // Wait 70ns before data ready
  set_flash_CE(0); // Wait 70ns before data ready
  set_flash_OE(0); // Wait 35ns before data ready

  // Assume worst case, 48MHz ~ 20ns per cycle (include setting OE)
  NOP();
  NOP();
  NOP();


  flash_delay(READ_DELAY); // First read fails without this
  uint32_t data = GPIO_PortInGet(gpioPortD);
  
  // Go back to start state
  set_flash_CE(1);
  set_flash_OE(1);

  return data & DATA_MASK;
}

void set_flash_voltage(float vin){
	DAC_Channel0OutputSet(DAC0, ((vin * 4096/2) / 3.3));
}

void flash_setup(void){
	/* Turn on the DAC clock.*/
	CMU_ClockEnable(cmuClock_DAC0, true);

	/* Enable GPIO in CMU */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* configure and enable the DAC. */
	DAC_setup();
	DAC_Enable(DAC0, 0, true);

	GPIO_DriveModeSet(gpioPortA, GPIO_P_CTRL_DRIVEMODE_HIGH);
	GPIO_DriveModeSet(gpioPortB, GPIO_P_CTRL_DRIVEMODE_HIGH);
	GPIO_DriveModeSet(gpioPortC, GPIO_P_CTRL_DRIVEMODE_HIGH);
	GPIO_DriveModeSet(gpioPortD, GPIO_P_CTRL_DRIVEMODE_HIGH);
	GPIO_DriveModeSet(gpioPortE, GPIO_P_CTRL_DRIVEMODE_HIGH);


	/* Configure PD0-7 as data line */
	flash_set_data_input();

	/* Configure address line:
	 * D8|D15-13|A14-12|E3-0|C9-C0
	 */
	for (int i=0;i<8;i++) {
		GPIO_PinModeSet(gpioPortC, i,gpioModePushPull , 0);
	}
	for (int i=0;i<4;i++) {
		GPIO_PinModeSet(gpioPortE, i,gpioModePushPull , 0);
	}
	for (int i=12;i<15;i++) {
		GPIO_PinModeSet(gpioPortA, i,gpioModePushPull , 0);
	}
	for (int i=13;i<16;i++){
			GPIO_PinModeSet(gpioPortD, i,gpioModePushPull , 0);
	}
	GPIO_PinModeSet(gpioPortD, 8 ,gpioModePushPull , 0);

	/* Configure control pins:
	 * B9  --> WE
	 * B10 --> OE
	 * B12 --> CE
	 * */
	GPIO_PinModeSet(gpioPortB, FLASH_WE_OFFSET, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortB, FLASH_OE_OFFSET, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortB, FLASH_CE_OFFSET, gpioModePushPull, 0);

	// Set all control signals to high
	// b'0001'0110'0000'0000
	//GPIO_PortOutSetVal(gpioPortB, 0x1600, 0x1600);
	flash_start_state();

}

void DAC_setup(void)
{
  /* Define DAC settings */
  DAC_Init_TypeDef init =
  {
    dacRefresh8,              /* Refresh every 8 prescaled cycles. */    \
    dacRef2V5,               /* 1.25V internal reference. */            \
    dacOutputPinADC,          /* Output to pin and ADC. */               \
    dacConvModeContinuous,    /* Continuous mode. */                     \
    0,                        /* No prescaling. */                       \
    false,                    /* Do not enable low pass filter. */       \
    false,                    /* Do not reset prescaler on ch0 start. */ \
    false,                    /* DAC output enable always on. */         \
    false,                    /* Disable sine mode. */                   \
    false                     /* Single ended mode. */                   \
  };

  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 500kHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the current value actually is. */
  init.prescale = DAC_PrescaleCalc(500000, 0);

  /* Set reference voltage to vdd.*/
  init.reference = dacRefVDD;

  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);

  /*Initialize DAC channel 0.*/
  DAC_InitChannel(DAC0, &initChannel, 0);
}


