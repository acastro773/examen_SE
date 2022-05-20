/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "string.h"
#include "clock_config.h"
#include "lcd.h"
#include "ctype.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

// Switches
void bt1_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 //SW1_POS y SW2 son los puertos de los botones.
 PORTC->PCR[3] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[3] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[3] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void bt2_init() {
 SIM->COPC = 0;       // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;  // Conecta el reloj al puerto C

 PORTC->PCR[12] |= PORT_PCR_MUX(1); // Activa el GPIO
 PORTC->PCR[12] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
 PORTC->PCR[12] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

 // IRQ
 PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
 NVIC_SetPriority(31, 0); // Prioridad de la interrupcion 31
 NVIC_EnableIRQ(31);   // Activa la interrupcion
}

void led_green_init()
{
 SIM->COPC = 0;         // Desactiva el Watchdog
 SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;    // Conecta el reloj al puerto D
 PORTD->PCR[5] = PORT_PCR_MUX(1); // Configura los pines necesarios como GPIO
 GPIOD->PDDR |= (1 << 5);   // Se configura como pin de salida
 GPIOD->PSOR |= (1 << 5);   // Se pone a 1 el pin de salida
}

void led_green_on(void)
{
 GPIOD->PCOR |= (1 << 5);
}

void led_green_off(void) {
 GPIOD->PSOR |= (1 << 5);
}

void led_red_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void led_red_on(void)
{
 GPIOE->PCOR |= (1 << 29);
}

void led_red_off(void) {
 GPIOE->PSOR |= (1 << 29);
}

int check_led_green(void) {
 return (!(GPIOD->PCOR & (1 << 5)));
}

int check_led_red(void) {
 return (!(GPIOE->PCOR & (1 << 29)));
}

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
	if(pressed_switch == (0x8)) {
		if (check_led_green()) {
			led_green_off();
		}
		led_red_on();
		PRINTF("LED VERMELLO ON\r\n");
	}
	if(pressed_switch == (0x1000)) {
		if (check_led_red()) {
			led_red_off();
		}
		led_green_on();
		PRINTF("LED VERDE ON\r\n");
	}
}

/*!
 * @brief Main function
 */
int main(void)
{
  char str[10];
  int num = 0;
  int cont = 0;
  int length = 0;
  int blink = 0;
  int blred = 0;
  int blgreen = 0;

  irclk_ini(); // Enable internal ref clk to use by LCD
  lcd_ini();
  bt1_init();
  bt2_init();
  led_red_init();
  led_green_init();
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  lcd_display_dec(cont);

  PRINTF("\r\nReinicio!\r\n");

  while (1)
    {
	if (blink) {
		if (blred || blgreen) {
			if (blred)
				led_red_off();	
			if (blgreen)
				led_green_off();
			delay();
			if (blred)
				led_red_on();	
			if (blgreen)
				led_green_on();	
			delay();
		}	
	}
	num = 1;
	PRINTF("Escribe o comando:\r\n");
        SCANF("%s",str);
	PRINTF("Comando escrito: %s\r\n", str);
	length = strlen(str);
	for (int i = 0; i<length; i++) {
		if (!isdigit(str[i]) && num)
			num = 0;
	}		
	if (num) {
		cont = atoi(str);
		PRINTF("Num: %d\r\n", cont);
	} else {
		if (strcmp(str, "led1") == 0) {
			if (check_led_red()) {
				led_red_off();
			}
			led_green_on();
			PRINTF("LED VERDE ON\r\n");
		} else if (strcmp(str, "led2") == 0) {
			if (check_led_green()) {
				led_green_off();
			}
			led_red_on();
			PRINTF("LED VERMELLO ON\r\n");
		} else if (strcmp(str, "off") == 0) {
			if (check_led_green()) {
				led_green_off();
				PRINTF("LED VERDE OFF\r\n");
			}
			if (check_led_red()) {
				led_red_off();
				PRINTF("LED VERMELLO OFF\r\n");
			}
		} else if (strcmp(str, "blink") == 0) {
			if (blink) {
				if (check_led_red()) {
					blred = 1;
					PRINTF("Blink do led vermello:\r\n");
				}
				if (check_led_green()) {
					blgreen = 1;
					PRINTF("Blink do led verde:\r\n");
				}
			} else {
				blink = 0;
				if (blred) {
					blred = 0;
					led_red_on();
				}
				if (blgreen) {
					blgreen = 0;
					led_green_on();
				}
			}
		} else
			PRINTF("Comando non reconhecido\r\n");
	}
	lcd_display_dec(cont);
    }
}
