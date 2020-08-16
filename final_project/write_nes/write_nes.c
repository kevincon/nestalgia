/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Zane Starr
*******************************************************************************/


#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
//#include "nes_task_helper.h"

const uint8_t NIN_A = 0;
const uint8_t NIN_B = 1;
const uint8_t NIN_SELECT = 2;
const uint8_t NIN_START = 3;
const uint8_t NIN_UP = 4;
const uint8_t NIN_DOWN = 5;
const uint8_t NIN_LEFT = 6;
const uint8_t NIN_RIGHT = 7;

NRK_STK TaskInput_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskInput;
void TaskInput_Handler(void);

NRK_STK TaskWrite_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskWrite;
void TaskWrite_Handler(void);

void nrk_create_taskset();
void process_input(char, uint8_t *);
void dataout(uint8_t *);

nrk_time_t timestart;

uint8_t buttons[8];
uint8_t buttons_all_released[8];
uint8_t nin_pins[8];

int main ()
{
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

	printf( "Starting up...\r\n" );

	nrk_init();
	nrk_time_set(0,0);
	
	nrk_gpio_direction(NRK_DEBUG_0, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_DEBUG_1, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_DEBUG_2, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_DEBUG_3, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_1, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_3, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_5, NRK_PIN_OUTPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_7, NRK_PIN_OUTPUT);
	
	nin_pins[NIN_A] = NRK_DEBUG_0;
	nin_pins[NIN_B] = NRK_DEBUG_1;
	nin_pins[NIN_SELECT] = NRK_DEBUG_2;
	nin_pins[NIN_START] = NRK_DEBUG_3;
	nin_pins[NIN_UP] = NRK_ADC_INPUT_1;
	nin_pins[NIN_DOWN] = NRK_ADC_INPUT_3;
	nin_pins[NIN_LEFT] = NRK_ADC_INPUT_5;
	nin_pins[NIN_RIGHT] = NRK_ADC_INPUT_7;
	
	buttons_all_released[0] = 1;
	buttons_all_released[1] = 1;
	buttons_all_released[2] = 1;
	buttons_all_released[3] = 1;
	buttons_all_released[4] = 1;
	buttons_all_released[5] = 1;
	buttons_all_released[6] = 1;
	buttons_all_released[7] = 1;
	
	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(RED_LED);
	
	nrk_create_taskset ();
	nrk_start();

	return 0;
}


void TaskInput_Handler()
{
	int8_t fd;
	uint8_t counter = 0;
	char c = ' ';
	nrk_sig_t uart_rx_signal;

	printf( "My node's address is %d\r\n",NODE_ADDR );

	printf( "TaskInput PID=%d\r\n",nrk_get_pid());
	//nrk_gpio_set(NRK_DEBUG_0);
	
	// Get the signal for UART RX
	uart_rx_signal=nrk_uart_rx_signal_get();
	// Register task to wait on signal
	nrk_signal_register(uart_rx_signal); 
	
	buttons[0] = 1;
	buttons[1] = 1;
	buttons[2] = 1;
	buttons[3] = 0;
	buttons[4] = 1;
	buttons[5] = 1;
	buttons[6] = 1;
	buttons[7] = 1;

	while(1) {
		nrk_led_set(RED_LED);
		// This shows you how to wait until a key is pressed to start
		//nrk_kprintf( PSTR("Press THE ANY KEY to start\r\n" ));
		nrk_kprintf( PSTR("LOOPING\r\n" ));

		if(counter == 0) {
			c = ' ';
			do {
				if(nrk_uart_data_ready(NRK_DEFAULT_UART))
					c=getchar();
				else
					nrk_event_wait(SIG(uart_rx_signal));
				
			} while(c!='w' && c!='s' && c!='a' && c!='d' &&
					c!=0x9 && c!=0xD && c!='j' && c!='k');
		
			nrk_led_set(ORANGE_LED);
			process_input(c, buttons);
			nrk_led_clr(ORANGE_LED);
			
			counter = 1;
		}
		else {
			// Now that we've processed input, let's send our signals
			nrk_led_set(BLUE_LED);
			nrk_kprintf( PSTR("SENDING THE KEYPRESSES\r\n" ));		
			counter++;
			
			//if(counter < 2) {
				nrk_kprintf( PSTR("\tPRESSED\r\n" ));
				dataout(buttons);
			//}
			//if(counter >= 2) {
			//	nrk_kprintf( PSTR("\tRELEASED\r\n" ));
			//	dataout(buttons_all_released);
			//}
			//if(counter >= 4) {
			//	counter = 0;
			//}
			nrk_led_clr(BLUE_LED);
			
			if(counter >= 20) {
				counter = 0;
				dataout(buttons_all_released);
			}
		}
		
		nrk_led_clr(RED_LED);
		nrk_wait_until_next_period();
	}
	nrk_close(fd);
}

void dataout(uint8_t *buttons){
	uint8_t i;
	
	for(i = 0; i < 8; i++) {
		if(buttons[i])
			nrk_gpio_set(nin_pins[i]);
		else
			nrk_gpio_clr(nin_pins[i]);
	}
}

void process_input(char c, uint8_t *buttons) {
	uint8_t i;
	
	for(i = 0; i < 8; i++)
		buttons[i] = 1;
	
	switch(c) {
		case 0x4B: // K
		case 0x6B: // k
			buttons[NIN_A] = 0;
			break;
		case 0x4A: // J
		case 0x6A: // j
			buttons[NIN_B] = 0;
			break;
		case 0x9: // TAB
			buttons[NIN_SELECT] = 0;
			break;
		case 0xD: // ENTER
			buttons[NIN_START] = 0;
			break;
		case 0x57: // W
		case 0x77: // w
			buttons[NIN_UP] = 0;
			break;
		case 0x53: // S
		case 0x73: // s
			buttons[NIN_DOWN] = 0;
			break;
		case 0x41: // A
		case 0x61: // a
			buttons[NIN_LEFT] = 0;
			break;
		case 0x44: // D
		case 0x64: // d
			buttons[NIN_RIGHT] = 0;
			break;
	}
	
	printf("You pressed %c, which has a code of %x\r\n", c, c);
	
	nrk_kprintf( PSTR("BUTTONS: " ));
	for(i = 0; i < 8; i++) {
		printf("%d ", buttons[i]);
	}
	nrk_kprintf( PSTR("\r\n" ));
	
}


void nrk_create_taskset()
{
	nrk_task_set_entry_function( &TaskInput, TaskInput_Handler);
	nrk_task_set_stk( &TaskInput, TaskInput_Stack, NRK_APP_STACKSIZE);
	TaskInput.prio = 1;
	TaskInput.FirstActivation = TRUE;
	TaskInput.Type = BASIC_TASK;
	TaskInput.SchType = PREEMPTIVE;
	TaskInput.period.secs = 0;
	TaskInput.period.nano_secs = 100*NANOS_PER_MS;
	TaskInput.cpu_reserve.secs = 0;
	TaskInput.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
	TaskInput.offset.secs = 0;
	TaskInput.offset.nano_secs= 0;
	nrk_activate_task (&TaskInput);
}
