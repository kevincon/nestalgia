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


NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void TaskWrite(void);

void print_buttons(uint8_t* buttons) {
	uint8_t i;
	nrk_kprintf( PSTR("BUTTONS: "));
	
	for(i = 0; i < 8; i++) {
		switch(i) {
			case 0:
				if(buttons[0]) nrk_kprintf( PSTR("A "));
				break;
			case 1:
				if(buttons[1]) nrk_kprintf( PSTR("B "));
				break;
			case 2:
				if(buttons[2]) nrk_kprintf( PSTR("Select "));
				break;
			case 3:
				if(buttons[3]) nrk_kprintf( PSTR("Start "));
				break;
			case 4:
				if(buttons[4]) nrk_kprintf( PSTR("Up "));
				break;
			case 5:
				if(buttons[5]) nrk_kprintf( PSTR("Down "));
				break;
			case 6:
				if(buttons[6]) nrk_kprintf( PSTR("Left "));
				break;
			case 7:
				if(buttons[7]) nrk_kprintf( PSTR("Right "));
				break;
		}
	}
	nrk_kprintf( PSTR("\r\n"));
}

// NRK_STK Stack2[NRK_APP_STACKSIZE];
// nrk_task_type TaskTwo;
// void TaskRead(void);


void nrk_create_taskset();

nrk_time_t twelve_us;
nrk_time_t six_us;
nrk_time_t timestart;


uint8_t buttons[8];

//uint8_t kill_stack(uint8_t val);

int
main ()
{
	twelve_us.secs = 0;
	twelve_us.nano_secs = 12000;
	six_us.secs = 0;
	six_us.nano_secs = 6000;

	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

	printf( "Starting up...\r\n" );

	nrk_init();
	nrk_time_set(0,0);
	//Set GPIO Port "DEBUG_0" for latch output
	nrk_gpio_direction(NRK_DEBUG_0, NRK_PIN_OUTPUT);
	
	//Set GPIO Port "DEBUG_1" for pulse output
	nrk_gpio_direction(NRK_DEBUG_1, NRK_PIN_OUTPUT);

	//Set GPIO Port "DEBUG_2" for data input
	nrk_gpio_direction(NRK_DEBUG_2, NRK_PIN_INPUT);
	
	nrk_create_taskset ();
	nrk_start();

	return 0;
}


void TaskWrite()
{
	int8_t fd,val;
	//uint16_t buf;
	uint8_t i;

	printf( "My node's address is %d\r\n",NODE_ADDR );

	printf( "TaskWrite PID=%d\r\n",nrk_get_pid());
	//nrk_gpio_set(NRK_DEBUG_0);
	
	while(1) {
		//nrk_time_get(&timestart);
		nrk_led_toggle(BLUE_LED);
		//Set latch high
		val=nrk_gpio_set(NRK_DEBUG_0);
		
		//Wait 12 us
		nrk_wait(twelve_us);
		//nrk_spin_wait_us(12);
		
		//Clear latch pin
		val=nrk_gpio_clr(NRK_DEBUG_0);
		
		// Read "A" button
		buttons[0] = !nrk_gpio_get(NRK_DEBUG_2);
		
		// Wait 6 us
		nrk_wait(six_us);
		//nrk_spin_wait_us(6);
		
		// Send 8 high pulses on the pulse pin
		for(i = 1; i < 8; i++){
			// Set pulse high
			val=nrk_gpio_set(NRK_DEBUG_1);
			
			// Wait six us
			nrk_wait(six_us);
			//nrk_spin_wait_us(6);
			
			// Read button
			buttons[i] = !nrk_gpio_get(NRK_DEBUG_2);
			
			// Clear pulse low
			val=nrk_gpio_clr(NRK_DEBUG_1);
			
			// Wait six more us
			nrk_wait(six_us);
			//nrk_spin_wait_us(6);
		}
		
		print_buttons(buttons);

		nrk_wait_until_next_period();
	}
	nrk_close(fd);
}

void
nrk_create_taskset()
{
	nrk_task_set_entry_function( &TaskOne, TaskWrite);
	nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
	TaskOne.prio = 1;
	TaskOne.FirstActivation = TRUE;
	TaskOne.Type = BASIC_TASK;
	TaskOne.SchType = PREEMPTIVE;
	TaskOne.period.secs = 0;
	TaskOne.period.nano_secs = 16666666;
	TaskOne.cpu_reserve.secs = 0;
	TaskOne.cpu_reserve.nano_secs =  50*NANOS_PER_MS;
	TaskOne.offset.secs = 0;
	TaskOne.offset.nano_secs= 0;
	nrk_activate_task (&TaskOne);
	
	// nrk_task_set_entry_function( &TaskTwo, TaskRead);
	// nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
	// TaskTwo.prio = 2;
	// TaskTwo.FirstActivation = TRUE;
	// TaskTwo.Type = BASIC_TASK;
	// TaskTwo.SchType = PREEMPTIVE;
	// TaskTwo.period.secs = 0;
	// TaskTwo.period.nano_secs = 500*NANOS_PER_MS;
	// TaskTwo.cpu_reserve.secs = 0;
	// TaskTwo.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
	// TaskTwo.offset.secs = 0;
	// TaskTwo.offset.nano_secs= 0;
	// nrk_activate_task (&TaskTwo);

}
