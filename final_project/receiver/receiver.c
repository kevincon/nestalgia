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
#include <basic_rf.h>
#include "../nestolgia_common.h"

NRK_STK TaskReceive_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskReceive;
void TaskReceive_Handler(void);

NRK_STK TaskInput_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskInput;
void TaskInput_Handler(void);

void nrk_create_taskset();
void dataout(uint8_t *);
//void nrk_register_drivers();
RF_RX_INFO* handle_packet(RF_RX_INFO*);
void initialize_all_nodes();
void display_presses(uint8_t);

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[7];

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

const uint8_t LEG_ACCEL_THRESH = 40;
const uint8_t GUN_ACCEL_THRESH = 15;

uint8_t buttons[8];
uint8_t buttons_all_released[8];
uint8_t nin_pins[8];

adc_data_t adc_gun, adc_gun_avg;
// 0 = done configuring, 1 through 99 = gathring average data
// 100 = should average and then set to 0
uint8_t adc_calib_count = 1;

uint8_t need_to_calibrate[3];
uint16_t packet_count[3];
uint8_t new_packet[3];

uint8_t gun_presses = 0;
uint8_t left_leg_presses = 0;
uint8_t right_leg_presses = 0;

uint8_t moving;
uint8_t move_counter = 0;


uint8_t press_without_packet_count = 0;

int main ()
{
	uint8_t i;
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
	
	nin_pins[NIN_A] = NRK_DEBUG_2;
	nin_pins[NIN_B] = NRK_DEBUG_1;	// POSSIBLY INCORRECT
	nin_pins[NIN_SELECT] = NRK_DEBUG_3;  // POSSIBLY INCORRECT
	nin_pins[NIN_START] = NRK_DEBUG_0;
	nin_pins[NIN_UP] = NRK_ADC_INPUT_1;
	nin_pins[NIN_DOWN] = NRK_ADC_INPUT_3;
	nin_pins[NIN_LEFT] = NRK_ADC_INPUT_5;
	nin_pins[NIN_RIGHT] = NRK_ADC_INPUT_7;
	
	adc_gun_avg.adxl_x = 419;
	adc_gun_avg.adxl_y = 444;
	adc_gun_avg.adxl_z = 419;
	
	for(i = 0; i < 8; i++)
		buttons_all_released[i] = 1;
	for(i = 0; i < 3; i++)
		packet_count[i] = 0;
		
	moving = MOVING_NONE;
	
	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(RED_LED);
	
	rfRxInfo.pPayload = rx_buf;
	rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;

	rf_init(&rfRxInfo, GROUP_NUMER, PAN_ID, SERVER_ADDR);
	rf_rx_on();
	set_rf_rx_callback(&handle_packet);
	
	nrk_create_taskset ();
	nrk_start();

	return 0;
}


void TaskReceive_Handler()
{
	printf( "My node's address is %d\r\n",SERVER_ADDR );
	printf( "TaskReceive PID=%d\r\n",nrk_get_pid());
	
	adc_calib_count = 1;	// Should start averaging stuffs
	initialize_all_nodes();

	while(1) {
		if(need_to_calibrate[GUN_ADDR - ADDR_START]) {
			tx_buf[0] = SET_THRESHOLD;
			tx_buf[1] = GUN_ACCEL_THRESH;
			
			rfTxInfo.destAddr = GUN_ADDR;
			rfTxInfo.pPayload = tx_buf;
			rfTxInfo.length = 2;
			rfTxInfo.ackRequest = 1; // TODO - Change me???
			
			if(rf_tx_packet(&rfTxInfo)) {
				nrk_kprintf(PSTR("*** Gun has ACKed new threshold\r\n"));
			}
			else {
				nrk_kprintf(PSTR("*** Gun has NOT ACKed new threshold\r\n"));
			}
		}
		if(need_to_calibrate[LEFT_LEG_ADDR - ADDR_START]) {
			tx_buf[0] = SET_THRESHOLD;
			tx_buf[1] = LEG_ACCEL_THRESH;
			
			rfTxInfo.destAddr = LEFT_LEG_ADDR;
			rfTxInfo.pPayload = tx_buf;
			rfTxInfo.length = 2;
			rfTxInfo.ackRequest = 1; // TODO - Change me???
			
			if(rf_tx_packet(&rfTxInfo)) {
				nrk_kprintf(PSTR("*** Left leg has ACKed new threshold\r\n"));
			}
			else {
				nrk_kprintf(PSTR("*** Left leg has NOT ACKed new threshold\r\n"));
			}
		}
		if(need_to_calibrate[RIGHT_LEG_ADDR - ADDR_START]) {
			tx_buf[0] = SET_THRESHOLD;
			tx_buf[1] = LEG_ACCEL_THRESH;
			
			rfTxInfo.destAddr = RIGHT_LEG_ADDR;
			rfTxInfo.pPayload = tx_buf;
			rfTxInfo.length = 2;
			rfTxInfo.ackRequest = 0; // TODO - Change me???
			
			if(rf_tx_packet(&rfTxInfo)) {
				nrk_kprintf(PSTR("*** Right leg has ACKed new threshold\r\n"));
			}
			else {
				nrk_kprintf(PSTR("*** Right leg has NOT ACKed new threshold\r\n"));
			}
		}
		if(new_packet[GUN_ADDR - ADDR_START]) {
			nrk_led_set(BLUE_LED);
			new_packet[GUN_ADDR - ADDR_START] = 0;
			press_without_packet_count = 0;
			
			printf("GUN (%d): ", packet_count[GUN_ADDR - ADDR_START]);
			display_presses(gun_presses);
			nrk_kprintf( PSTR("\r\n"));
			
			nrk_led_clr(BLUE_LED);
		}
		if(new_packet[LEFT_LEG_ADDR - ADDR_START]) {
			new_packet[LEFT_LEG_ADDR - ADDR_START] = 0;
			// TODO HANDLE LEFT LEG PRESSES HERE
			
			printf("LEFT LEG (%d): ", packet_count[LEFT_LEG_ADDR - ADDR_START]);
			display_presses(left_leg_presses);
			nrk_kprintf(PSTR("\r\n"));
		}
		if(new_packet[RIGHT_LEG_ADDR - ADDR_START]) {
			new_packet[RIGHT_LEG_ADDR - ADDR_START] = 0;
			// TODO HANDLE RIGHT LEG PRESSES HERE
			
			printf("RIGHT LEG (%d): ", packet_count[RIGHT_LEG_ADDR - ADDR_START]);
			display_presses(right_leg_presses);
			nrk_kprintf(PSTR("\r\n"));
		}
		
		nrk_wait_until_next_period();
	}
}

void display_presses(uint8_t press) {
	(press & NEGATIVE_X) ? nrk_kprintf(PSTR("-X ")) : nrk_kprintf(PSTR("   "));
	(press & POSITIVE_X) ? nrk_kprintf(PSTR("+X ")) : nrk_kprintf(PSTR("   "));
	(press & NEGATIVE_Y) ? nrk_kprintf(PSTR("-Y ")) : nrk_kprintf(PSTR("   "));
	(press & POSITIVE_Y) ? nrk_kprintf(PSTR("+Y ")) : nrk_kprintf(PSTR("   "));
	(press & NEGATIVE_Z) ? nrk_kprintf(PSTR("-Z ")) : nrk_kprintf(PSTR("   "));
	(press & POSITIVE_Z) ? nrk_kprintf(PSTR("+Z ")) : nrk_kprintf(PSTR("   "));
	(press & TRIGGER_PRESSED) ? nrk_kprintf(PSTR("T ")) : nrk_kprintf(PSTR("  "));
}

RF_RX_INFO* handle_packet(RF_RX_INFO* packet_info) {
	//nrk_kprintf( PSTR("===HANDLING PACKET===\r\n"));
	// TODO: OPTIMIZE!
	if(packet_info->srcAddr == GUN_ADDR) {
		//memcpy(&adc_gun, packet_info->pPayload, 6);
		gun_presses = packet_info->pPayload[6];
		new_packet[GUN_ADDR - ADDR_START] = 1;
		packet_count[GUN_ADDR - ADDR_START]++;
	}
	else if(packet_info->srcAddr == LEFT_LEG_ADDR) {
		left_leg_presses = packet_info->pPayload[6];
		new_packet[LEFT_LEG_ADDR - ADDR_START] = 1;
		packet_count[LEFT_LEG_ADDR - ADDR_START]++;
	}
	else if(packet_info->srcAddr == RIGHT_LEG_ADDR) {
		right_leg_presses = packet_info->pPayload[6];
		new_packet[RIGHT_LEG_ADDR - ADDR_START] = 1;
		packet_count[RIGHT_LEG_ADDR - ADDR_START]++;
	}

	return packet_info;
}

void TaskInput_Handler()
{
	uint8_t i;
	nrk_sig_t uart_rx_signal;

	printf( "TaskInput PID=%d\r\n",nrk_get_pid());
	
	// Get the signal for UART RX
	uart_rx_signal=nrk_uart_rx_signal_get();
	// Register task to wait on signal
	nrk_signal_register(uart_rx_signal);
	
	for(i = 0; i < 8; i++)
		buttons[i] = 1;
	
	while (adc_calib_count > 0)
		nrk_wait_until_next_period();

	while(1) {
		if(move_counter > 0) move_counter++;
		if(move_counter >= MOVE_THRESH) { move_counter = 0; moving = MOVING_NONE; }
		
		if(press_without_packet_count++ >= 10) {
			while(press_without_packet_count >= 10)
				nrk_wait_until_next_period();
		}
			
	
		nrk_kprintf(PSTR("== PRESS: "));
		
		if(gun_presses & TRIGGER_PRESSED) {
			buttons[NIN_A] = 0; nrk_kprintf(PSTR("T "));
			
			if(adc_gun.adxl_x > (adc_gun_avg.adxl_x + ACCEL_THRESH)) {
				buttons[NIN_RIGHT] = 0; nrk_kprintf(PSTR("R "));
			}
			else {
				buttons[NIN_RIGHT] = 1; nrk_kprintf(PSTR("  "));
			}
			
			if(adc_gun.adxl_x < (adc_gun_avg.adxl_x - ACCEL_THRESH)) {
				buttons[NIN_LEFT] = 0; nrk_kprintf(PSTR("L "));
			}
			else {
				buttons[NIN_LEFT] = 1; nrk_kprintf(PSTR("  "));
			}
			
			if(adc_gun.adxl_y < (adc_gun_avg.adxl_y - ACCEL_THRESH)) {
				buttons[NIN_UP] = 0; nrk_kprintf(PSTR("U "));
			}
			else {
				buttons[NIN_UP] = 1; nrk_kprintf(PSTR("  "));
			}
			
			if(adc_gun.adxl_y > (adc_gun_avg.adxl_y + ACCEL_THRESH)) {
				buttons[NIN_DOWN] = 0; nrk_kprintf(PSTR("D "));
			}
			else {
				buttons[NIN_DOWN] = 1; nrk_kprintf(PSTR("  "));
			}
		}
		else {
			buttons[NIN_A] = 1; nrk_kprintf(PSTR("  "));
			
			if(moving == MOVING_RIGHT) {
				buttons[NIN_RIGHT] = 0; nrk_kprintf(PSTR("R "));
			}
			else {
				buttons[NIN_RIGHT] = 1; nrk_kprintf(PSTR("  "));
			}
			
			if(moving == MOVING_LEFT) {
				buttons[NIN_LEFT] = 0; nrk_kprintf(PSTR("L "));
			}
			else {
				buttons[NIN_LEFT] = 1; nrk_kprintf(PSTR("  "));
			}
			
			buttons[NIN_UP] = 1;
			buttons[NIN_DOWN] = 1;
		}
		
		//nrk_kprintf(PSTR("\r\n"));
	
		nrk_led_set(GREEN_LED);
		dataout(buttons);
		nrk_led_clr(GREEN_LED);
		
		nrk_wait_until_next_period();
	}
}

void dataout(uint8_t *buttons){
	uint8_t i;
	
	for(i = 0; i < 8; i++) {
		buttons[i] ? nrk_gpio_set(nin_pins[i]) : nrk_gpio_clr(nin_pins[i]);
	}
}

void initialize_all_nodes() {
	need_to_calibrate[GUN_ADDR - ADDR_START] = 1;
	need_to_calibrate[LEFT_LEG_ADDR - ADDR_START] = 1;
	need_to_calibrate[RIGHT_LEG_ADDR - ADDR_START] = 1;
}

void nrk_create_taskset()
{
	nrk_task_set_entry_function( &TaskReceive, TaskReceive_Handler);
	nrk_task_set_stk( &TaskReceive, TaskReceive_Stack, NRK_APP_STACKSIZE);
	TaskReceive.prio = 1;
	TaskReceive.FirstActivation = TRUE;
	TaskReceive.Type = BASIC_TASK;
	TaskReceive.SchType = PREEMPTIVE;
	TaskReceive.period.secs = 0;
	TaskReceive.period.nano_secs = 20*NANOS_PER_MS;
	TaskReceive.cpu_reserve.secs = 0;
	TaskReceive.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
	TaskReceive.offset.secs = 0;
	TaskReceive.offset.nano_secs= 0;
	nrk_activate_task (&TaskReceive);
	
	nrk_task_set_entry_function( &TaskInput, TaskInput_Handler);
	nrk_task_set_stk( &TaskInput, TaskInput_Stack, NRK_APP_STACKSIZE);
	TaskInput.prio = 2;
	TaskInput.FirstActivation = TRUE;
	TaskInput.Type = BASIC_TASK;
	TaskInput.SchType = PREEMPTIVE;
	TaskInput.period.secs = 0;
	TaskInput.period.nano_secs = 20*NANOS_PER_MS;
	TaskInput.cpu_reserve.secs = 0;
	TaskInput.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
	TaskInput.offset.secs = 0;
	TaskInput.offset.nano_secs= 0;
	nrk_activate_task (&TaskInput);
	
}
