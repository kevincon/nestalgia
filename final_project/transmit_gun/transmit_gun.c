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
#include <nrk_driver_list.h>	// FOR SENSOR BOARD / GYRO / GENERIC ADC
#include <nrk_driver.h>	// FOR SENSOR BOARD / GYRO / GENERIC ADC
//#include <ff_basic_sensor.h>	// FOR SENSOR BOARD
#include <adc_driver.h>	// FOR GYRO / GENERIC ADC
#include <basic_rf.h>
#include "../nestolgia_common.h"

uint8_t ACCEL_THRESH = 35; // 50 was pretty decent

NRK_STK TaskTransmit_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskTransmit;
void TaskTransmit_Handler(void);

void nrk_create_taskset();
void nrk_register_drivers();
RF_RX_INFO* handle_packet(RF_RX_INFO*);

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[7];

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

// 0 = done configuring, 1 through 99 = gathring average data
// 100 = should average and then set to 0
uint8_t adc_calib_count = 1;

int main ()
{
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

	printf( "Starting up...\r\n" );

	nrk_init();
	nrk_time_set(0,0);
	
	nrk_gpio_direction(NRK_DEBUG_0, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_1, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_3, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_5, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_7, NRK_PIN_OUTPUT);
	
	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(RED_LED);
	
	nrk_register_drivers();
	
	//rtl_task_config();
	
	rfRxInfo.pPayload = rx_buf;
	rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;

	rfTxInfo.destAddr = SERVER_ADDR;
	rfTxInfo.pPayload = tx_buf;
	rfTxInfo.length = 7;
	rfTxInfo.ackRequest = 0;
	
	rf_init(&rfRxInfo, GROUP_NUMER, PAN_ID, GUN_ADDR);
	
	nrk_create_taskset ();
	nrk_start();

	return 0;
}


void TaskTransmit_Handler()
{
	uint8_t val, trigger_pressed = 0;
	int8_t fd;
	adc_data_t gyro_data, gyro_data_avg;
	uint16_t packet_count = 0;
	//nrk_sig_t uart_rx_signal;
	
	adc_calib_count = 1;
	
	printf( "My node's address is %d\r\n", GUN_ADDR);
	printf( "TaskTransmit PID=%d\r\n",nrk_get_pid());
	
	// Open an instance of the ADC driver
	fd=nrk_open(ADC_DEV_MANAGER,READ);
    if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open adc driver\r\n"));
	
	// Make sure that setting up ADC didn't mess with our ADC pin directions
	nrk_gpio_direction(NRK_ADC_INPUT_1, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_3, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_5, NRK_PIN_INPUT);
	nrk_gpio_direction(NRK_ADC_INPUT_7, NRK_PIN_OUTPUT);
	
	nrk_gpio_clr(NRK_ADC_INPUT_7); // KEEP Auto Zero LOW!
	
	gyro_data_avg.adxl_x = 410;
	gyro_data_avg.adxl_y = 440;
	gyro_data_avg.adxl_z = 410;
	
	nrk_led_set(GREEN_LED);

	while(1) {
		nrk_led_set(ORANGE_LED);
		
		//Read Gyro X
		val=nrk_set_status(fd,ADC_CHAN,CHAN_1);
		val=nrk_read(fd,tx_buf,2);
		//sprintf (tx_buf, "acc_x: %d\t", adxl_x);

		//Read Gyro Y
		val=nrk_set_status(fd,ADC_CHAN,CHAN_3);
		val=nrk_read(fd,tx_buf+2,2);
		//sprintf (tx_buf, "acc_y: %d\t", adxl_y);

		//Read VRef
		val=nrk_set_status(fd,ADC_CHAN,CHAN_5);
		val=nrk_read(fd,tx_buf+4,2);
		//sprintf (tx_buf, "acc_z: %d\t", adxl_z);
		
		memcpy(&gyro_data, tx_buf, 6);
		
		nrk_led_clr(ORANGE_LED);
		
		if(adc_calib_count > 0 && adc_calib_count <= 10)
			adc_calib_count++;
		else if(adc_calib_count > 10) {
			adc_calib_count++;
			gyro_data_avg.adxl_x += (uint16_t)((int16_t)(gyro_data.adxl_x - gyro_data_avg.adxl_x) / (int16_t)(adc_calib_count - 10));
			gyro_data_avg.adxl_y += (uint16_t)((int16_t)(gyro_data.adxl_y - gyro_data_avg.adxl_y) / (int16_t)(adc_calib_count - 10));
			gyro_data_avg.adxl_z += (uint16_t)((int16_t)(gyro_data.adxl_z - gyro_data_avg.adxl_z) / (int16_t)(adc_calib_count - 10));
			
			if(adc_calib_count < 100) {
				printf("NEW AVG (count %d): X: %d, Y: %d, Z: %d\r\n",
					adc_calib_count, gyro_data_avg.adxl_x, gyro_data_avg.adxl_y, gyro_data_avg.adxl_z);
			}
			else if(adc_calib_count >= 100) {
				adc_calib_count = 0;
				nrk_led_clr(RED_LED);
				printf("===FINAL AVG: X: %d, Y: %d, Z: %d\r\n",  gyro_data_avg.adxl_x, gyro_data_avg.adxl_y, gyro_data_avg.adxl_z);
			}
		}
		else if(adc_calib_count == 0) {
			printf("%d = X: %d (%d, %d), Y: %d (%d, %d), Z: %d (%d, %d)\r\n",
				packet_count,
				gyro_data.adxl_x,
				gyro_data.adxl_x < (gyro_data_avg.adxl_x - ACCEL_THRESH),
				gyro_data.adxl_x > (gyro_data_avg.adxl_x + ACCEL_THRESH),
				gyro_data.adxl_y,
				gyro_data.adxl_y < (gyro_data_avg.adxl_y - ACCEL_THRESH),
				gyro_data.adxl_y > (gyro_data_avg.adxl_y + ACCEL_THRESH),
				gyro_data.adxl_z,
				gyro_data.adxl_z < (gyro_data_avg.adxl_z - ACCEL_THRESH),
				gyro_data.adxl_z > (gyro_data_avg.adxl_z + ACCEL_THRESH));
			
			tx_buf[6] = 0x0;
			if(gyro_data.adxl_x > gyro_data_avg.adxl_x + ACCEL_THRESH)
				tx_buf[6] |= POSITIVE_X;
			else if(gyro_data.adxl_x < gyro_data_avg.adxl_x - ACCEL_THRESH)
				tx_buf[6] |= NEGATIVE_X;
			if(gyro_data.adxl_y > gyro_data_avg.adxl_y + ACCEL_THRESH)
				tx_buf[6] |= POSITIVE_Y;
			else if(gyro_data.adxl_y < gyro_data_avg.adxl_y - ACCEL_THRESH)
				tx_buf[6] |= NEGATIVE_Y;
				
			// This doesn't really make sense on the gun, but no biggie :)
			// Can just ignore on receiver end
			if(gyro_data.adxl_z > gyro_data_avg.adxl_z + ACCEL_THRESH)
				tx_buf[6] |= POSITIVE_Z;
			else if(gyro_data.adxl_z < gyro_data_avg.adxl_z - ACCEL_THRESH)
				tx_buf[6] |= NEGATIVE_Z;
			
			if(nrk_gpio_get(NRK_DEBUG_0)) {
				trigger_pressed = 0;
			}
			else {
				trigger_pressed = 1;
				tx_buf[6] |= TRIGGER_PRESSED;
			}
				
			// Send the packet
			nrk_led_set(BLUE_LED);
			
			nrk_kprintf(PSTR("Transmitting: "));
			for(val = 0; val < 7; val++) {
				printf("%d ", tx_buf[val]);
			}
			nrk_kprintf(PSTR("\r\n"));
			
			rf_tx_packet(&rfTxInfo);
			packet_count++;
			nrk_led_clr(BLUE_LED);
		}

		printf("X: %d\tY: %d\tVRef: %d\tT: %d",
			gyro_data.adxl_x, gyro_data.adxl_y,
			gyro_data.adxl_z, trigger_pressed);

		// Send the packet
		nrk_led_set(BLUE_LED);
		
		nrk_kprintf(PSTR("\r\nTransmitting: "));
		for(val = 0; val < 8; val++) {
			printf("%d ", tx_buf[val]);
		}
		nrk_kprintf(PSTR("\r\n"));
		
		rf_tx_packet(&rfTxInfo);
		nrk_led_clr(BLUE_LED);		
		
		// Transmit the packet to the mole
		nrk_kprintf( PSTR("\r\n"));
		
		nrk_wait_until_next_period();
	}
	nrk_close(fd);
}

RF_RX_INFO* handle_packet(RF_RX_INFO* packet_info) {
	if(packet_info->srcAddr == SERVER_ADDR) {
		switch(packet_info->pPayload[0]) {
			case SET_THRESHOLD:
				ACCEL_THRESH = packet_info->pPayload[1];
				printf("*** NEW THRESHOLD: %d\r\n", ACCEL_THRESH);
				break;
			case RECALIBRATE:
				adc_calib_count = 1;
				nrk_kprintf(PSTR("*** RECALIBRATING\r\n"));
				break;
			default:
				nrk_kprintf(PSTR("*** UNRECOGNIZED SERVER COMMAND\r\n"));
				break;
		}
	}

	return packet_info;
}

void nrk_register_drivers()
{
	int8_t val;

	// Register the Basic FireFly Sensor device driver
	// Make sure to add: 
	//     #define NRK_MAX_DRIVER_CNT  
	//     in nrk_cfg.h
	// Make sure to add: 
	//     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
	//     in makefile
	//val=nrk_register_driver( &dev_manager_ff_sensors,FIREFLY_SENSOR_BASIC);
	//if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );
	
	
	// IMPORTANT!!! SEE http://www.nanork.org/wiki/adc-driver
	// Register the ADC device driver
	val=nrk_register_driver( &dev_manager_adc,ADC_DEV_MANAGER);
	if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );
}

void nrk_create_taskset()
{
	nrk_task_set_entry_function( &TaskTransmit, TaskTransmit_Handler);
	nrk_task_set_stk( &TaskTransmit, TaskTransmit_Stack, NRK_APP_STACKSIZE);
	TaskTransmit.prio = 1;
	TaskTransmit.FirstActivation = TRUE;
	TaskTransmit.Type = BASIC_TASK;
	TaskTransmit.SchType = PREEMPTIVE;
	TaskTransmit.period.secs = 0;
	TaskTransmit.period.nano_secs = 20*NANOS_PER_MS;
	TaskTransmit.cpu_reserve.secs = 0;
	TaskTransmit.cpu_reserve.nano_secs = 50*NANOS_PER_MS;
	TaskTransmit.offset.secs = 0;
	TaskTransmit.offset.nano_secs= 0;
	nrk_activate_task (&TaskTransmit);
}
