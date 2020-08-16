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

#define GROUP_NUMER 18
#define PAN_ID 0x1234
#define MY_ADDR 0x5678
#define SERVER_ADDR 0x1337

/*#define GROUP_NUMER 11
#define PAN_ID 0xCCCC
#define MY_ADDR 0x5678
#define SERVER_ADDR 0xABAB*/

NRK_STK TaskTransmit_Stack[NRK_APP_STACKSIZE];
nrk_task_type TaskTransmit;
void TaskTransmit_Handler(void);

void nrk_create_taskset();
void nrk_register_drivers();

//Make sure this is in both Master and Slave code:
typedef struct {
	uint16_t adxl_x, adxl_y, adxl_z; 
} adc_data_t;

uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[7];

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;


int main ()
{
	nrk_setup_ports();
	nrk_setup_uart(UART_BAUDRATE_115K2);

	printf( "Starting up...\r\n" );

	nrk_init();
	nrk_time_set(0,0);
	
	nrk_gpio_direction(NRK_DEBUG_0, NRK_PIN_INPUT);
	
	nrk_led_clr(ORANGE_LED);
	nrk_led_clr(BLUE_LED);
	nrk_led_clr(GREEN_LED);
	nrk_led_clr(RED_LED);
	
	nrk_register_drivers();
	
	//rtl_task_config();
	
	nrk_create_taskset ();
	nrk_start();

	return 0;
}


void TaskTransmit_Handler()
{
	uint8_t val, trigger_pressed = 0;
	int8_t fd;
	adc_data_t accel_data, accel_min, accel_max;
	nrk_sig_t uart_rx_signal;
	
	accel_min.adxl_x = 0xffff;
	accel_min.adxl_y = 0xffff;
	accel_min.adxl_z = 0xffff;
	
	accel_max.adxl_x = 0x0;
	accel_max.adxl_y = 0x0;
	accel_max.adxl_z = 0x0;
	
	// 18 = group number + 10 as channel
	// 1234 = panId, personal area netword identification number
	// 5678 = myAddr for this node
	
	rfRxInfo.pPayload = rx_buf;
	rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;

	rfTxInfo.destAddr = SERVER_ADDR;
	rfTxInfo.pPayload = tx_buf;
	rfTxInfo.length = 6;
	rfTxInfo.ackRequest = 0;
	
	rf_init(&rfRxInfo, GROUP_NUMER, PAN_ID, MY_ADDR);

	printf( "My node's address is %d\r\n",NODE_ADDR );
	printf( "TaskTransmit PID=%d\r\n",nrk_get_pid());
	
	fd=nrk_open(FIREFLY_SENSOR_BASIC,READ);
	if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));
	
	nrk_led_set(GREEN_LED);

	while(1) {
		nrk_led_set(ORANGE_LED);
		//Read ADC sensors
		val=nrk_set_status(fd,SENSOR_SELECT,ACC_X);
		val=nrk_read(fd,tx_buf,2);
		//sprintf (tx_buf, "acc_x: %d\t", adxl_x);

		val=nrk_set_status(fd,SENSOR_SELECT,ACC_Y);
		val=nrk_read(fd,tx_buf+2,2);
		//sprintf (tx_buf, "acc_y: %d\t", adxl_y);

		val=nrk_set_status(fd,SENSOR_SELECT,ACC_Z);
		val=nrk_read(fd,tx_buf+4,2);
		//sprintf (tx_buf, "acc_z: %d\t", adxl_z);
		
		memcpy(&accel_data, tx_buf, 6);
		
		if(nrk_gpio_get(NRK_DEBUG_2))
			trigger_pressed = 1;
		else
			trigger_pressed = 0;
		tx_buf[6] = trigger_pressed;
		
		nrk_led_clr(ORANGE_LED);

		if(accel_data.adxl_x < accel_min.adxl_x)
			accel_min.adxl_x = accel_data.adxl_x;
		if(accel_data.adxl_x > accel_max.adxl_x)
			accel_max.adxl_x = accel_data.adxl_x;
			
		if(accel_data.adxl_y < accel_min.adxl_y)
			accel_min.adxl_y = accel_data.adxl_y;
		if(accel_data.adxl_y > accel_max.adxl_y)
			accel_max.adxl_y = accel_data.adxl_y;
		
		if(accel_data.adxl_z < accel_min.adxl_z)
			accel_min.adxl_z = accel_data.adxl_z;
		if(accel_data.adxl_z > accel_max.adxl_z)
			accel_max.adxl_z = accel_data.adxl_z;
		// Now variables adxl_AXIS have the ADC values.
		
		printf("X: (%d,  %d,  %d), Y: (%d,  %d,  %d), Z: (%d,  %d,  %d), TRIG: %d",
			accel_min.adxl_x, accel_data.adxl_x, accel_max.adxl_x,
			accel_min.adxl_y, accel_data.adxl_y, accel_max.adxl_y,
			accel_min.adxl_z, accel_data.adxl_z, accel_max.adxl_z,
			trigger_pressed);

		// Construct packet
		
		//((adc_data_t)tx_buf) = accel_data;
		//tx_buf[0] = accel_data.adxl_x;
		//tx_buf[1] = accel_data.adxl_y;
		//tx_buf[2] = accel_data.adxl_z;
		// Send the packet
		nrk_led_set(BLUE_LED);
		
		nrk_kprintf(PSTR("\r\nTransmitting: "));
		for(val = 0; val < 7; val++) {
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
	val=nrk_register_driver( &dev_manager_ff_sensors,FIREFLY_SENSOR_BASIC);
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
