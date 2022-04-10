
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hello_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include <termios.h>
#include <matrix/matrix/math.hpp>
#include <poll.h>
#include "attitude_send.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>
#include <px4_platform_common/px4_config.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arch/board/board.h>
#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/dog_mission_count.h>
#include <uORB/topics/dog_mission_item.h>
#include <uORB/topics/vehicle_gps_position.h>

px4::AppState AttitudeSend::appState;

#if 1
int AttitudeSend::set_baudrate(int *uart_fd, unsigned baud)
{
	/*
        int speed;

        switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
                PX4_ERR("ERR: unknown baudrate: %d", baud);
                return -EINVAL;
        }*/
        struct termios uart_config;
        int termios_state;

        tcgetattr(*uart_fd, &uart_config);
        uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                                 INLCR | PARMRK | INPCK | ISTRIP | IXON);
        uart_config.c_oflag = 0;
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

        /* no parity, one stop bit, disable flow control */
        uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

        /* set baud rate */
        if ((termios_state = cfsetispeed(&uart_config, baud)) < 0) {
                printf("ERR: %d (cfsetispeed)", termios_state);
                return -1;
        }
        if ((termios_state = cfsetospeed(&uart_config, baud)) < 0) {
                printf("ERR: %d (cfsetospeed)", termios_state);
                return -1;
        }
        if ((termios_state = tcsetattr(*uart_fd, TCSANOW, &uart_config)) < 0) {
                printf("ERR: %d (tcsetattr)", termios_state);
                return -1;
        }

        return 0;
}
#endif

void AttitudeSend::pack_float(uint8_t *buf, float val)
{
	int i = *(int *)&val;

	buf[0] = (i >> 0)  & 0xff;
	buf[1] = (i >> 8)  & 0xff;
	buf[2] = (i >> 16) & 0xff;
	buf[3] = (i >> 24) & 0xff;
}

void AttitudeSend::pack_double(uint8_t *buf, double val)
{
	unsigned long long i = *(unsigned long long  *)&val;

	buf[0] = (i >> 0)  & 0xff;
	buf[1] = (i >> 8)  & 0xff;
	buf[2] = (i >> 16) & 0xff;
	buf[3] = (i >> 24) & 0xff;
	buf[4] = (i >> 32) & 0xff;
	buf[5] = (i >> 40) & 0xff;
	buf[6] = (i >> 48) & 0xff;
	buf[7] = (i >> 56) & 0xff;
}

uint8_t AttitudeSend::pack_attitude_angular_velocity(uint8_t *buf, float pitch, float roll, float yaw, float x, float y, float z)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x01;	//attitude cmd
	buf[3] = 24; 	//len
	
	pack_float(&buf[4],  pitch);
	pack_float(&buf[8],  roll);
	pack_float(&buf[12], yaw);
	pack_float(&buf[16], x);
	pack_float(&buf[20], y);
	pack_float(&buf[24], z);
	
	for (i = 0; i < 28; i++)
	{
		sum += buf[i];
	}
	buf[28] = sum;
	buf[29] = 0x00;

	return 30;
}

uint8_t AttitudeSend::pack_attitude_angular_velocity(uint8_t *buf, float pitch, float roll, float yaw, float x, float y, float z,
						  float acc_x, float acc_y, float acc_z)
						 //uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch5, uint8_t ch8, uint8_t ch9)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x01;	//attitude cmd
	buf[3] = 36; 	//len 这个长度是表示发送的负载数据的长度，不包括包头以及校验
	
	pack_float(&buf[4],  pitch);
	pack_float(&buf[8],  roll);
	pack_float(&buf[12], yaw);
	pack_float(&buf[16], x);
	pack_float(&buf[20], y);
	pack_float(&buf[24], z);
	
	pack_float(&buf[28], acc_x);
	pack_float(&buf[32], acc_y);
	pack_float(&buf[36], acc_z);
	/*
	buf[40] = ch0;
	buf[41] = ch1;
	buf[42] = ch2;
	buf[43] = ch3;
	buf[44] = ch5;
	buf[45] = ch8;
	buf[46] = ch9;
	buf[47] = ch0;
	*/
	for (i = 0; i < 40; i++)
	{
		sum += buf[i];
	}
	buf[40] = sum;
	buf[41] = 0x00;

	return 42;
}

void AttitudeSend::pack_uint16(uint8_t *buf, uint16_t val)
{
	buf[0] = (val >> 0)  & 0xff;
	buf[1] = (val >> 8)  & 0xff;
}

void AttitudeSend::pack_rc(uint8_t *buf, uint16_t *values, uint8_t rc_lost)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x02;	//rc cmd
	buf[3] = 0x25; 	//len
	buf[4] = rc_lost;
	for (i = 0; i < 18; i++) {
		pack_uint16(&buf[5 + i*2], values[i]);
	}

	for (i = 0; i < 41; i++) {
		sum += buf[i];
	}

	buf[41] = sum;
	buf[42] = 0x00;
}

int AttitudeSend::pack_gps(uint8_t *buf, uint8_t sate, float eph, float epv, float lat, float lon, float alt)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x05;	//gps cmd
	buf[3] = 21; 	//len

	buf[4] = sate;
	pack_float(&buf[5], eph);
	pack_float(&buf[9], epv);
	pack_float(&buf[13], lat);
	pack_float(&buf[17], lon);
	pack_float(&buf[21], alt);
	
	for (i = 0; i < 24; i++) {
		sum += buf[i];
	}

	buf[25] = sum;
	buf[26] = 0x00;

	return 27;
}

void AttitudeSend::pack_mission_count(uint8_t *buf, uint16_t count)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x03;	//mission count cmd
	buf[3] = 0x02; 	//len
	pack_uint16(&buf[4], count);
	for (i = 0; i < 6; i++) {
		sum += buf[i];
	}
	buf[6] = sum;
	buf[7] = 0x00;
}


void AttitudeSend::pack_mission_item(uint8_t *buf, uint16_t seq, float speed, double longitude, double latitude)
{
	uint8_t i = 0;
	uint8_t sum = 0;

	buf[0] = 0xAA;
	buf[1] = 0xAF;
	buf[2] = 0x04;	//missin item cmd
	buf[3] = 0x16; 	//len 22
	
	pack_uint16(&buf[4], seq);
	pack_float(&buf[6], speed);
	pack_double(&buf[10], longitude);
	pack_double(&buf[18], latitude);

	for (i = 0; i < 26; i++) {
		sum += buf[i];
	}
	buf[26] = sum;
	buf[27] = 0x00;
}

int AttitudeSend::main(int argc, char **argv)
{
	appState.setRunning(true);
	//char const *uart_name = "/dev/ttyACM0";
	//char const *uart_name = "/dev/ttyS3";
	//uint64_t time_tick = 0;
	char uart_name[20] = "";
	uint8_t send_buf[50];
	
	if (strcmp("usb", argv[1]) == 0) {
		strcpy(uart_name, "/dev/ttyACM0");
		printf("send attitude by usb\r\n");
	} else if (strcmp("serial", argv[1]) == 0) {
		strcpy(uart_name, "/dev/ttyS3");
		printf("send attitude by serial\r\n");
	} else {
		strcpy(uart_name, "/dev/ttyS3");
		printf("send attitude by default serial\r\n");
	}
	int att_sub   			= orb_subscribe(ORB_ID(vehicle_attitude));
	int att_v_sub 			= orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int acc_sub   			= orb_subscribe(ORB_ID(vehicle_acceleration));
	//int rc_sub 			= orb_subscribe(ORB_ID(input_rc));
	int dog_mission_count_sub   	= orb_subscribe(ORB_ID(dog_mission_count));
	int dog_mission_item_sub   	= orb_subscribe(ORB_ID(dog_mission_item));
	int gps_pos_sub 		= orb_subscribe(ORB_ID(vehicle_gps_position));

	struct vehicle_angular_velocity_s 	att_v;
	struct vehicle_attitude_s 		att;
	struct vehicle_acceleration_s 		acc;
	//struct input_rc_s rc;
	struct dog_mission_count_s 		dog_mission_count;
	struct dog_mission_item_s 		dog_mission_item;
	struct vehicle_gps_position_s 		gps_pos;

	matrix::Eulerf attitude;
	bool updated;

	//uint8_t ch0 = 0, ch1 = 0, ch2 = 0, ch3 = 0, ch5 = 0, ch8 = 0, ch9 = 0;

	//int test_uart = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //
	int test_uart = open(uart_name, O_RDWR | O_NOCTTY); //
        if (test_uart < 0) {
                printf("ERROR opening UART %s, aborting..\n", uart_name);
                return test_uart;
        } else {
                printf("Writing to UART %s\n", uart_name);
        }
	if(fcntl(test_uart,F_SETFL,0) < 0) {   //阻塞，即使前面在open串口设备时设置的是非阻塞的，这里设为阻塞后，以此为准
        	printf("fcntl failed\n");
        }
	
	//set_baudrate(&test_uart, 460800);//bau is 115200
	//set_baudrate(&test_uart, 115200);//bau is 256000
	//set_baudrate(&test_uart, 256000);//bau is 256000
	//set_baudrate(&test_uart, 460800);//bau is 256000
	set_baudrate(&test_uart, 921600);//bau is 256000
	
	while (!appState.exitRequested()) {

        	orb_check(gps_pos_sub, &updated);
        	if (updated) {
                	orb_copy(ORB_ID(vehicle_gps_position), gps_pos_sub, &gps_pos);
			/*printf("gps pos:%10.7f %10.7f %10.7f %10.7f %10.7f %d\r\n", 	(double)(gps_pos.lat/10000000.0f), 
											(double)(gps_pos.lon/10000000), 
											(double)(gps_pos.alt/1000.0f), 
											(double)gps_pos.eph, 
											(double)gps_pos.epv,
											gps_pos.satellites_used);*/
			pack_gps(send_buf, gps_pos.satellites_used, gps_pos.eph, gps_pos.epv, (gps_pos.lat/10000000.0f), (gps_pos.lon/10000000), (gps_pos.alt/1000.0f));
			//write(test_uart, send_buf, 27);
		}

        	orb_check(att_sub, &updated);
        	if (updated) {
                	orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			attitude = matrix::Quatf(att.q);
			//printf("pitch:%10.7f, roll:%10.7f, yaw:%10.7f\r\n", (double)attitude.theta(), (double)attitude.phi(), (double)attitude.psi());
		}
		orb_check(att_v_sub, &updated);
        	if (updated) {
                	orb_copy(ORB_ID(vehicle_angular_velocity), att_v_sub, &att_v);
			//printf("timestamp:%llu\r\n",att_v.timestamp); 
			//printf("pitch_v:%f roll_v:%f yaw_v:%f\r\n", (double)att_v.xyz[1], (double)att_v.xyz[0], (double)att_v.xyz[2]);
		}
        	orb_check(acc_sub, &updated);
        	if (updated) {
                	orb_copy(ORB_ID(vehicle_acceleration), acc_sub, &acc);
			//printf("acc_x:%f acc_y:%f acc_z:%f\r\n", (double)acc.xyz[0], (double)acc.xyz[1], (double)acc.xyz[2]);
		}
		/*orb_check(rc_sub, &updated);
        	if (updated) {
			orb_copy(ORB_ID(input_rc), rc_sub, &rc);
			ch0 = rc.values[0]/10;
			ch1 = rc.values[1]/10;
			ch2 = rc.values[2]/10;
			ch3 = rc.values[3]/10;
			ch5 = rc.values[5]/10;
			ch8 = rc.values[8]/10;
			ch9 = rc.values[9]/10;
		}*/
		
		//send mission count
		orb_check(dog_mission_count_sub, &updated);
        	if (updated) {
			orb_copy(ORB_ID(dog_mission_count), dog_mission_count_sub, &dog_mission_count);
			//printf("mission_count:%d\r\n", dog_mission_count.wp_count);
			pack_mission_count(send_buf, dog_mission_count.wp_count);
			write(test_uart, send_buf, 8);
		}

		//send mission item
		orb_check(dog_mission_item_sub, &updated);
        	if (updated) {
			orb_copy(ORB_ID(dog_mission_item), dog_mission_item_sub, &dog_mission_item);
			//printf("seq:%d, mission_speed:%f %f %f\r\n", dog_mission_item.wp_seq, (double)dog_mission_item.speed, dog_mission_item.longitude, dog_mission_item.latitude);
			pack_mission_item(send_buf, dog_mission_item.wp_seq, dog_mission_item.speed, dog_mission_item.longitude, dog_mission_item.latitude);
			write(test_uart, send_buf, 27);
		}
		uint8_t send_bytes = 0;
		//send attitude 
		//send_bytes = pack_attitude_angular_velocity(send_buf, attitude.theta(), attitude.phi(), attitude.psi(), 
		//					      att_v.xyz[1],     att_v.xyz[0],   att_v.xyz[2]);
		send_bytes = pack_attitude_angular_velocity(send_buf, attitude.theta(), attitude.phi(), attitude.psi(), 
							    att_v.xyz[1],     att_v.xyz[0],   att_v.xyz[2],
							    acc.xyz[0],       acc.xyz[1],     acc.xyz[2]);
		//pack_attitude_angular_velocity(send_buf, attitude.theta(), attitude.phi(), attitude.psi(), 
		//					 att_v.xyz[1],     att_v.xyz[0],   att_v.xyz[2],
		//					 acc.xyz[0],       acc.xyz[1],     acc.xyz[2],
		//					 ch0, ch1, ch2, ch3, ch5, ch8, ch9);
		//write(test_uart, send_buf, 30);
		write(test_uart, send_buf, send_bytes);
		px4_usleep(5000);//delay 2.5ms
	}

	return 0;
}

