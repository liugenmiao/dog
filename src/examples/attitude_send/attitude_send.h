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
 * @file hello_example.h
 * Example app for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */
#pragma once

#include <px4_platform_common/app.h>

class AttitudeSend
{
public:
	AttitudeSend() {}
	~AttitudeSend() {}

	int main(int argc, char **argv);
	void pack_float(uint8_t *buf, float val);
	uint8_t pack_attitude_angular_velocity(uint8_t *buf, float pitch, float roll, float yaw, float x, float y, float z);
	uint8_t pack_attitude_angular_velocity(uint8_t *buf, float pitch, float roll, float yaw, float x, float y, float z, float acc_x, float acc_y, float acc_z);
	//void pack_attitude_angular_velocity(uint8_t *buf, float pitch, float roll, float yaw, float x, float y, float z, float acc_x, float acc_y, float acc_z,
	//					 uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch5, uint8_t ch8, uint8_t ch9);
	int set_baudrate(int *uart_fd, unsigned baud);
	
	void pack_uint16(uint8_t *buf, uint16_t val);
	void pack_rc(uint8_t *buf, uint16_t *values, uint8_t rc_lost);


	void pack_double(uint8_t *buf, double val);
	void pack_mission_count(uint8_t *buf, uint16_t count);
	void pack_mission_item(uint8_t *buf, uint16_t seq, float speed, double longitude, double latitude);
	int pack_gps(uint8_t *buf, uint8_t sate, float eph, float epv, float lat, float lon, float alt);

	static px4::AppState appState; /* track requests to terminate app */

private:
	//int _serial_fd;

};


