/// ----------------------------------------------
/// Joystick Header
/// ----------------------------------------------

#ifndef JOY_H
#define JOY_H


#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>

//-------------------------------------------------------------------------------------------------
// Define parâmetros do Joystick
//-------------------------------------------------------------------------------------------------

#define JS_EVENT_BUTTON         0x01            // Button pressed/released
#define JS_EVENT_AXIS           0x02            // Joystick moved
#define JS_EVENT_INIT           0x80            // Initial state of device

#define JOY_DEV "/dev/input/js0"				// Define o lugar onde o joystick está

struct js_event 
{
	unsigned int time;     		                // Event timestamp in milliseconds 
	short value;    			                // Value 
	unsigned char type;      	                // Event type 
	unsigned char number;    	                // Axis/button number 
};

using namespace std;

int openJoystick();                             									// Função que abre o joystick para leitura
int closeJoystick(int joy_fd);                  									// Função que fecha o joystick e encerra a leitura
double mapJoyValue(int x, int in_min, int in_max, double out_min, double out_max); 	// Função que mapeia os valores de um intervalo para outro 

#endif
