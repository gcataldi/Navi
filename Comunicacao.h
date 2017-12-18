/// ----------------------------------------------
/// Comunicação Header
/// ----------------------------------------------

#ifndef COMU_H
#define COMU_H

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include <unistd.h>

// Parâmetros da Comunicação SPI
#define SPI_DEVICE "/dev/spidev0.0"

using namespace std;

int IniciaSPI();
int spiTxRx(unsigned char txDat, int spi_fd);
int spiSendCommandWriteActuators( char command, int op, int controlValue, int spi_fd );
int spiSendCommandJoystick(char command, char valueSign, int joyValue, char op, int spi_fd);
float spiSendCommandReadData( char command, char op, int spi_fd );
void FinalizaSPI(int spi_fd); 

#endif
