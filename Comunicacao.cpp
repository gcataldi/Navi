/// ----------------------------------------------
/// Classe Comunicação SPI
/// ----------------------------------------------

#include "Comunicacao.h"

// --------------------------------------------------------------- //
// Função que inicia a comunicação SPI
// Retorna o file descriptor correspondente da conexão SPI 
// --------------------------------------------------------------- //

unsigned int spi_speed 	 = 	200000;			// 2Mhz
unsigned int spi_mode	 = 	0;				// Modo de comunicação 0
unsigned int spi_bits	 =  8;				// 8 bits por palavra

int IniciaSPI() 
{	
	int aux = 0;
	
	// Setup SPI
	// Open file spidevB.C, where B is the numer of the device and C is the Chip Select (CS)
	// Open file spidev0.0 (chip enable 0) for read/write access
	
	int spi_fd = 0;
		
	spi_fd = open( SPI_DEVICE , O_RDWR);  
	
	// Testa se o dispositivo SPI foi aberto
	if ( spi_fd < 0 )
	{
		cout << " Can't open the SPI Device... " << endl;
		return -1;			
	}
	
	// Configura a variável de velocidade
	// Passa um ponteiro com a variável spi_speed
	aux = ioctl (spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
	if (aux == -1)
	{
		cout << " Can't set SPI Speed... "	<< endl;
		return -1;	
	}
	
	// Configura o número de bits por palavra
	aux = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
	if (aux == -1)
	{
		cout << " Can't set SPI Bits per Word... "	<< endl;
		return -1;	
	}
		
	// Configura o modo de conexão SPI
	aux = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
	if (aux == -1)
	{
		cout << " Can't set SPI Mode... "	<< endl;
		return -1;	
	}

	cout << " Device is now ready to use SPI... " << endl;
	
	return spi_fd;
}

// --------------------------------------------------------------- //
// Função que finaliza a comunicação SPI
// --------------------------------------------------------------- //

void FinalizaSPI(int spi_fd) 
{
	close(spi_fd);
	return;
}

// --------------------------------------------------------------- //
// Função que transmite um byte via SPI e retorna um byte
// como resultado.
//
// Estabelece uma estrutura de dados, spi_ioc_transfer como
// definido pela spidev.h e carrega os vários membros para
// passar os dados e parâmetros de configuração para o dispositivo
// SPI via IOCTL
//
// Variáveis locais txDat e rxDat são definidas e passadas
// como referência 
// --------------------------------------------------------------- //

int spiTxRx(unsigned char txDat, int spi_fd)
{
	unsigned char rxDat;
  
	// O driver spidev estabele uma estrutura de dados spi_ioc_transfer
	struct spi_ioc_transfer spi;
	
	// Aloca memória para a estrutura
	memset (&spi, 0, sizeof (spi));

	// Carregar a estrutura com três informações
	// .tx_buf - a pointer to a variable holding the data to be transferred
	// .rx_buf - a pointer to a variable to hold received data
	// .len - length in bytes of tx and rx buffers
	spi.tx_buf        = (unsigned long)&txDat;
	spi.rx_buf        = (unsigned long)&rxDat;
	spi.len           = 1;							// Single Byte Transfer, could be more

	// Chamar ioctl e passar a estrutura SPI para o arquivo virtual
	ioctl (spi_fd, SPI_IOC_MESSAGE(1), &spi);

	// txDat byte será transferido para o arduino e o byte retornado 
	// estará em rxDat
	return rxDat;
}



// --------------------------------------------------------------- //
// Protocolo que usa a função spiTxRx para enviar um comando
// formatado para o Arduino, um byte de cada vez, pegando assim
// a resposta   
// --------------------------------------------------------------- //

//-----------------------------------------------------------------------------------------
// Função que envia dados via SPI para o Arduino referente aos atuadores
// SPI SEND COMMAND WRITE ACTUATORS ARGUMENTS 
// 1) COMMAND = 'm'
// 2) OP = 			  1  || 2  || 3  || 4
// 3) CONTROL VALUE = Pd || Pe || Sd || Se
// 4) SPI_FD = SPI file descriptor
int spiSendCommandWriteActuators( char command, int op, int controlValue, int spi_fd )
{
	unsigned char resultByte;
	bool handshake;	
	
	// Unions allow variables to occupy the same memory space
	// a convenient way to move back and forth between 8-bit and
	// 16-bit values etc.
		
	union p1Buffer_T
	{
		int p1Int;
		unsigned char p1Char[2];
	} p1Buffer;
			
	p1Buffer.p1Int = controlValue;

	union p2Buffer_T
	{
		int p2Int;
		unsigned char p2Char[2];
	} p2Buffer;
			
	p2Buffer.p2Int = op;
	
	// Envia um comando de Handshake para o Arduino e enquanto ele não
	// responder com o comando certo fica preso no laço
	do
	{
		handshake = false;

		// Envia 1 byte "c" como comando de Handshake para o Arduino
		spiTxRx('c', spi_fd);

		// Recebe o comando do Arduino em resultByte e ao 
		// mesmo tempo envia o comando da operação para o Arduino
		resultByte = spiTxRx(command, spi_fd);

		// Se a resposta do arduino for "a", concluímos o handshake
		if (resultByte == 'a')
		{
			handshake = true;
		}

	}
	while (handshake == false);	
	
	// Send the parameters one byte at a time.
	
	// Envia valor da operação
	spiTxRx(p2Buffer.p2Char[0], spi_fd);

	spiTxRx(p2Buffer.p2Char[1], spi_fd);

	// Envia valor do controle
	spiTxRx(p1Buffer.p1Char[0], spi_fd);

	spiTxRx(p1Buffer.p1Char[1], spi_fd);

	//Push more zeros through 

	spiTxRx( 0, spi_fd );

	spiTxRx( 0, spi_fd );

	return 1;
}
//-----------------------------------------------------------------------------------------

// Função que envia comando para a leitura de dados do Arduino via SPI e retorna o float lido
// ARGUMENTOS
// 1) GIROSCÓPIO = 'g', 'x'/'y'/'z';
// 2) ACELEÔMETRO = 'a', 'x'/'y'/'z';
// 3) SONAR = 's', '-';
// 4) BATERIA = 'b', 'p'/'n';						// p = positive/ n = negative => Alarme de Emergência
// 5) COMPASSO (HEADING) = 'h', '-';			
float spiSendCommandReadData( char command, char op, int spi_fd )
{
	unsigned char resultByte;
	unsigned char valueSign;
	bool handshake;

	float dataRead = 0.0;							// Valor a ser lido

	union p1Buffer_T
	{
		int p1Int;
		unsigned char p1Char[2];
	} p1Buffer;

	// Inicializa a variável
	p1Buffer.p1Int = 0;

	// Envia um comando de Handshake para o Arduino e enquanto ele não
	// responder com o comando certo fica preso no laço
	do
	{
		handshake = false;

		// Envia 1 byte "c" como comando de Handshake para o Arduino
		spiTxRx('c', spi_fd);												// BYTE[0]

		// Recebe o comando do Arduino em resultByte e ao 
		// mesmo tempo envia o comando da operação para o Arduino
		resultByte = spiTxRx(command, spi_fd);								// BYTE[1]

		// Se a resposta do arduino for "a", concluímos o handshake
		if (resultByte == 'a')
		{
			handshake = true;
		}

	}
	while (handshake == false);

	spiTxRx(op, spi_fd);								// Manda o subcomando // BYTE[2]

	spiTxRx(0, spi_fd);									// BYTE[3]

	spiTxRx(0, spi_fd);									// Pega o valor do sinal // BYTE[4]
	
	spiTxRx(0, spi_fd);									// Pega 2 bytes do número // BYTE[5]
	
	spiTxRx(0, spi_fd);									// BYTE[6]
	
	spiTxRx(0, spi_fd);									// BYTE[7]

	// Envia um comando de Handshake para o Arduino e enquanto ele não
	// responder com o comando certo fica preso no laço
	do
	{
		handshake = false;

		// Envia 1 byte "c" como comando de Handshake para o Arduino
		spiTxRx('c', spi_fd);												// BYTE[0]

		// Recebe o comando do Arduino em resultByte e ao 
		// mesmo tempo envia o comando da operação para o Arduino
		resultByte = spiTxRx(command, spi_fd);								// BYTE[1]

		// Se a resposta do arduino for "a", concluímos o handshake
		if (resultByte == 'a')
		{
			handshake = true;
		}

	}
	while (handshake == false);

	spiTxRx(op, spi_fd);								// Manda o subcomando // BYTE[2]

	spiTxRx(0, spi_fd);									// BYTE[3]

	valueSign = spiTxRx(0, spi_fd);						// Pega o valor do sinal // BYTE[4]

	resultByte = spiTxRx(0, spi_fd);					// Pega 2 bytes do número // BYTE[5]
	p1Buffer.p1Char[0] = resultByte;

	resultByte = spiTxRx(0, spi_fd);					// BYTE[6]
	p1Buffer.p1Char[1] = resultByte;

	spiTxRx(0, spi_fd);									// BYTE[7]

	if ( valueSign == 'n' )
	{
		if ( (command == 's') || (command == 'b') )
		{
			dataRead = (float) (-1.0) * ( (float) p1Buffer.p1Int )/(100.0);
		}
		else if ( (command == 'g') || (command == 'a') )
		{
			dataRead = (float) (-1.0) * ( (float) p1Buffer.p1Int )/(100.0);

		}

		return dataRead;
	}
	else
	{
		if ( (command == 's') || (command == 'b') )
		{
			dataRead = (float) ( (float) p1Buffer.p1Int )/(100.0);
		}
		else if ( (command == 'g') || (command == 'a') )
		{
			dataRead = (float) ( (float) p1Buffer.p1Int )/(100.0);
		}
		return dataRead;
	}	
}

// Função que envia dados via SPI do joystick para o Arduino e recebe a resposta
int spiSendCommandJoystick(char command, char valueSign, int joyValue, char op, int spi_fd)
{
	unsigned char resultByte;
	bool handshake;
	
	
	// Unions allow variables to occupy the same memory space
	// a convenient way to move back and forth between 8-bit and
	// 16-bit values etc.
		
	union p1Buffer_T
	{
		int p1Int;
		unsigned char p1Char[2];
	} p1Buffer;
			
	p1Buffer.p1Int = joyValue;
	
	// Envia um comando de Handshake para o Arduino e enquanto ele não
	// responder com o comando certo fica preso no laço
	do
	{
		handshake = false;

		// Envia 1 byte "c" como comando de Handshake para o Arduino
		spiTxRx('c', spi_fd);
		usleep (5);									// Dorme por 5 microssegundos esperando o envio e a resposta

		// Recebe o comando do Arduino em resultByte e ao 
		// mesmo tempo envia o comando da operação para o Arduino
		resultByte = spiTxRx(command, spi_fd);

		// Se a resposta do arduino for "a", concluímos o handshake
		if (resultByte == 'a')
		{
			handshake = true;
		}
		usleep (5);  									// Dorme por 5 microssegundos esperando o envio e a resposta

	}
	while (handshake == false);	

	// Send the parameters one byte at a time.

	spiTxRx(valueSign, spi_fd);
	usleep(5);

	spiTxRx(p1Buffer.p1Char[0], spi_fd);
	usleep(5);

	spiTxRx(p1Buffer.p1Char[1], spi_fd);
	usleep(5);

	spiTxRx(op, spi_fd);
	usleep(5);

	// Push two more zeros through 
	spiTxRx( 0, spi_fd );
	usleep(5);

	spiTxRx( 0, spi_fd );
	usleep(5);

	return 1;
}


	
