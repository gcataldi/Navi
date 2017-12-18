#include "Joy.h"

using namespace std;

// Função que abre o Joystick
int openJoystick()
{
    int joy_fd = 0;                             // File descriptor para o Joystick

    // Abre o joystick em Blocking Mode
    do
    {
        joy_fd = open (JOY_DEV, O_RDONLY);
        
        if ( joy_fd < 0 )
        {
            cout << "Cannot open joystick device... " << endl;
        }
        else
        {
            cout << "Joystick device connected... " << endl;
        }
    }
    while( joy_fd == 0 );

    // Não deixa a função read bloquear o código
    fcntl(joy_fd, F_SETFL, O_NONBLOCK); 
    
    return joy_fd;
}

// Função que fecha o arquivo de leitura do Joystick
int closeJoystick( int joy_fd )
{
    close (joy_fd);

    return 0;
}

// Função que mapeia os valores de um intervalo para outro 
double mapJoyValue(int x, int in_min, int in_max, double out_min, double out_max)
{
  return (double) ( (double)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
