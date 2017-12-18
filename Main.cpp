///////////////////////////////////////////////////////////
// PROJETO BIRROTOR                                 ///////
// Trabalho de Graduação de Engenharia Mecatrônica  ///////
// Autor: Gabriel Luiz Pedreira Cataldi             ///////
// Matrícula: 10/0102085                            ///////
///////////////////////////////////////////////////////////

/// ----------------------------------------------
/// Main do Programa
/// ----------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include <vector>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>

#include <chrono>
#include <functional>

#include <sys/ioctl.h>
#include <fcntl.h>                              // Header files to File control options
#include <cstring>                              // Header files to manipulate strings and arrays
#include <cstdio>                               // Header files to perform Input/Output operations
#include <cstdint>                              // Header files to define a set of integral type aliases

#include <cmath>
#include <errno.h>
#include <termios.h>

#include "Comunicacao.h"
#include "Datalogger.h"
#include "Joy.h"

using namespace std;

#define MODE_JOY                        1       // Modo de Controle para o Joystick
#define MODE_CLOSED_LOOP_JOY_P          1       // Modo de Controle Malha Aberta Proporcional para o Joystick
#define MODE_CLOSED_LOOP                1       // Modo de Controle Malha Fechada 
#define MODE_CLOSED_LOOP_PD             1       // Modo de Controle Malha Fechada Proporcinal + Derivativo

#define MODE_ANGLE_TO_POWER             0       // Modo em que a potência dos motores varia de acordo com a posição dos servos P/cos(theta/2)

#define GYRO_FILTER                     1       // Ativa o filtro do giroscópio 
#define GYRO_LB_FILTER                  1       // Ativa o filtro que bloqueia entradas muito altas no giroscópio 
#define SONAR_FILTRO                    1       // Ativa o filtro e offset do sonar

#define GYRO_UP_LIMIT                   100.0
#define GYRO_DOWN_LIMIT                 -100.0

#define LOW_BATTERY_LEVEL               10.8    // Tensão correspondente à bateria fraca, 3.6V por célula

#define SENSORS_DPS_TO_RAD  (0.017453293F)      // Converte degrees/s to rad/s

//-------------------------------------------------------------------------------------------------
// Ângulo Estáveis de Decolagem
//-------------------------------------------------------------------------------------------------

#define anguloEsqDec    89.0                    // Valores de hardware correspondentes aos 90 graus de decolagem
#define anguloDirDec    86.0

//-------------------------------------------------------------------------------------------------
// Parâmetros de Controle para o Joystick
//-------------------------------------------------------------------------------------------------

#define K1              71.5
#define K2              71.5

// EIXO X
#define K_X0              10

// EIXO Y
#define KD_Y0             90
#define KD_Y1             -45
#define KE_Y0             90
#define KE_Y1             45

// EIXO Z
#define K_Z0             90
#define K_Z1             45

//-------------------------------------------------------------------------------------------------
// Offsets provenientes da calibração dos equipamentos
//-------------------------------------------------------------------------------------------------

// Servos
#define OFFSET_SD         4                     // Correção para os valores do Servo Direito
#define OFFSET_SE         1                     // Correção para os valores do Servo Esquerdo

// Giroscópio
#define OFFSET_GX         1.15                // Correção de valores do giroscópio com o drone parado - EIXO X
#define OFFSET_GY         1.52                  // Correção de valores do giroscópio com o drone parado - EIXO Y
#define OFFSET_GZ         -0.3206               // Correção de valores do giroscópio com o drone parado - EIXO Z

// Sonar
#define OFFSET_SONAR      -3.36                  // Correção de valores do sonar com o drone parado - Altura

//-------------------------------------------------------------------------------------------------
// Define o período das tarefas
//-------------------------------------------------------------------------------------------------

#define PERIOD_CONTROL_TASK             50      // 50ms
#define PERIOD_BATTERY_TASK             5000   // 10s

//-------------------------------------------------------------------------------------------------
// Declaração das THREADS
//-------------------------------------------------------------------------------------------------

void threadJoystick();                          // Thread que lê o joystick
void threadControl();                           // Thread responsavel pelo controle 
//void threadGPS();                               // Thread que realiza a medição do GPS

//-------------------------------------------------------------------------------------------------
// Declaração de Funções
//-------------------------------------------------------------------------------------------------

double getRealTime();                           // Função lê o tempo atual com base no CLOCK_REALTIME
bool medeBateria();                             // Função que controla a periodicidade da leitura da bateria

//-------------------------------------------------------------------------------------------------
// Declaração de Variáveis Globais
//-------------------------------------------------------------------------------------------------

// Eixos do Joystick
volatile double X_j = 0.0;                      // Eixo X do Joystick (ROLL/ROLAGEM)
volatile double Y_j = 0.0;                      // Eixo Y do Joystick (PITCH/ARFAGEM)
volatile double Z_j = 0.0;                      // Eixo Z do Joystick (YAW/GUINADA)
volatile double A_j = -1.0;                     // Eixo A do Joystick (THROTTLE/ACELERAÇÃO)

// Flag que indica que o joystick está ativado
bool flagJoystick = false;

// Flags de Emergência
bool flagBattery = false;                       // Flag para indicar que deve ser realizada medição da bateria, ativa em um período de PERIOD_BATTERY_TASK
bool modoEmergencia = false;                    // Flag que indica que a bateria está fraca e deve ser acionado modo de emergência
bool botaoEmergencia = false;                   // Flag que indica que o botão de emergencia para desligar os motores foi acionado

// EXIT APPLICATION
bool exitApplication = false;                   // Indica o comando para sair do programa

// VARIÁVEL DE TEMPO GLOBAL
volatile double T = 0.0;

//-------------------------------------------------------------------------------------------------
// Main do Programa
//-------------------------------------------------------------------------------------------------

// Main
int main( int argc, char* argv[])
{
    double tempoInicioMain = 0.0;               // Marcador de Tempo do Início do Programa
    double tempoFinalMain = 0.0;                // Marcador de Tempo do Final do Programa

    tempoInicioMain = getRealTime();            // Lê o tempo atual com base no CLOCK_REALTIME

    // Cria e inicializa as threads
    std::thread first ( threadControl );
    std::thread second ( threadJoystick );
    //std::thread third ( threadGPS );

    //-------------------------------------------------------------------------------------------------
    // Loop infinito que segura o programa para manter a execução das threads e impede que ele termine
    //-------------------------------------------------------------------------------------------------
    do
    {
    }
    while ( exitApplication == false );

    // Encerra as Threads e Sincroniza
    first.join();
    second.join();
    //third.join();

    // Faz a diferença entre o tempo lido no início e o tempo atual
    tempoFinalMain = getRealTime() - tempoInicioMain;
    
    //cout << tempoFinalMain << endl;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------
// THREADS
//-------------------------------------------------------------------------------------------------

// Thread com as tarefas relacionadas ao controle
void threadControl()
{
    // Tempo inicial global
    double startTime = 0.0;                         // Tempo inicial de execução da thread

    // Variação do ângulo em relação ao centro (90 graus)
    double deltaTheta_D = 0.0;
    double deltaTheta_E = 0.0;
    
    // Tempo da tarefa de Controle
    double taskStartTime = 0.0;                     // Tempo que marca o início da tarefa de Controle
    double taskNextStartTime = 0.0;                 // Tempo que marca o próximo início da tarefa de Controle
    double deltaTime = 0.0;                         // Tempo decorrido na execução da tarefa de Controle

    // Bateria
    int acionaAlarme = 0;                           // Contador para ativar alarme de bateria fraca, se igual a 1 o alarme deve ser acionado
    double V_BAT = 0.0;                             // Valor de tensão lido na bateria
    double batteryTime = 0.0;                       // Valor que controla a periodicidade da leitura da bateria
    double batteryStartTime = 0.0;                  // Valor que pega o início da tarefa de bateria
    int batteryFirstTime = 0;                       // Marca o ínicio da tarefa, primeira vez que passa na tarefa da Bateria e marca o ínicio do período

    // Giroscópio
    double gyroX = 0.0;                             // Leitura do valor do giroscópio no eixo X
    double gyroY = 0.0;                             // Leitura do valor do giroscópio no eixo Y
    double gyroZ = 0.0;                             // Leitura do valor do giroscópio no eixo Z

    // Sonar
    double sonarAltura = 0.0;                       // Leitura da altura pelo sonar

    // Atuadores
    volatile double Pd = 0.0;                       // Potência/Velocidade do motor direito     (0 a 180)
    volatile double Pe = 0.0;                       // Potência/Velocidade do motor esquerdo    (0 a 180)
    volatile double Sd = anguloDirDec;              // Posição do servo motor direito           (0 a 180)
    volatile double Se = anguloEsqDec;              // Posição do servo motor esquerdo          (0 a 180)    

    volatile double Sd_Desired = 0.0;               // Posição desejada para o servo direito
    volatile double Se_Desired = 0.0;               // Posição desejada para o servo esquerdo
    volatile double Pd_Desired = 0.0;               // Potência desejada para o motor direito
    volatile double Pe_Desired = 0.0;               // Potência desejada para o motor esquerdo
    
    volatile double Pd_Old = 0.0;                   // Valor antigo de Pd (t-1)
    volatile double Pe_Old = 0.0;                   // Valor antigo de Pe (t-1)
    volatile double Sd_Old = 0.0;                   // Valor antigo de Sd (t-1)
    volatile double Se_Old = 0.0;                   // Valor antigo de Se (t-1)

    // Sensor Giroscópio
    volatile double gyroX_Old = 0.0;                // Valor antigo do gyro em X (t-1)
    volatile double gyroY_Old = 0.0;                // Valor antigo do gyro em Y (t-1)
    volatile double gyroZ_Old = 0.0;                // Valor antigo do gyro em Z (t-1)

    bool flagFirstTimeControl = true;               // Sinaliza que é a primeira vez que entra no algoritmo de controle de malha fechada
    bool flagFirstTimeControl_P = true;             // Sinaliza que é a primeira vez que entra no algoritmo de controle proporcional do Joystick

    // SPI DEVICE
    int spi_fd = 0;                                 // SPI FIle descriptor
    bool flagSPI = false;                           // Sinaliza que a conexão SPI iniciou com sucesso                     

    //-------------------------------------------------------------------------------------------------
    // Open SPI Communication
    //-------------------------------------------------------------------------------------------------
    
    spi_fd = IniciaSPI();                           // Inicia comunicação SPI

    if ( spi_fd < 0 )                               // Sinaliza se a comunicação SPI foi realizada com sucesso
    {
        flagSPI = false;
    }
    else
    {
        flagSPI = true;
    }

    //-------------------------------------------------------------------------------------------------
    // Datalogger
    //-------------------------------------------------------------------------------------------------
    
    // Inicia o Datalogger
    initDatalogger();

    // Insere os valores iniciais das variáveis
    dataloggerUpdate( T , gyroX, gyroY, gyroZ, sonarAltura, V_BAT, deltaTime, Sd, Se, Pd, Pe, X_j, Y_j, Z_j, A_j);        

    // Pega o valor inicial do tempo quando começa a thread
    startTime = getRealTime();

    // Pega o valor do tempo de execução atual -> Inicializa a variável T, que será contada a partir daqui
    T = (getRealTime() - startTime);

    //-------------------------------------------------------------------------------------------------
    // Laço Principal
    //-------------------------------------------------------------------------------------------------
    
    // Fica preso aqui até ser pressionado o botão de encerrar o programa
    while( exitApplication == false )
    {
        // Calcula tempo de início da tarefa
        taskStartTime = getRealTime();

        //-------------------------------------------------------------------------------------------------
        // LEITURA DOS SENSORES
        //-------------------------------------------------------------------------------------------------

        // Leitura do Giroscópio em graus/s 
        gyroX = spiSendCommandReadData( 'g', 'x', spi_fd )/SENSORS_DPS_TO_RAD;             
        gyroY = spiSendCommandReadData( 'g', 'y', spi_fd )/SENSORS_DPS_TO_RAD;
        gyroZ = spiSendCommandReadData( 'g', 'z', spi_fd )/SENSORS_DPS_TO_RAD;

        // CALIBRAÇÃO E FILTRO DO GIROSCÓPIO
        if ( GYRO_FILTER == 1 )
        {
            // Calibração para a Leitura do Giroscópio ( OFFSET MEDIDO COM BASE NA CALIBRAÇÃO FEITA COM O DRONE PARADO )
            gyroX = gyroX + OFFSET_GX;
            gyroY = gyroY + OFFSET_GY;
            gyroZ = gyroZ + OFFSET_GZ;

            // Filtro que elimina ruídos quando o drone está "parado" ou "pairando"
            if ( (gyroX > -2.5) && (gyroX < 2.5)  )
            {
                gyroX = 0.0;
            }

            if ( (gyroY > -2.5) && (gyroY < 2.5)  )
            {
                gyroY = 0.0;
            }
            
            if ( (gyroZ > -2.5) && (gyroZ < 2.5)  )
            {
                gyroZ = 0.0;
            }

            // Filtro que bloqueia entradas muito altas do giroscópio 
            // SATURADOR DO GIROSCÓPIO
            if ( GYRO_LB_FILTER == 1 )
            {
                if ( gyroX > GYRO_UP_LIMIT )
                {
                    gyroX = GYRO_UP_LIMIT;   
                }
                else if ( gyroX < GYRO_DOWN_LIMIT )
                {
                    gyroX = GYRO_DOWN_LIMIT;
                }

                if ( gyroY > GYRO_UP_LIMIT )
                {
                    gyroY = GYRO_UP_LIMIT;   
                }
                else if ( gyroY < GYRO_DOWN_LIMIT )
                {
                    gyroY = GYRO_DOWN_LIMIT;
                }

                if ( gyroZ > GYRO_UP_LIMIT )
                {
                    gyroZ = GYRO_UP_LIMIT;   
                }
                else if ( gyroZ < GYRO_DOWN_LIMIT )
                {
                    gyroZ = GYRO_DOWN_LIMIT;
                }
            }
        }

        // Leitura do Sonar
        sonarAltura = spiSendCommandReadData( 's', '-', spi_fd );

        // FILTRO SONAR
        if ( SONAR_FILTRO == 1 )
        {
            sonarAltura = sonarAltura + OFFSET_SONAR;

            if ( sonarAltura < 0 )
            {
                sonarAltura = 0.0;
            }
        }

        //-------------------------------------------------------------------------------------------------
        // LEITURA DA BATERIA (TAREFA PERIÓDICA)
        //-------------------------------------------------------------------------------------------------

        // Verifica se é a primeira vez do período
        if ( batteryFirstTime == 0 )
        {
            batteryStartTime = (getRealTime() - startTime);        // Valor de tempo T que iniciou a tarefa, sempre T = (getRealTime() - startTime)

            batteryFirstTime++;                                    // Incrementa o contador
        }

        // Pega o valor do tempo de execução atual
        batteryTime = (getRealTime() - startTime) ;

        // Ativa ou não a execução da tarefa
        if ( (batteryFirstTime != 0) && ( (batteryTime - batteryStartTime) >= PERIOD_BATTERY_TASK ) )       // Ativa se o tempo de execução da tarefa for maior ou igual ao período indicado de espera
        {
            flagBattery = true;
            batteryFirstTime = 0;
        }
        else
        {
            flagBattery = false;
        }

        // Realiza a leitura da tensão na Bateria se a flag estiver ativada
        if ( flagBattery == true )
        {
            // Faz a leitura da tensão na bateria via SPI
            V_BAT = spiSendCommandReadData( 'b', 'n', spi_fd );

            // Verifica se a bateria estiver fraca
            if ( (V_BAT < LOW_BATTERY_LEVEL) && ( V_BAT != 0 ) )
            {
                acionaAlarme++;

                // O contador deve ser maior que 5 para acionar o modo de emergência para evitar que ruídos atrapalhem
                if ( acionaAlarme >= 5 )
                {
                    modoEmergencia = true;                                  // Ativa modo de emergência bateria fraca

                    V_BAT = spiSendCommandReadData( 'b', 'p', spi_fd );     // Ativa modo de emergência bateria fraca
                }
                else
                {
                    modoEmergencia = false;
                }
            }
            else
            {
                acionaAlarme = 0;
            }

            flagBattery = false;
        }

        //-------------------------------------------------------------------------------------------------
        // CONTROLE
        //-------------------------------------------------------------------------------------------------

        // Controle usando o joystick
        if ( (MODE_JOY == 1) && (flagJoystick == true) )                                       // Ativa apenas se for pressionado o botão correspondente do joystick
        {

            // Controle em Malha Aberta usando o joystick (DIRETO)
            if ( MODE_CLOSED_LOOP_JOY_P == 0 )                
            {
                // THROTTLE -----------------------------------------------------------------------------------
                if ( A_j != -1.0 )
                {
                    // Seta o valor da potência dos motores
                    Pd = ( K2 * A_j + K1 );
                    Pe = ( K2 * A_j + K1 );
                }
                else if ( A_j == -1.0 )
                {
                    Pd = 0;
                    Pe = 0;
                }

                // EIXO Z -------------------------------------------------------------------------------------
                if ( ((Y_j <= 0.15) || (Y_j >= -0.15)) && ((Z_j >= 0.52) || (Z_j <= 0.01)) )        // YAW
                {
                    if ( Z_j > 0.52 )                                                               // Compensação para aliviar o defeito do joystick, só de apoiar a mão os valores dão um salto
                    {
                        Sd = 93.75 * Z_j + 41.25 - OFFSET_SD;
                        Se = 93.75 * Z_j + 41.25 - OFFSET_SE;                        
                    }
                    else
                    {
                        Sd = K_Z1 * Z_j + K_Z0 - OFFSET_SD;
                        Se = K_Z1 * Z_j + K_Z0 - OFFSET_SE;
                    }

                }
                else if ( ((Y_j <= 0.15) || (Y_j >= -0.15)) && ((Z_j < 0.52) || (Z_j > 0.01)) )     // Se o manche estiver no centro, assegura que não tenha ação
                {
                    Sd = anguloDirDec;
                    Se = anguloEsqDec;
                }

                // EIXO Y -------------------------------------------------------------------------------------
                if ( ( Y_j >= 0.15 ) || ( Y_j <= -0.15 ) )                                          // PITCH
                {
                    Sd = - KD_Y1 * Y_j + KD_Y0 - OFFSET_SD;
                    Se = - KE_Y1 * Y_j + KE_Y0 - OFFSET_SE;
                }


                // EIXO X -------------------------------------------------------------------------------------
                if ( X_j > 0.25 )                                                                    // ROLL
                {
                    Pe +=  K_X0 * X_j;
                }
                else if ( X_j < -0.25)
                {
                    Pd += (-1) * K_X0 * X_j;
                }
            }
            // Controle Proporcional em Malha Fechada usando o joystick 
            else if ( MODE_CLOSED_LOOP_JOY_P == 1 ) 
            {

                // THROTTLE -----------------------------------------------------------------------------------
                if ( A_j != -1.0 )
                {
                    // Seta o valor da potência dos motores
                    Pd_Desired = ( K2 * A_j + K1 );
                    Pe_Desired = ( K2 * A_j + K1 );
                }
                else if ( A_j == -1.0 )
                {
                    Pd_Desired = 0;
                    Pe_Desired = 0;
                }
                // Verifica se é a primeira vez que passa por aqui
                if ( flagFirstTimeControl_P == true )
                {
                    Sd_Old = anguloDirDec;
                    Se_Old = anguloEsqDec;

                    Pd_Old = Pd_Desired;
                    Pe_Old = Pe_Desired;

                    flagFirstTimeControl_P = false;
                }

                // EIXO Z -------------------------------------------------------------------------------------
                if ( ((Y_j <= 0.15) || (Y_j >= -0.15)) && ((Z_j >= 0.52) || (Z_j <= 0.01)) )        // YAW
                {
                    if ( Z_j > 0.52 )                                                               // Compensação para aliviar o defeito do joystick
                    {
                        Sd_Desired = 93.75 * Z_j + 41.25 - OFFSET_SD;
                        Se_Desired = 93.75 * Z_j + 41.25 - OFFSET_SE;                        
                    }
                    else
                    {
                        Sd_Desired = K_Z1 * Z_j + K_Z0 - OFFSET_SD;
                        Se_Desired = K_Z1 * Z_j + K_Z0 - OFFSET_SE;
                    }

                }
                else if ( ((Y_j <= 0.15) || (Y_j >= -0.15)) && ((Z_j < 0.52) || (Z_j > 0.01)) )     // Se o manche estiver no centro, assegura que não tenha ação
                {
                    Sd_Desired = anguloDirDec;
                    Se_Desired = anguloEsqDec;
                }

                // EIXO Y -------------------------------------------------------------------------------------
                if ( ( Y_j >= 0.15 ) || ( Y_j <= -0.15 ) )                                          // PITCH
                {
                    Sd_Desired = - KD_Y1 * Y_j + KD_Y0 - OFFSET_SD;
                    Se_Desired = - KE_Y1 * Y_j + KE_Y0 - OFFSET_SE;
                }

                // Controle Proporcional para aliviar a interface do joystick com o usuário, deixando a resposta mais lenta e mais amigável ao usuário
                Sd = Sd_Old + 0.2 * ( Sd_Desired - Sd_Old );
                Se = Se_Old + 0.2 * ( Se_Desired - Se_Old );

                // EIXO X -------------------------------------------------------------------------------------
                if ( X_j > 0.25 )                                                                   // ROLL
                {
                    Pe_Desired +=  K_X0 * X_j;
                }
                else if ( X_j < -0.25)
                {
                    Pd_Desired += (-1) * K_X0 * X_j;
                }    
                
                // Controle Proporcional
                Pe = Pe_Old + 0.2 * (Pe_Desired - Pe_Old);
                Pd = Pd_Old + 0.2 * (Pd_Desired - Pd_Old);
            }
                        
            // --------------------------------------------------------------------------------------------
            // SATURADOR SERVOS
            // --------------------------------------------------------------------------------------------
            
            // Servo Direito entre 135 e 45 graus
            if ( (Sd + OFFSET_SD ) > 135 )
            {
                Sd = 135 - OFFSET_SD;
            }
            else if ( (Sd + OFFSET_SD ) < 45 )
            {
                Sd = 45 - OFFSET_SD;
            }

            // Servo Esquerdo entre 135 e 45 graus
            if ( (Se + OFFSET_SE) > 135 )
            {
                Se = 135 - OFFSET_SE;
            }
            else if ( (Se + OFFSET_SE) < 45 )
            {
                Se = 45 - OFFSET_SE;
            }

            // Calcula a variação do ângulo dos servos em relação a posição de 90 graus (Valor fica entre 45 e -45)
            deltaTheta_D = 90.0 - (Sd + OFFSET_SD);
            deltaTheta_E = 90.0 - (Se + OFFSET_SE);

            // SATURADOR DO DELTA_THETA
            if ( (deltaTheta_D > 45.0) || (deltaTheta_D < -45.0) )
            {
                deltaTheta_D = 45.0;
            }

            if ( (deltaTheta_E > 45.0) || (deltaTheta_E < -45.0) )
            {
                deltaTheta_E = 45.0;
            }

            // Calcula o valor da potência levando em consideração os ângulos dos servos
            if ((MODE_ANGLE_TO_POWER == 1) && ( Pd > 75 ) && ( Pe > 75 ))
            {
                Pd = Pd/( cos( deltaTheta_D/3 ) );   
                Pe = Pe/( cos( deltaTheta_E/3 ) );
            }

            // --------------------------------------------------------------------------------------------
            // SATURADOR MOTORES BRUSHLESS
            // --------------------------------------------------------------------------------------------

            // Saturador para a potência do rotor direito
            if ( Pd > 160 )
            {
                Pd = 160;
            }
            else if ( Pd < 0 )
            {
                Pd = 0;
            }

            // Saturador para a potência do rotor esquerdo
            if ( Pe > 160 )
            {
                Pe = 160;
            }
            else if ( Pe < 0 )
            {
                Pe = 0;
            }

            // --------------------------------------------------------------------------------------------
            // COMANDOS PARA OS ATUADORES
            // --------------------------------------------------------------------------------------------

            // Envia comandos via SPI para os atuadores 
            spiSendCommandWriteActuators( 'm', 1, (int) Pd, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 2, (int) Pe, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 3, (int) Sd, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 4, (int) Se, spi_fd );                  // Comando para enviar valores aos atuadores
                        
            // Reinicia as variáveis                                                                            
            Sd_Old = Sd;                                                               // Joga o valor de Sd no instante (t) para o instante (t-1)
            Se_Old = Se;                                                               // Joga o valor de Se no instante (t) para o instante (t-1)
            Pd_Old = Pd;
            Pe_Old = Pe;
        }
        // CONTROLE EM MALHA FECHADA PROPORCIONAL + DERIVATIVO -> PILOTO AUTOMÁTICO
        else if ( (MODE_CLOSED_LOOP == 1) && (flagJoystick == false) )
        {
            // THROTTLE -----------------------------------------------------------------------------------
            if ( A_j != -1.0 )
            {
                Pd_Desired = ( K2 * A_j + K1 );
                Pe_Desired = ( K2 * A_j + K1 );
            }
            else if ( A_j == -1.0 )
            {
                Pd_Desired = 0;
                Pe_Desired = 0;
            }

            // CONTROLE PD DE ORIENTAÇÃO
            if ( MODE_CLOSED_LOOP_PD == 1 )
            {
                if ( flagFirstTimeControl == true )
                {
                    Sd_Old = anguloDirDec;
                    Se_Old = anguloEsqDec;
                    gyroX_Old = 0.0;
                    gyroY_Old = 0.0;
                    gyroZ_Old = 0.0;

                    Pd_Old = Pd_Desired;
                    Pe_Old = Pe_Desired;

                    flagFirstTimeControl = false;
                }

                // Controle Guinada (YAW) + Arfagem (PITCH) -- GIROSCÓPIO
                Sd = Sd_Old + 0.77 * ( anguloDirDec - Sd_Old ) + 0.12 * ( 0 - gyroZ_Old ) + 0.06 * ( 0 - gyroX_Old );
                Se = Se_Old + 0.77 * ( anguloEsqDec - Se_Old ) + 0.12 * ( 0 - gyroZ_Old ) - 0.06 * ( 0 - gyroX_Old );

                // Controle Rolagem (ROLL) -- GIROSCÓPIO
                Pd = Pd_Old + 0.77 * (Pd_Desired - Pd_Old) + 0.025 * ( 0 - gyroY_Old );
                Pe = Pe_Old + 0.77 * (Pe_Desired - Pe_Old) - 0.025 * ( 0 - gyroY_Old );

            }

            // --------------------------------------------------------------------------------------------
            // SATURADOR SERVOS
            // --------------------------------------------------------------------------------------------

            // Servo Direito entre 135 e 45 graus
            if ( (Sd + OFFSET_SD ) > 135 )
            {
                Sd = 135 - OFFSET_SD;
            }
            else if ( (Sd + OFFSET_SD ) < 45 )
            {
                Sd = 45 - OFFSET_SD;
            }

            // Servo Esquerdo entre 135 e 45 graus
            if ( (Se + OFFSET_SE) > 135 )
            {
                Se = 135 - OFFSET_SE;
            }
            else if ( (Se + OFFSET_SE) < 45 )
            {
                Se = 45 - OFFSET_SE;
            }

            // Calcula a variação do ângulo dos servos em relação com a posição de 90 graus
            deltaTheta_D = 90.0 - (Sd + OFFSET_SD);
            deltaTheta_E = 90.0 - (Se + OFFSET_SE);

            // SATURADOR DO DELTA_THETA
            if ( (deltaTheta_D > 45.0) || (deltaTheta_D < -45.0) )
            {
                deltaTheta_D = 45.0;
            }

            if ( (deltaTheta_E > 45.0) || (deltaTheta_E < -45.0) )
            {
                deltaTheta_E = 45.0;
            }

            // Calcula o valor da potência levando em consideração os ângulos dos servos
            if ( (MODE_ANGLE_TO_POWER == 1) && ( Pd > 75 ) && ( Pe > 75 ))
            {
                Pd = Pd/( cos( deltaTheta_D/3 ) );   
                Pe = Pe/( cos( deltaTheta_E/3 ) );
            }

            // --------------------------------------------------------------------------------------------
            // SATURADOR
            // --------------------------------------------------------------------------------------------
            // Saturador para a potência do rotor direito
            if ( Pd > 160 )
            {
                Pd = 160;
            }
            else if ( Pd < 0 )
            {
                Pd = 0;
            }

            // Saturador para a potência do rotor esquerdo
            if ( Pe > 160 )
            {
                Pe = 160;
            }
            else if ( Pe < 0 )
            {
                Pe = 0;
            }

            // --------------------------------------------------------------------------------------------
            // COMANDOS PARA OS ATUADORES
            // --------------------------------------------------------------------------------------------

            // Envia comandos via SPI para os atuadores 
            spiSendCommandWriteActuators( 'm', 1, (int) Pd, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 2, (int) Pe, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 3, (int) Sd, spi_fd );                  // Comando para enviar valores aos atuadores
            spiSendCommandWriteActuators( 'm', 4, (int) Se, spi_fd );                  // Comando para enviar valores aos atuadores
                        
            // Reinicia as variáveis
            gyroX_Old = gyroX;
            gyroY_Old = gyroY;
            gyroZ_Old = gyroZ;
            Sd_Old = Sd;
            Se_Old = Se;
            Pd_Old = Pd;
            Pe_Old = Pe;
            
        }

        //-------------------------------------------------------------------------------------------------
        // Controla a periodicidade da tarefa
        //-------------------------------------------------------------------------------------------------

        // Testa o tempo da tarefa e vê se a tarefa passou do seu deadline
        deltaTime = getRealTime() - taskStartTime;

        //if ( deltaTime > PERIOD_CONTROL_TASK )
        //{
            //cout << "Task passed the deadline, check your period... " << endl;
        //}

        //-------------------------------------------------------------------------------------------------
        // Atualiza o Datalogger
        //-------------------------------------------------------------------------------------------------

        // Pega o valor do tempo atual
        T = (getRealTime() - startTime);

        // Salva as variáveis no datalogger
        dataloggerUpdate( T , gyroX, gyroY, gyroZ, sonarAltura, V_BAT, deltaTime, (Sd + OFFSET_SD), (Se + OFFSET_SE), (Pd*100.0/180.0), (Pe*100.0/180.0), X_j, Y_j, Z_j, A_j );

        // Calcula o próximo tempo de início da tarefa
        //taskNextStartTime = PERIOD_CONTROL_TASK - ( getRealTime() - taskStartTime );

        // Espera até o tempo certo para recomeçar a tarefa de novo
        //this_thread::sleep_for(std::chrono::milliseconds((unsigned long long)taskNextStartTime));
    }
    
    // Encerra o datalogger
    closeDatalogger();   

    // Envia comandos via SPI para os atuadores 
    spiSendCommandWriteActuators( 'm', 1, (int) 0, spi_fd );                  // Comando para enviar valores aos atuadores
    spiSendCommandWriteActuators( 'm', 2, (int) 0, spi_fd );                  // Comando para enviar valores aos atuadores
    spiSendCommandWriteActuators( 'm', 3, (int) anguloDirDec, spi_fd );                  // Comando para enviar valores aos atuadores
    spiSendCommandWriteActuators( 'm', 4, (int) anguloEsqDec, spi_fd );                  // Comando para enviar valores aos atuadores
    
    // Finaliza a comunicação SPI
    if( flagSPI == true )
    {
		FinalizaSPI( spi_fd );		
    }

}

// Thread com as tarefas relacionadas à leitura do joystick
void threadJoystick()
{
    // JOYSTICK DEVICE
    int joy_fd = 0;                                 // File descriptor para o Joystick
    struct js_event joystick;				        // Struct com as características do joystick  
    
    int joyValue = 0;                               // Variável que armazana o valor da entrada do joystick

    //-------------------------------------------------------------------------------------------------
    // Open Joystick
    //-------------------------------------------------------------------------------------------------
    joy_fd = openJoystick();    

    flagJoystick = true;                            // Variável que indica que os comandos para os atuadores estão sendo dados pelo joystick
    
    // Leitura do Joystick enquanto o botão de encerrar o programa não for pressionado
    while( exitApplication == false )
    {
        // Lê a entrada que está vindo do joystick
        read (joy_fd, &joystick, sizeof(joystick));

        // Verifica se a entrada é do tipo BUTTON ou AXIS
        if ( joystick.type == JS_EVENT_BUTTON )
        {
            switch ( joystick.number )
            {
                // Caso o botão pressionado seja o número 2 o programa finaliza
                case 1:                                                     
                    if ( joystick.value == 1 )
                    {
                       exitApplication = true;
                    }
                    break;
                // Caso o botão pressionado seja o número 3, ativa o Modo Joystick
                case 2:
                    if ( joystick.value == 1 )
                    {
                       flagJoystick = true;
                    }
                    break;
                // Caso o botão pressionado seja o número 4, ativa o Modo de Estabilização                    
                case 3:
                    if ( joystick.value == 1 )
                    {
                       flagJoystick = false;
                    }
                    break;                                      
                default:
                    break;                
            }
        }
        else if ( joystick.type == JS_EVENT_AXIS )
        {
            switch ( joystick.number )
            {
                // Caso o Axis seja o 0, corresponde ao eixo X -> Direita = 32767 / Esquerda = -32767
                case 0:
                    // Salva o valor lido em uma variável
                    joyValue = joystick.value;

                    // Mapear o valor entre -1 e 1 e salvar numa variável global
                    X_j = mapJoyValue( joyValue, -32767, 32767, -1, 1 );            // RIGHT = 1, LEFT = -1
                    
                    break;

                // Caso o Axis seja o 1, corresponde ao eixo Y -> Trás = 32767 / Frente = -32767
                case 1:
                    // Salva o valor lido em uma variável
                    joyValue = joystick.value;

                    // Mapear o valor entre -1 e 1 e salvar numa variável global
                    Y_j = mapJoyValue( joyValue, -32767, 32767, 1, -1 );            // FRONT = 1, BACK = -1

                    break;

                // Caso o Axis seja o 2, corresponde ao eixo RZ -> Direita = -676 / Centro = -18580/ Esquerda = -32767
                case 2:
                    // Salva o valor lido em uma variável
                    joyValue = joystick.value;

                    // Mapear o valor entre -1 e 1 e salvar numa variável global
                    Z_j = mapJoyValue( joyValue, -676, -32767, -1, 1 );            // RIGHT = -1, LEFT = 1

                    break;

                // Caso o Axis seja o 3, corresponde Throttle -> Trás = 32767 / Frente = -32767
                case 3:
                    // Salva o valor lido em uma variável
                    joyValue = joystick.value;

                    // Mapear o valor entre -1 e 1 e salvar numa variável global
                    A_j = mapJoyValue( joyValue, -32767, 32767, 1, -1 );            // FRONT = 1, BACK = -1
                    
                    break;

                default:
                    break;
                
            }

        }
    }

    // Fecha o arquivo de leitura do Joystick
    if (closeJoystick(joy_fd) != 0)
    {
        cout << "Failed to close joystick device... " << endl;
    }
}

/*
void threadGPS()
{

}
*/

//-------------------------------------------------------------------------------------------------
// FUNCTIONS
//-------------------------------------------------------------------------------------------------

// Função lê o tempo atual com base no CLOCK_REALTIME
double getRealTime()
{
	double tempo;
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp);
	tempo = ( (((double)tp.tv_sec * 1000) + ((double)tp.tv_nsec / 1000000)));

	return (double)(tempo);		
}
