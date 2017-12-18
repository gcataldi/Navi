///////////////////////////////////////////////////////////
// PROJETO BIRROTOR                                 ///////
// Trabalho de Graduação de Engenharia Mecatrônica  ///////
// Autor: Gabriel Luiz Pedreira Cataldi             ///////
// Matrícula: 10/0102085                            ///////
///////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Bibliotecas
////////////////////////////////////////////////////////////////////////////

#include <Ultrasonic.h>             // SONAR -> Carrega a biblioteca do sensor ultrassônico
#include <Wire.h>                   // Biblioteca permite comunicar com dispositivos I2C
#include <Adafruit_Sensor.h>        // IMU -> Biblioteca do IMU da Adafruit
#include <Adafruit_LSM303_U.h>      // IMU -> Biblioteca para o Acelerômetro
#include <Adafruit_L3GD20_U.h>      // IMU -> Biblioteca para o Giroscópio
#include <Adafruit_10DOF.h>         // IMU -> Biblioteca para trabalhar com o Adafruit 10DOF Breakout
#include <Servo.h>                  // Biblioteca para controle dos servos


////////////////////////////////////////////////////////////////////////////
// Definição dos Pinos
////////////////////////////////////////////////////////////////////////////

// ESCs
#define ESCD            3           // Define ESC Direito no pino digital 9
#define ESCE            9           // Define ESC Esquerdo no pino digital 3

// Servos
#define SME             5           // Define Servo Motor Esquerdo no pino digital 5
#define SMD             6           // Define Servo Motor Direito no pino digital 6

// Medidor de Tensao na Bateria 
#define divisorTensao   A0          // A0 -> entrada analógica para divisor de tensão
#define lowBattery      A1          // A6 -> usado como saída digital para acionar alarme de bateria fraca

// Sonar
#define pinoTrigger     7           // Define Trigger do Sonar no pino digital 7
#define pinoEcho        8           // Define Echo do Sonar no pino digital 8

////////////////////////////////////////////////////////////////////////////
// Joystick
////////////////////////////////////////////////////////////////////////////
#define axisX           1
#define axisY           2
#define axisZ           3
#define axisThr         4

////////////////////////////////////////////////////////////////////////////
// Ângulo Estáveis de Decolagem
////////////////////////////////////////////////////////////////////////////

#define anguloEsqDec    89
#define anguloDirDec    86

////////////////////////////////////////////////////////////////////////////
// Medidor de Tensão da Bateria
////////////////////////////////////////////////////////////////////////////

// Valores dos Resistores no Divisor de Tensão
#define R1              26943   //27k
#define R2              2100    //2,2k

// Tensão de Referência do Arduino Utilizado
#define tensaoRef       3.3         // Valor de 3.3V para o Arduino Pro Mini

// Modo DEBUG
#define MODE_DEBUG      0

////////////////////////////////////////////////////////////////////////////
// INICIALIZAÇÃO DOS SENSORES
////////////////////////////////////////////////////////////////////////////

// Assign a unique ID to the sensors 
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

//Inicializa o sensor ultrassônico nos pinos definidos 
Ultrasonic ultrasonic(pinoTrigger, pinoEcho);

////////////////////////////////////////////////////////////////////////////
// Declaração de Variáveis Globais
////////////////////////////////////////////////////////////////////////////

// SPI -> receiveBuffer and dataSPI are used to capture incoming data from SPI
unsigned char dataSPI;
unsigned char receiveBuffer[7];

// Joystick -> Variáveis de uso 
int flagJoy = 0;                // Indica que o comando é do Joystick
int flagSign = 0;               // Indica se é positivo ou negativo

// SPI -> marker is used as a pointer to keep track of the current position in the incoming data packet
byte marker = 0;

// SPI -> Unions allow variables to occupy the same memory space a convenient way to move back 
// and forth between 8-bit and 16-bit values
union       
{
    int p1Int;
    unsigned char  p1Char [2];
} p1Buffer;

union       
{
    int p2Int;
    unsigned char  p2Char [2];
} p2Buffer;

union       
{
    int p3Int;
    unsigned char  p3Char [2];
} p3Buffer;

// Servos e ESCs -> Variáveis de hardware
Servo servoEsquerdo;
Servo servoDireito;
Servo ESCdireito; 
Servo ESCesquerdo;

// Variáveis para o modo de emergência
int modoEmergencia = 0;
int redLedState = 0;  

// Flags
int flagGyro = 0;                           // Indica que a leitura é do giroscópio
int flagBatt = 0;                           // Indica que a leitura é da Bateria    
int flagSonar = 0;

// Valores lidos dos sensores
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
unsigned char valueSign;

////////////////////////////////////////////////////////////////////////////
// Setup do Arduino
////////////////////////////////////////////////////////////////////////////

void setup (void)
{
    if ( MODE_DEBUG == 1 )
    {
        Serial.begin(9600);       
    }
  
    // Configura servos e escs
    servoEsquerdo.attach(SME);           // Servo esquerdo para a porta digital 5 do arduino Pro mini
    servoDireito.attach(SMD);            // Servo direito para a porta digital 6 do arduino Pro mini
    servoEsquerdo.write(anguloEsqDec);   // Valor inicial de ângulo estavel para decolagem
    servoDireito.write(anguloDirDec);    // Valor inicial de ângulo estavel para decolagem
    ESCdireito.attach(ESCD);             // Esc direito para a porta digital 9 do arduino Pro mini
    ESCesquerdo.attach(ESCE);            // Esc esquerdo para a porta digital 3 do arduino Pro mini
    
    // Inicializa os motores
    armMotors();

    // Inicializa sensores da IMU
    gyro.begin(GYRO_RANGE_250DPS);                       // Giroscópio (rad/s)
      
    // Configura a comunicação SPI
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
}  

////////////////////////////////////////////////////////////////////////////
// Laço Principal do Arduino
////////////////////////////////////////////////////////////////////////////

void loop (void)
{
  /*
    if ( MODE_DEBUG == 1 )
    {
        sensors_event_t event;
        gyro.getEvent(&event);

        Serial.println("x ");
        Serial.println(event.gyro.x*1000);
        Serial.println("y ");
        Serial.println(event.gyro.y*1000);
        Serial.println("z ");
        Serial.println(event.gyro.z*1000);
        
    }

    if ( MODE_DEBUG == 1 )
    {
        float teste = 0.0;

        teste = medeBateria();

        Serial.println(teste);

        delay(500);
          
    }
*/
    if((SPSR & (1 << SPIF)) != 0)
    {
        spiHandler();
    }
    
}

////////////////////////////////////////////////////////////////////////////
// Funções
////////////////////////////////////////////////////////////////////////////

void spiHandler()
{
    switch ( marker )
    {
        // Handshake
        case 0:
            dataSPI = SPDR;
            if ( dataSPI == 'c' )
            {
                SPDR = 'a';
                marker++;
            }
            break;
        
        // Comando
        case 1:
            receiveBuffer[marker-1] = SPDR;
            marker++;
            break;
        case 2:
            receiveBuffer[marker-1] = SPDR;
            marker++;
            break;
        case 3:
            receiveBuffer[marker-1] = SPDR;
            marker++;
            if ( (flagGyro == 1) || (flagBatt == 1) || (flagSonar == 1) )
            {
                SPDR = valueSign;
            }
            break;
        case 4:
            receiveBuffer[marker-1] = SPDR; 
            marker++;            
            // Converte o valor para inteiro para ser enviado
            if ( (flagGyro == 1) || (flagBatt == 1) || (flagSonar == 1) )
            {
                SPDR = p3Buffer.p3Char[0];                      // Envia o primeiro Byte do Inteiro
            }
            break;
        case 5:
            receiveBuffer[marker-1] = SPDR;
            marker++;
            if ( (flagGyro == 1) || (flagBatt == 1) || (flagSonar == 1) )
            {
                SPDR = p3Buffer.p3Char[1];                      // Envia o segundo byte do Inteiro
                flagGyro = 0; 
                flagBatt = 0;
                flagSonar = 0;
            }
            break;
        case 6:
            receiveBuffer[marker-1] = SPDR;
            executeCommand();
            marker++;  
            break;
        case 7:
            dataSPI = SPDR;
            // Zera as variáveis de controle
            marker = 0;
            break;
        default:
            break;
    }

}

// Funções dos motores - Inicializa os motores com velocidade 0 
//                       e espera 1 segundo para evitar erros
void armMotors()
{
  setSpeedDIR(0); 
  setSpeedESQ(0);
}

// Função que seta a velocidade do motor Direito
void setSpeedDIR(int velocidade)
{ 
  // Pela biblioteca servo.h em uma potência/velocidade
  ESCdireito.write(velocidade);
}

// Função que seta a velocidade do motor Esquerdo
void setSpeedESQ(int velocidade)
{
  // Pela biblioteca servo.h em uma potência/velocidade 
  ESCesquerdo.write(velocidade);
}

void executeCommand()
{   
    // Command
    // MOTORES -----------------------------------------------------------------------------------------------
    if ( receiveBuffer[0] == 'm' )                    // COMMAND = 'm' => Escreve o valor correspondente nos atuadores
    {
        // Inicializa as variáveis
        int op = 0;
        int Pd = 0;
        int Pe = 0;
        int Sd = 0;
        int Se = 0;

        // Operation: 1/2/3/4 = Pd/Pe/Sd/Se
        p1Buffer.p1Char[0] = receiveBuffer[1];
        p1Buffer.p1Char[1] = receiveBuffer[2];

        // Valor do controle
        p2Buffer.p2Char[0] = receiveBuffer[3];
        p2Buffer.p2Char[1] = receiveBuffer[4];

        op =  p1Buffer.p1Int;
        
        switch ( op )
        {
            case 1:
                Pd = p2Buffer.p2Int;
                setSpeedDIR(Pd);
                //Serial.println(Pd);
                break;
            case 2:
                Pe = p2Buffer.p2Int;
                setSpeedESQ(Pe);
                //Serial.println(Pe);
                break;
            case 3:
                Sd = p2Buffer.p2Int;
                servoDireito.write(Sd);
                //Serial.println(Sd);
                break;
            case 4:
                Se = p2Buffer.p2Int;
                servoEsquerdo.write(Se);
                //Serial.println(Se);
                break;
            default:
              break;
        }
    }
    // GIROSCÓPIO -----------------------------------------------------------------------------------------------
    else if ( (receiveBuffer[0] == 'g') && ( flagGyro == 0 ) )     // COMMAND = 'g' -> Comando do Giroscópio
    {
        flagGyro = 1;

        sensors_event_t event;

        gyro.getEvent(&event);

        if ( receiveBuffer[1] == 'x' )
        {
            gyroX = event.gyro.x;                           // Lê o valor do sensor no eixo X em rad/s

            if ( (gyroX >= 0) )                             // Sinaliza se o número é positivo ou negativo
            {
                valueSign = 'p';
            }
            else
            {
                valueSign = 'n';
            }             
            p3Buffer.p3Int = abs((int) (gyroX * 100.0));    // Multiplica por 100 para deslocar a vírgula e Converte o valor para inteiro
            
        }
        else if ( receiveBuffer[1] == 'y' )
        {
            gyroY = event.gyro.y;                           // Lê o valor do sensor no eixo Y em rad/s

            if ( (gyroY >= 0) )                             // Sinaliza se o número é positivo ou negativo
            {
                valueSign = 'p';
            }
            else
            {
                valueSign = 'n';
            }
            
            p3Buffer.p3Int = abs((int) (gyroY * 100.0));     // Multiplica por 100 para deslocar a vírgula e Converte o valor para inteiro                    
        } 
        else if ( receiveBuffer[1] == 'z' )      
        {
            gyroZ = event.gyro.z;                            // Lê o valor do sensor no eixo Z em rad/s

            if ( (gyroZ >= 0) )                              // Sinaliza se o número é positivo ou negativo
            {
                valueSign = 'p';
            }
            else
            {
                valueSign = 'n';
            } 

            p3Buffer.p3Int = abs((int) (gyroZ * 100.0));     // Multiplica por 100 para deslocar a vírgula e Converte o valor para inteiro            
        }          
    }
    // BATERIA -----------------------------------------------------------------------------------------------
    else if ( (receiveBuffer[0] == 'b') && ( flagBatt == 0 ) )
    {
        float V_BAT = 0.0;

        flagBatt = 1;

        valueSign = 'p';                                     // Apenas para manter o padrão da comunicação, tensão da bateria sempre será positiva

        if ( receiveBuffer[1] == 'n' )                       // Leitura da Bateria
        {
            V_BAT = medeBateria();

            p3Buffer.p3Int = abs( (int) ( V_BAT * 100.0 ) ); // Multiplica por 100 para deslocar a vírgula e Converte o valor para inteiro

            modoEmergencia = 0;
        }
        else if ( receiveBuffer[1] == 'p' )                  // Comando para acionar o alarme de bateria fraca
        {
            modoEmergencia = 1;                              // Ativa modo de emergência
            
            // Pisca o LED vermelho no período determinado no Mestre
            if ( redLedState == 0 ) 
            {
                digitalWrite(lowBattery, HIGH);              // Liga o LED Vermelho (nível lógico ALTO de tensão)
            } 
            else 
            {
                digitalWrite(lowBattery, LOW);               // Desliga o LED Vermelho (nível lógico BAIXO)
            }
        }

    }
    else if ( (receiveBuffer[0] == 's') && ( flagSonar == 0 ) )
    {
        float altura = 0.0;

        flagSonar = 1;

        valueSign = 'p';                                     // Apenas para manter o padrão da comunicação, tensão da bateria sempre será positiva

        if ( receiveBuffer[1] == '-' )                       // Leitura da Bateria
        {
            altura =leituraSonar();
            
            p3Buffer.p3Int = abs( (int) ( altura * 100.0 ) ); // Multiplica por 100 para deslocar a vírgula e Converte o valor para inteiro

        }

    }

}

// Função que realiza a leitura do sonar
float leituraSonar()
{
  // Inicialização das variáveis locais
  long microsec = 0;                // Tempo de retorno do sinal do ultrassom
  float alturaCM = 0.0;             //Le as informações do sensor em cm
  
  // Tempo de retorno do sinal do ultrassom
  microsec = ultrasonic.timing();
  
  // Calcula a distancia em centimetros
  alturaCM = ultrasonic.convert(microsec, Ultrasonic::CM);
  
  if ( MODE_DEBUG == 1 )
  {
      // Apresenta os dados na tela
      Serial.print("Distancia em cm:  ");
      Serial.print(alturaCM);
      Serial.println("");

      // Tempo de espera
      delay(100);
  }

  return alturaCM;
}

// Função que realiza a leitura da tensão na bateria
float medeBateria()
{
    // Inicialização das variáveis
    float V_ADC = 0.0;
    float V_BAT = 0.0;
    int valorMedido = 0.0;

    // Lê as informações na porta analógica escolhida (0 a 1023)
    valorMedido = analogRead(divisorTensao);

    // Conversão da tensão lida, sabendo que a porta analógica tem 10 bits (1024 valores diferentes)
    V_ADC = ( (float) valorMedido * tensaoRef) / 1024.0;

    //Serial.println(V_ADC);
  
    // Tensão da bateria
    V_BAT  = V_ADC * (R1+R2)/R2;
    
    return V_BAT;

}

