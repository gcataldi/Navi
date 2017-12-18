#include "Datalogger.h"

using namespace std;

// Cria uma Struct de GDATALOGGER
GDATALOGGER gDataLogger;

// Cria um mutex para sinalizar que o código precisa de acesso exclusivo
mutex dataloggerMutex;

// Função que inicializa o Datalogger usando a GDataLogger
bool initDatalogger()
{
    bool activateLogger = false;

    if ( DATALOGGER_ENABLED == 1 )
    {
        activateLogger = true;

        // Trava o mutex do datalogger
        dataloggerMutex.lock();

        // Nome do Arquivo
        char dataloggerFileName[256];
        char dataloggerFileNameNew[256];
        int i = 0;
        FILE *file;

        strcpy(dataloggerFileName, DATALOGGER_FOLDER);
        strcat(dataloggerFileName, GDATALOGGER_FILE);
        strcat(dataloggerFileName, GDATALOGGER_EXTENSION);

        // Verifica se já existe um arquivo gravado
        if ( MULTIPLE_FILES == 1 )
        {
            while( file = fopen(dataloggerFileName, "r") )
            {
                fclose(file);
    
                strcpy(dataloggerFileNameNew, DATALOGGER_FOLDER);
                strcat(dataloggerFileNameNew, GDATALOGGER_FILE);
                strcat(dataloggerFileNameNew, "_%d.mat");
    
                sprintf(dataloggerFileName, dataloggerFileNameNew, ++i);
            }
        }

        // Enquanto o arquivo existir criar um novo

        // Inicia o GDATALOGGER, caso não consiga retorna  
        if ( !gDataLogger_Init(&gDataLogger, dataloggerFileName, NULL ) )
        {
            dataloggerMutex.unlock();

            cout << "Datalogger could not start... " << endl;

            return false;
        }

        // Declaração de variáveis no GDATALOGGER
        // Argumentos da gDataLogger_DeclareVariable:
        // datalogger pointer, variable name, units, number of rows, number of columns, queue size 
    
        #if DATALOGGER_VAR_TIME == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "t", (char*) "s", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_TIME

        #if DATALOGGER_VAR_GYRO_X == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "gyroX", (char*)"dps", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_GYRO_X

        #if DATALOGGER_VAR_GYRO_Y == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "gyroY", (char*)"dps", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_GYRO_Y

        #if DATALOGGER_VAR_GYRO_Z == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "gyroZ", (char*)"dps", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_GYRO_Z

        #if DATALOGGER_VAR_SONAR == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "altura", (char*)"cm", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_SONAR

        #if DATALOGGER_VAR_VBAT == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "V_Bateria", (char*)"V", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_VBAT

        #if DATALOGGER_VAR_DT == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "dt", (char*) "s", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_DT

        #if DATALOGGER_VAR_SD == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Sd", (char*) "graus", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_SD

        #if DATALOGGER_VAR_SE == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Se", (char*) "graus", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_SE

        #if DATALOGGER_VAR_PD == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Pd", (char*) "potencia/velocidade", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_PD

        #if DATALOGGER_VAR_PE == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Pe", (char*) "potencia/velocidade", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_PE

        #if DATALOGGER_VAR_X == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "X", (char*) "un", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_X

        #if DATALOGGER_VAR_Y == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Y", (char*) "un", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_Y

        #if DATALOGGER_VAR_Z == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "Z", (char*) "un", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_Z

        #if DATALOGGER_VAR_A == 1
        gDataLogger_DeclareVariable(&gDataLogger, (char*) "A", (char*) "un", 1, 1, DATALOGGER_STANDARD_QUEUE_SIZE);
        #endif // DATALOGGER_VAR_A

        // Destrava o mutex
        dataloggerMutex.unlock();

        return activateLogger;
    }
    else if ( DATALOGGER_ENABLED == 2 )
    {
        activateLogger = true;

        // Trava o mutex do datalogger
        dataloggerMutex.lock();

        // Nome do Arquivo
        string fileName = "data/Birrotor_Test.txt";
        
        // Abre o arquivo para escrita
        ofstream fileObject ( fileName.c_str() );

        #if DATALOGGER_VAR_TIME == 1
        fileObject << "Time "; 
        #endif // DATALOGGER_VAR_TIME

        // Destrava o mutex
        dataloggerMutex.unlock();

        return activateLogger;

    }
    else
    {
        cout << "Datalogger is not enabled... " << endl;

        return activateLogger;
    }

}

// Função que fecha o GDATALOGGER
bool closeDatalogger()
{
    bool closeLogger = false;
    
    if ( DATALOGGER_ENABLED == 1 )
    {
        closeLogger = true;
    
        // Trava o mutex do datalogger
        dataloggerMutex.lock();

        // Update data and Empty buffers in the file
        gDataLogger_MatfileUpdate(&gDataLogger);
        
        // Close datalogger 
        gDataLogger_Close(&gDataLogger); 

        // Destrava o mutex
        dataloggerMutex.unlock();
    
        return closeLogger;
    }
    else if ( DATALOGGER_ENABLED == 2 )
    {

    }
    else
    {
        cout << "Datalogger is not enabled... " << endl;
    
        return closeLogger;
    }    
}

// Atualiza os valores e insere no datalogger
int dataloggerUpdate(double tempo, double gyroX, double gyroY, double gyroZ, double sonarAltura, double VBAT, double dt, double Sd, double Se, double Pd, double Pe, double X, double Y, double Z, double A)
{
    dataloggerMutex.lock();

    #if DATALOGGER_VAR_TIME
    gDataLogger_InsertVariable(&gDataLogger, (char*) "t", &tempo);
    #endif // DATALOGGER_VAR_TIME

    #if DATALOGGER_VAR_GYRO_X
    gDataLogger_InsertVariable(&gDataLogger, (char*) "gyroX", &gyroX);
    #endif // DATALOGGER_VAR_GYRO_X

    #if DATALOGGER_VAR_GYRO_Y
    gDataLogger_InsertVariable(&gDataLogger, (char*) "gyroY", &gyroY);
    #endif // DATALOGGER_VAR_GYRO_Y

    #if DATALOGGER_VAR_GYRO_Z
    gDataLogger_InsertVariable(&gDataLogger, (char*) "gyroZ", &gyroZ);
    #endif // DATALOGGER_VAR_GYRO_Z

    #if DATALOGGER_VAR_SONAR
    gDataLogger_InsertVariable(&gDataLogger, (char*) "altura", &sonarAltura);
    #endif // DATALOGGER_VAR_SONAR

    #if DATALOGGER_VAR_VBAT
    gDataLogger_InsertVariable(&gDataLogger, (char*) "V_Bateria", &VBAT);
    #endif // DATALOGGER_VAR_VBAT
    
    #if DATALOGGER_VAR_DT
    gDataLogger_InsertVariable(&gDataLogger, (char *) "dt", &dt);
    #endif // DATALOGGER_VAR_TIME

    #if DATALOGGER_VAR_SD
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Sd", &Sd);
    #endif // DATALOGGER_VAR_SD

    #if DATALOGGER_VAR_SE
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Se", &Se);
    #endif // DATALOGGER_VAR_SE

    #if DATALOGGER_VAR_PD
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Pd", &Pd);
    #endif // DATALOGGER_VAR_PD

    #if DATALOGGER_VAR_PE
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Pe", &Pe);
    #endif // DATALOGGER_VAR_PE

    #if DATALOGGER_VAR_X
    gDataLogger_InsertVariable(&gDataLogger, (char *) "X", &X);
    #endif // DATALOGGER_VAR_X

    #if DATALOGGER_VAR_Y
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Y", &Y);
    #endif // DATALOGGER_VAR_Y

    #if DATALOGGER_VAR_Z
    gDataLogger_InsertVariable(&gDataLogger, (char *) "Z", &Z);
    #endif // DATALOGGER_VAR_Z

    #if DATALOGGER_VAR_A
    gDataLogger_InsertVariable(&gDataLogger, (char *) "A", &A);
    #endif // DATALOGGER_VAR_A

    dataloggerMutex.unlock();

    return true;
}
