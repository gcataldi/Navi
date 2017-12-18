/// ----------------------------------------------
/// Comunicação Header
/// ----------------------------------------------

#ifndef DATA_H
#define DATA_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>       
#include <thread>         
#include <mutex>    
#include <fstream>      

#include <pthread.h>

#include "gdatalogger/gqueue.h"
#include "gdatalogger/gmatlabdatafile.h"
#include "gdatalogger/gdatalogger.h" 

#define DATALOGGER_STANDARD_QUEUE_SIZE          1000

// Ativador do Datalogger
#define DATALOGGER_ENABLED          1                   // 1 = gdatalogger (.mat) // 2 = datalogger  (.txt)

// Variáveis que serão gravadas
#define DATALOGGER_VAR_TIME         1
#define DATALOGGER_VAR_GYRO_X       1
#define DATALOGGER_VAR_GYRO_Y       1
#define DATALOGGER_VAR_GYRO_Z       1
#define DATALOGGER_VAR_SONAR        1
#define DATALOGGER_VAR_VBAT         1
#define DATALOGGER_VAR_DT           1
#define DATALOGGER_VAR_SD           1
#define DATALOGGER_VAR_SE           1
#define DATALOGGER_VAR_PD           1
#define DATALOGGER_VAR_PE           1
#define DATALOGGER_VAR_X            1
#define DATALOGGER_VAR_Y            1
#define DATALOGGER_VAR_Z            1
#define DATALOGGER_VAR_A            1

// Nome e local do arquivo
#define DATALOGGER_FOLDER           "data/"
#define GDATALOGGER_FILE             "birrotor"
#define GDATALOGGER_EXTENSION        ".mat"

#define DATALOGGER_FILE             "Birrotor_Teste"
#define DATALOGGER_EXTENSION        ".txt"

#define MULTIPLE_FILES              1

using namespace std;


bool initDatalogger();
bool closeDatalogger();
int dataloggerUpdate(double tempo, double gyroX, double gyroY, double gyroZ, double sonarAltura, double VBAT, double dt, double Sd, double Se, double Pd, double Pe, double X, double Y, double Z, double A);

#endif
