#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BQ_Commands.h"
#include "EKF_Functions.h"

#define buttonA_pin A0

/*-------------------------------------------*/
/*------ Global Variables -------------------*/
/*-------------------------------------------*/

unsigned char SysON;
unsigned char CHARGE;

/*-------------------------------------------*/
/*------ Task Periods -----------------------*/
/*-------------------------------------------*/

const unsigned long EKF_Period = 500;
const unsigned long Button_Period = 500;

/*-------------------------------------------*/
/*--------- Button State Machine ------------*/
/*-------------------------------------------*/

enum ButtonStates {ButtonINIT, OFF, ButtonPressed_ON, ON, ButtonPressed_OFF};

int Button_TickFun(int state){
    bool button = digitalRead(buttonA_pin);

    switch(state){

        case ButtonINIT:
            SysON = 0;
            state = OFF;
        break;

        case OFF:
            if(button && !CHARGE){
                SysON = 1;
                state = ButtonPressed_ON;
            }
        break;

        case ButtonPressed_ON:
            if(!button){
                state = ON;
            }
        break;

        case ON:
            if(button || CHARGE){
                SysON = 0;
                state = ButtonPressed_OFF;
            }
        break;

        case ButtonPressed_OFF:
            if(!button){
                state = OFF;
            }
        break;

        default:
            state = ButtonINIT;
        break;
    }

    return state;
}

/*-------------------------------------------*/
/*---------- EKF State Machine --------------*/
/*-------------------------------------------*/

enum EKF_State { EKF_init, EKF_RUN };

int TickFun_ExtendedKalmanFilter(int state){
    static float current = 0.031f; //change to get current command
    static float voltage = 3.65f;  //change to get voltage command
    static unsigned char count;

    const float min_SOC = 0.3;
    const float max_SOC = 0.6;

    /*----------- State Transitions ----------*/
    switch(state){
        case EKF_init:
            cells_INIT(NUM_CELLS);
            count = 0;
            state = EKF_RUN;
        break;

        case EKF_RUN:
            state = EKF_RUN;
        break;

        default:
            state = EKF_init;
        break;
    }

    /*------------- State Actions ------------*/
    switch(state){
        case EKF_init:
            break;

        case EKF_RUN:
            if(count < NUM_CELLS){
                Prediction_TimeUpdate(count, current);
                Correction_MeasUpdate(count, voltage, current);

                Serial.print("Measured Voltage: ");
                Serial.println(voltage);

                Serial.print("Soc: ");
                Serial.println(ekf[3].SoC);

                count++;
            }
            else if(count >= NUM_CELLS){
                count = 0;
            }
            break;

        default:
            break;
    }

    return state;
}

/*-------------------------------------------*/
/*---------- BMS State Machine --------------*/
/*-------------------------------------------*/

/*
enum states {BMS_INIT, IDLE, DISCHRG, DISCHRG_DONE, CHRG} BMS_state;
int BMS_Test_TickFun(int state){
    //static unsigned char idx = 0;
    //unsigned char pack_v = 0;

    //transitions
    switch(state){
        case (BMS_INIT):
            //disableDSCHRG_FETS();
            //disableCHRG_FETS();
            //disableBalancing();
            CHARGE = 0;
            state = IDLE;
        break;

        case (IDLE):
            if(SysON == 1){
                //EnableDSCHRG_FETS();
                //EnableBalancing();
                state = DISCHRG;
            }
        break;

        case (DISCHRG):
            if(SysON == 0 ){
                //disableDSCHRG_FETS();
                //disableBalancing();
                state = IDLE;
            }/*
            else if( SysON == 1 && ( pack_v <= 33 || ekf.SoC <= MinSoC){
                //disableDSCHRG_FETS();
                state = DISCHRG_DONE;
            }
        break;

        /*case (DISCHRG_DONE):
            if(abs(current) >= 100){
                EnableCHRG_FETS();
                CHARGE = 1;
                state = CHRG;
            }
        break;
        
        case (CHRG):
            if( abs(current) >= 100 && !CHARGE || ekf.SoC >= MaxSoc){
                disableBalancing();
                state = IDLE;
                }
        break;

        default:
            state = BMS_INIT;
            break;
    }

    //actions
    switch(state){
        case (BMS_INIT):
        break;

        case IDLE:
        break;

        case DISCHRG:
            /*
            if( idx < 10){
                cell_v[idx] = GetVoltage();
                idx++;
            }
            else{
                idx = 0;
            }*
            
            pack_v = sum(cell_v);
            
           break;

        case (DISCHRG_DONE):
        break;

        case (CHRG):
        /*if( idx < 10){
            cell_v[idx] = GetVoltage();
        }
        if (pack_v >= 37 || ekf.SoC >= MaxSoC){
            CHRG = 0;
        }

        pack_v = sum(cell_v);
        break;

        default:
        break;
    }
    
    return state;
}
*/

/*-------------------------------------------*/
/*----------- FreeRTOS Task Wrappers --------*/
/*-------------------------------------------*/

void ButtonTask(void *pvParameters){
    int state = ButtonINIT;
    while(true){
        state = Button_TickFun(state);
        vTaskDelay(Button_Period / portTICK_PERIOD_MS);
    }
}

void EKFTask(void *pvParameters){
    int state = EKF_init;
    while(true){
        state = TickFun_ExtendedKalmanFilter(state);
        vTaskDelay(EKF_Period / portTICK_PERIOD_MS);
    }
}

/*void BMSTest(void *pvParameters){
    int state = BMS_INIT;
    while(true){
        state = BMS_Test_Tickfun(state);
        vTaskDelay( BMS_Period / portTICK_PERIOD_MS);
    }
}*/

/*-------------------------------------------*/
/*--------------- Setup ---------------------*/
/*-------------------------------------------*/

void setup(){
    Wire.begin();
    Serial.begin(9600);
    delay(10);

    pinMode(buttonA_pin, INPUT);

    /*
  //1) Enter CONFIGUPDATE mode: subcommand 0x0090
    sendSubcommand(0x0090);
    waitCfgUpdate(true);

  //3) Write Vcell Mode = 0x0007 at data memory address 0x9304
    bqWriteDataMemWord(0x9304, 0x03FF); //battery cell configuration
    bqWriteDataMemWord(0x9335, 0x0003); //auto cell balancing active
    bqWriteDataMemWord(0x9343, 0x0050); //FET ENABLE
    bqWriteDataMemWord(0x9308, 0x000D); //Fet control Enable
    bqWriteDataMemWord(0x9309, 0x0001); //Charge pump enable
    bqWriteDataMemWord(0x9336, 0x0000);  //configure minimum  temperature
  //bqWriteDataMemWord(0x933F, 0x0E74); //auto cell balancing - relax
  
  //4) Exit CONFIGUPDATE: subcommand 0x0092
    sendSubcommand(0x0092);
    waitCfgUpdate(false);
*/
    
/*------- Create FreeRTOS Tasks ----------*/
    xTaskCreatePinnedToCore(
        ButtonTask,
        "ButtonTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        EKFTask,
        "EKFTask",
        8192,
        NULL,
        1,
        NULL,
        1
    );

    /*xTaskCreatePinnedToCore(
        BMSTest,
        4096,
        NULL,
        1,
        NULL,
        1
    );*/
}

/*-------------------------------------------*/
/*--------------- Loop ----------------------*/
/*-------------------------------------------*/
void loop(){
    // Nothing needed here when using FreeRTOS
}