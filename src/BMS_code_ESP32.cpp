#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BQ_Commands.h"
#include "EKF_Functions.h"

#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define buttonA_pin 15
#define redLED 14
#define greenLED 13

/*-------------------------------------------*/
/*------ Global Variables -------------------*/
/*-------------------------------------------*/

unsigned char SysON;
unsigned char CHARGE;

/*-------------------------------------------*/
/*------ Task Periods -----------------------*/
/*-------------------------------------------*/

const unsigned long EKF_Period = 500;
const unsigned long Button_Period = 100;
const unsigned long BMS_Period = 500;

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
    int16_t current = -1*directCommand(0x3A)/(3); //change to get current command
    int16_t battCell_voltage[10];  //change to get voltage command
    for (int i =0; i<NUM_CELLS; i++){
        battCell_voltage[i]=directCommand(CELL_NO_TO_ADDR(i+1));
    }
    static unsigned char count;
    Serial.print("current: ");
    Serial.println(current);
    /*----------- State Transitions ----------*/
    switch(state){
        case EKF_init:
            for(int i = 0; i < NUM_CELLS; i++){
                cells_INIT(i);
            }
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
                Correction_MeasUpdate(count, battCell_voltage[count], current);
                
                Serial.print("Measured Voltage: ");
                Serial.println(battCell_voltage[7]);

                Serial.print("Soc: ");
                Serial.println(ekf[7].SoC);

                count++;
            }
            else if(count >= NUM_CELLS){
                float soc_sum =0.0f;
                for(int i = 0; i < NUM_CELLS; i++){
                    soc_sum += ekf[i].SoC;
                }
                float pack_soc = soc_sum/NUM_CELLS;
                Serial.print("Pack SoC: "); 
                Serial.println(pack_soc);
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


enum states {BMS_INIT, IDLE, DISCHRG, DISCHRG_DONE, CHRG} BMS_state;

int BMS_Test_TickFun(int state){
    static unsigned char idx = 0;
    int batt_cell[10];
    int16_t pack_curr = directCommand(0x3A)/3;
    unsigned int pack_v = 0;
    for (int i =0; i<NUM_CELLS; i++){batt_cell[i]=directCommand(CELL_NO_TO_ADDR(i+1)); pack_v +=batt_cell[i];}
   /* Serial.print("Pack Voltage: ");
    Serial.println(pack_v);
    Serial.print("Pack Current: ");
    Serial.println(pack_curr);*/

    //transitions
    switch(state){
        case (BMS_INIT):
            CHARGE = 0;
            state = IDLE;
        break;

        case (IDLE):
            if(SysON == 1){
                state = DISCHRG;
            }
        break;

        case (DISCHRG):
            if(SysON == 0 ){
                state = IDLE;
            }
            else if( ( pack_v <= 30000)){
                state = DISCHRG_DONE;
            }
        break;

        case (DISCHRG_DONE):
            if(abs(pack_curr) >= 100){

                CHARGE = 1;
                state = CHRG;
            }
        break;
        
        case (CHRG):
            if( abs(pack_curr) >= 100 && !CHARGE){
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
        sendSubcommand(0x0095);
        break;

        case IDLE:
         sendSubcommand(0x0095);
       //  Serial.println("IDLE");
         digitalWrite(greenLED, 0);
         digitalWrite(redLED, 0);
        break;

        case DISCHRG:
         sendSubcommand(0x0096);
       //  Serial.println("DISCHRG");
         digitalWrite(greenLED, 1);
         digitalWrite(redLED, 0);
        break;

        case (DISCHRG_DONE):
         sendSubcommand(0x0095);
         //Serial.println("DISCHRG done");
         digitalWrite(greenLED, 0);
         digitalWrite(redLED, 1);
        break;

        case (CHRG):
            sendSubcommand(0x0096);
         //   Serial.println("CHRG");
            digitalWrite(redLED, 0);
            if (idx%2 == 0){
                digitalWrite(greenLED, 1);
            }
            else{
                digitalWrite(greenLED, 0);
            }
        break;

        default:
        break;
    }
    
    return state;
}




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

void BMSTest(void *pvParameters){
    int state = BMS_INIT;
    while(true){
        state = BMS_Test_TickFun(state);
        vTaskDelay( BMS_Period / portTICK_PERIOD_MS);
    }
}

/*-------------------------------------------*/
/*--------------- Setup ---------------------*/
/*-------------------------------------------*/

void setup(){
    Wire.begin();
    Serial.begin(9600);
    delay(10);

   pinMode(buttonA_pin, INPUT);
   pinMode(redLED, OUTPUT);
   pinMode(greenLED, OUTPUT);

    
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

    xTaskCreatePinnedToCore(
        BMSTest,
        "BMSTest",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

/*-------------------------------------------*/
/*--------------- Loop ----------------------*/
/*-------------------------------------------*/
void loop(){
    // Nothing needed here when using FreeRTOS
}