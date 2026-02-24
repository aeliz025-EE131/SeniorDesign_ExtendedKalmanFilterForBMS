#include "BQ_Commands.h"    //ALL COMMANDS/SUBCOMMANDS FUNCTIONS FOR BQ
#include "TimerISR.h"
#include "EKF_Functions.h" //ALL EKF FUNCTIONS FOR BQ

//#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define NUM_TASKS 3
#define buttonA_pin 4

typedef struct task{
	signed 	 char state; 		//Task's current state
	unsigned long period; 		//Task period
	unsigned long elapsedTime; 	//Time elapsed since last task tick
	int (*TickFct)(int); 		//Task tick function
} task;

task tasks[NUM_TASKS]; // declared task array with 5 tasks

void TimerISR() {
	for ( unsigned int i = 0; i < NUM_TASKS; i++ ) {                   // Iterate through each task in the task array
		if ( tasks[i].elapsedTime == tasks[i].period ) {           // Check if the task is ready to tick
			tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
			tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
		}
		tasks[i].elapsedTime += GCD_PERIOD;                        // Increment the elapsed time by GCD_PERIOD
	}
}

//ALL PERIODS TO DECLARE
const unsigned char EKF_Period = 100; //100ms
const unsigned char Button_Period = 100;
const unsigned char BMS_Period = 100;
const unsigned char GCD_PERIOD = 100;

//ALL GLOBAL VARIABLES SHARED ACROSS TICK FUNCTIONS
EKF_1RC ekf[NUM_CELLS];
bool SysON;
bool CHARGE;
bool DSCHRG_FET;
bool CHRG_FET;
float current; //get current command (ex 100mA or -10mA)
uint16_t cell_v[10]; //get voltage in mV (ex. 3700 mV) for each cell

/*-------------------------------------------*/
/*--------- Button State Machine ------------*/
/*-------------------------------------------*/
enum ButtonStates {ButtonINIT, OFF, ButtonPressed_ON, ON, ButtonPressed_OFF};
int Button_TickFun(int state){
    bool button = digitalRead(buttonA_pin); //read button
    //transitions
    switch(state){
        case (ButtonINIT):
            SysON = 0;
            state = OFF;
            break;
        
        case (OFF):
            if(button && !CHARGE){
                SysON = 1;
                state = ButtonPressed_ON;
            }
            break;

        case (ButtonPressed_ON):
            if(!button){
                state = ON;
            }
            break;

        case (ON):
            if(button || CHARGE){
                SysON = 0;
                state = ButtonPressed_OFF;
            }
            break;

        case (ButtonPressed_OFF):
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
enum EKF_State { EKF_init, EKF_RUN};
int TickFun_ExtendedKalmanFilter(int state){
    static unsigned char i;

    //State transitions
    switch(state){
        case(EKF_init):
            //setting up parameters for each cell (total of 10)
            //please initalize all parameters and variables before proceeding!!
            //ALL PARAMETERS ARE IN THE EKF_FUNCTIONS.h
            for(int idx = 0; idx < NUM_CELLS; idx++){
                cells_INIT(idx);}
            i = 0;
            state = EKF_RUN;
            break;

        case (EKF_RUN):
        break;

        default:
            state = EKF_init;
            break;
    }

    //State Actions
    switch(state){
        case(EKF_init):
        break;

        case(EKF_RUN):
        float I_A = current / 1000.0f;     //convert mA → A
        if(i < NUM_CELLS){
            float V_V = cell_v[i] / 1000.0f;   // *** FIXED: mV → V
            Prediction_TimeUpdate(i, I_A);
            Correction_MeasUpdate(i, V_V, I_A); 
        }
        else{
            i = 0; //restart the indexing
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
            }*/
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
        break;*/

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
            */
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

        pack_v = sum(cell_v);*/
        break;

        default:
        break;
    }
    
    return state;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(10);
  
  pinMode(buttonA_pin, INPUT);

  // 1) Enter CONFIGUPDATE mode: subcommand 0x0090
  sendSubcommand(0x0090);
  waitCfgUpdate(true);

  // 3) Write Vcell Mode = 0x0007 at data memory address 0x9304
  bqWriteDataMemWord(0x9304, 0x03FF); //battery cell configuration
  bqWriteDataMemWord(0x9335, 0x0003); //auto cell balancing active
  bqWriteDataMemWord(0x9343, 0x0050); //FET ENABLE
  bqWriteDataMemWord(0x9308, 0x000D); //Fet control Enable
  bqWriteDataMemWord(0x9309, 0x0001); //Charge pump enable
  bqWriteDataMemWord(0x9336, 0x0000);  //configure minimum  temperature
  //bqWriteDataMemWord(0x933F, 0x0E74); //auto cell balancing - relax
  
  // 4) Exit CONFIGUPDATE: subcommand 0x0092
  sendSubcommand(0x0092);
  waitCfgUpdate(false);

  //Set up Tasks
  // TODO: Assign your tasks into the tasks[] array
    tasks[0].period = Button_Period;
    tasks[0].state = ButtonINIT;
    tasks[0].elapsedTime = Button_Period;
    tasks[0].TickFct = &Button_TickFun;

    tasks[1].period = EKF_Period;
    tasks[1].state = EKF_init;
    tasks[1].elapsedTime = EKF_Period;
    tasks[1].TickFct = &TickFun_ExtendedKalmanFilter;

    tasks[2].period = BMS_Period;
    tasks[2].state = BMS_INIT;
    tasks[2].elapsedTime = BMS_Period;
    tasks[2].TickFct = &BMS_Test_TickFun;

    TimerSet(GCD_PERIOD);
    TimerOn();
}

void loop() {}