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
//const unsigned [***] BMS_Period = */
const unsigned long GCD_PERIOD = 100;

//ALL GLOBAL VARIABLES SHARED ACROSS TICK FUNCTIONS
bool SysON;
bool CHARGE;
bool DSCHRG_FET;
bool CHRG_FET;
uint8_t cell_v[10]; //mV

//assign subcommand to obtain voltage and current
float current; //get current command (ex 100mA or -10mA)
float voltage; //get voltage in mV (ex. 3700 mV)

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
            if(button && !CHRG){
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
            if(button || CHRG){
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
enum EKF_State { EKF_init, EKF_Prediciton, EKF_Update};
int TickFun_ExtendedKalmanFilter(int state){

    //State transitions
    switch(state){
        case(EKF_init):
            //initalize all parameters and variables before proceeding!!
            //please...
            ekf.SoC = 0.5;      //initializing SoC at 50%
            ekf.Vrc = 0.0;
            ekf.R_0 = 0.03;     //internal resistance is 0.03 milli-ohms based on the datasheet
            ekf.Q_nom = 3.5 * 3600;    //Nomincal capacity is 3500 mAhr = 3.5 Ahr

            /*ekf.R_1 = 
            ekf.C_1 =
            ekf.dt = 0.1
            ekf.a = exp(-dt/(R_1*C_1)); 
            ekf.b = R_1*(1 - a);*/

            //Covariance matrix
            ekf.P_00 = 0.01f;
            ekf.P_01 = 0.0f;
            ekf.P_10 = 0.0f;
            ekf.P_11 = 0.01f;

            //Process Noise 
            ekf.Q_00 = 1e-6;
            ekf.Q_11 = 1e-5;

            //Measurement noise
            ekf.R = 1e-3;

            state = EKF_Prediciton;
            break;

        case(EKF_Prediciton):
            //grab current
            state = EKF_Update;
            break;

        case(EKF_Update):
            //grab current AND voltage
            state = EKF_Prediciton;
            break;

        default:
            state = EKF_init;
            break;
    }

    //State Actions
    switch(state){
        case(EKF_init):
            break;

        case(EKF_Prediciton):
            Prediction_TimeUpdate(current);
            break;

        case(EKF_Update):
            Correction_MeasUpdate(voltage, current);
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
    //cells[10];
    //I = GetCurrent();

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
            }
            else if( SysON == 1 && cell_v[i] <= 3300){
                //disableDSCHRG_FETS();
                //enableCHRG_FETS();
                state = DISCHRG_DONE;
            }
        break;

        /*case (DISCHRG_DONE):
            if(I >= 100){
                CHARGE = 1;
                state = CHRG;
            }
        break;
        
        case (CHRG):
            if( I = 0 && !CHARGE){
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
}

void loop() {
    
    // TODO: Assign your tasks into the tasks[] array
    tasks[0].period = Button_Period;
    tasks[0].state = ButtonINIT;
    tasks[0].elapsedTime = Button_Period;
    tasks[0].TickFct = &Button_TickFun;

    tasks[1].period = EKF_Period;
    tasks[1].state = EKF_init;
    tasks[1].elapsedTime = EKF_Period;
    tasks[1].TickFct = &TickFun_ExtendedKalmanFilter;
}