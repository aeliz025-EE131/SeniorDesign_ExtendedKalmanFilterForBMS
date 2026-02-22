#include "BQ_Commands.h"    //ALL COMMANDS/SUBCOMMANDS FUNCTIONS FOR BQ
#include "TimerISR.h"
#include "EKF_Functions.h" //ALL EKF FUNCTIONS FOR BQ


#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define packVolt 0x36;


unsigned int buttonA_pin =4 ;
unsigned int buttonB_pin =5 ;
unsigned int redLED = 3;
unsigned int greenLED =2;
unsigned int num_cell = 10;

enum states {INIT,buttonAPressed, buttonBPressed,DCHG, DCHG_done ,CHG, CHG_done} state;
void tick(){
  
  int buttonA;
  int buttonB;
  int batt_cell[10];
  int16_t batt_Curr = directCommand(0x3A)/3;
  for (int i =0; i<num_cell; i++){batt_cell[i]=directCommand(CELL_NO_TO_ADDR(i+1));}
  
  switch(state){
  
    case INIT:
     buttonA = digitalRead(buttonA_pin);
     if(buttonA == 1){state = buttonAPressed;}
     break;
     
    case buttonAPressed:
     buttonA = digitalRead(buttonA_pin);
     Serial.println("button A pressed");
     if(buttonA == 0){state = DCHG;}
     break;
     
    case buttonBPressed:
     buttonA = digitalRead(buttonA_pin);
     Serial.println("button B pressed");
     if(buttonA == 0){state = INIT;}
     break;
     
    case DCHG:
      buttonA = digitalRead(buttonA_pin);
      if(buttonA == 1){state = buttonBPressed;}
      for (int i =0; i<num_cell; i++){ if(batt_cell[i]< 2700){state = DCHG_done;}}
     
     break;
    case DCHG_done:
     buttonA = digitalRead(buttonA_pin);
     if(buttonA == 1){state = buttonBPressed;}
     //if(batt_Curr < 0){state = CHG;}
     
     break;
    case CHG:
     for (int i =0; i<num_cell; i++){ if(batt_cell[i]> 3700){state = INIT;}}
     break;
    case CHG_done:
     break;
  }
  switch(state){
     case INIT:
      sendSubcommand(0x0095);
      Serial.println("INIT");
      digitalWrite(greenLED, 0);
      digitalWrite(redLED, 0);
     break;
    case DCHG:
     sendSubcommand(0x0096);
     Serial.println("DCHG");
     digitalWrite(greenLED, 1);
     break;
    case DCHG_done:
     sendSubcommand(0x0095);
     Serial.println("DCHG done");
     digitalWrite(greenLED, 0);
     digitalWrite(redLED, 1);
     break;
    case CHG:
     sendSubcommand(0x0096);
     Serial.println("CHG");
     digitalWrite(redLED, !(isDischarging()));
     digitalWrite(greenLED, isDischarging());
     digitalWrite(greenLED, !(isDischarging()));
     break;
    case CHG_done:
     break;
  }
  
}

//state machine states
enum states { EKF_init, EKF_Prediciton, EKF_Update} EKF_State;
/*-------------------------------------------*/
/*---------- EKF State Machine --------------*/
/*-------------------------------------------*/
void TickFun_ExtendedKalmanFilter(){
    //assign subcommand to obtain voltage and current
    unsigned char current;
    unsigned char voltage;

    //State transitions
    switch(EKF_State){
        case(EKF_init):
            //initalize all parameters and variables before proceeding!!
            //please...
            ekf.SoC = 0.5;      //initializing SoC at 50%
            ekf.Vrc = 0.0;
            ekf.R_0 = 0.03;     //internal resistance is 0.03 milli-ohms based on the datasheet
            ekf.Q_nom = 3.5;    //Nomincal capacity is 3500 mAhr = 3.5 Ahr

            /*ekf.R_1 = 
            ekf.C_1 =
            ekf.dt = 0.1
            ekf.a = exp(-dt/(R_1*C_1)); 
            ekf.b = R_1*(1 - a);*/

            //Covariance matrix
            ekf.P_00 = 1;
            ekf.P_01 = 0;
            ekf.P_10 = 1;
            ekf.P_11 = 0;

            //Process Noise 
            ekf.Q_00 = 1e-6;
            ekf.Q_11 = 1e-5;

            //Measurement noise
            ekf.R = 1e-3;

            EKF_State = EKF_Prediciton;
            break;

        case(EKF_Prediciton):
            //grab current
            EKF_State = EKF_Update;
            break;

        case(EKF_Update):
            //grab current AND voltage
            EKF_State = EKF_Prediciton;
            break;

        default:
            EKF_State = EKF_init;
            break;
    }

    //State Actions
    switch(EKF_State){
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
}

void setup() {


  Wire.begin();
  Serial.begin(115200);
  delay(10);
  
  pinMode(buttonA_pin, INPUT);
  pinMode(buttonB_pin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

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
  // normal operation

 // bqWriteDataMemWord(0x0084, 0x0ED8);
  //bqWriteDataMemWord(0x0083, 0x0200);
//  int16_t TS1 = directCommand(0x70);
 // int16_t TS3 = directCommand(0x74);
 // int16_t data =bqReadDataMemWord(0x0085);
//  int32_t currData = directCommand(0x3A)/3;
 // Serial.println('1');
  tick();
  delay(500);
}