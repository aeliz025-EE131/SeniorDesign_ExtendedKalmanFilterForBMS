#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <TimerISR.h>
/*#include <avr/io.h>
#include <avr/interrupt.h>
*/

//state machine states
enum states { EKF_init, EKF_Prediciton, EKF_Update} EKF_State;

//All state space variables and battery parameters
//battery parameters must be obtained from pulse discharge test
typedef struct{
    //State space vector [Soc, Vrc]
    float SoC;
    float Vrc;

    //Covariance Matrix 2x2
    float P_00;
    float P_01;
    float P_10;
    float P_11;

    //Process Noise 1x2
    float Q_00;
    float Q_11;
    float R;                            //Measurement Noise 1x1

    float R_0;                          //internal resistance
    float R_1;                          //Diffusion Resistance
    float C_1;                          //Diffusion Capacitance
    float Q_nom;                        //Nominal capacity in Ahr
    float dt;                           //delta t = 100ms = 0.1s

    float a;                            //exp(-dt/(R_1*C_1))
    float b;                            //R_1(1 - a)

} EKF_1RC;
static EKF_1RC ekf;

/*-------------------------------------------*/
/*-------- OCV-SOC LookUp Table - -----------*/
/*-------------------------------------------*/
#define OCV_TABLE_SIZE 17

// SOC in ascending order (0 → 1)
const float soc_table[OCV_TABLE_SIZE] =
{
    0.0143f,
    0.0429f,
    0.0714f,
    0.1000f,
    0.1429f,
    0.2143f,
    0.2857f,
    0.3571f,
    0.4286f,
    0.5000f,
    0.5714f,
    0.6429f,
    0.7143f,
    0.7857f,
    0.8571f,
    0.9286f,
    1.0000f
};

const float ocv_table[OCV_TABLE_SIZE] =
{
    2.55f,
    2.80f,
    3.05f,
    3.25f,
    3.40f,
    3.52f,
    3.60f,
    3.66f,
    3.72f,
    3.78f,
    3.85f,
    3.90f,
    3.95f,
    4.00f,
    4.04f,
    4.08f,
    4.12f
};

float OCV_lookup(float soc)
{
    if (soc <= soc_table[0])
        return ocv_table[0];

    if (soc >= soc_table[OCV_TABLE_SIZE - 1])
        return ocv_table[OCV_TABLE_SIZE - 1];

    for (int i = 0; i < OCV_TABLE_SIZE - 1; i++)
    {
        if (soc >= soc_table[i] && soc <= soc_table[i + 1])
        {
            float x0 = soc_table[i];
            float x1 = soc_table[i + 1];
            float y0 = ocv_table[i];
            float y1 = ocv_table[i + 1];

            float slope = (y1 - y0) / (x1 - x0);

            return y0 + slope * (soc - x0);
        }
    }

    return ocv_table[0];
}

float dOCV_dSoC_lookup(float soc)
{
    if (soc <= soc_table[0])
        soc = soc_table[0];

    if (soc >= soc_table[OCV_TABLE_SIZE - 1])
        soc = soc_table[OCV_TABLE_SIZE - 1];

    for (int i = 0; i < OCV_TABLE_SIZE - 1; i++)
    {
        if (soc >= soc_table[i] && soc <= soc_table[i + 1])
        {
            float x0 = soc_table[i];
            float x1 = soc_table[i + 1];
            float y0 = ocv_table[i];
            float y1 = ocv_table[i + 1];

            return (y1 - y0) / (x1 - x0);
        }
    }

    return 0.0f;
}

/*-------------------------------------------*/
/*-------- PREDICITON TIME UPDATE -----------*/
/*-------------------------------------------*/
void Prediction_TimeUpdate( float I ){
    ekf.SoC = ekf.SoC - (ekf.dt/(ekf.Q_nom)) * I;
    ekf.Vrc = ekf.a * ekf.Vrc + ekf.b * I;

    float P_00 = ekf.P_00;
    float P_01 = ekf.P_01;
    float P_10 = ekf.P_10;
    float P_11 = ekf.P_11;

    //Jacobian matrix F(x,u) = [1 0; 0 a]
    //P = F*P*(F)^T + Q
    ekf.P_00 = P_00 + ekf.Q_00; //1
    ekf.P_01 = ekf.a * P_01;    //0
    ekf.P_10 = ekf.a * P_10;    //0
    ekf.P_11 = ekf.a * ekf.a * P_11 + ekf.Q_11; //a
}

/*-------------------------------------------*/
/*----- Correction Measurement Update -------*/
/*-------------------------------------------*/
void Correction_MeasUpdate( float V, float I){
    //Vt = V_OCV(SoC(K)) - Vrc - R0*I(K)
    float V_Predict = OCV_lookup(ekf.SoC) - ekf.Vrc - ekf.R_0*I;
    float dK = V - V_Predict;

    //linearize measurement equation 
    //Hk = dh(x,u)/dx = [d(V_OCV(SoC(K)))/d(SoC) -1]
    float H_0 = dOCV_dSoC_lookup(ekf.SoC);
    float H_1 = -1.0f;

    //innovation covariance S
    //S(k) = H*P_predict*(H)^T + R
    float S = H_0*(ekf.P_00*H_0 + ekf.P_01*H_1) +
              H_1*(ekf.P_10*H_0 + ekf.P_11*H_1) +
              ekf.R;

    //Compute the Kalman Gains K0 and K1
    //K = P*(H)^T*(S)^(-1)
    float KalmanGain_0 = (ekf.P_00*H_0 + ekf.P_01*H_1)/S;
    float KalmanGain_1 = (ekf.P_10*H_0 + ekf.P_11*H_1)/S;

    //Updating the state space equations
    //x = x(k) + K*dK
    ekf.SoC += KalmanGain_0*dK;
    ekf.Vrc += KalmanGain_1*dK;

    //REASSIGNMENT
    float P_00_Update = ekf.P_00;
    float P_01_Update = ekf.P_01;
    float P_10_Update = ekf.P_10;
    float P_11_Update = ekf.P_11;

    //UPDATING THE COVARIANCE MATRIX
    //P(k)​=(I−K(k)*​H(k)​)P(k)−​
    ekf.P_00 = P_00_Update - KalmanGain_0*(H_0*P_00_Update + H_1*P_10_Update);
    ekf.P_01 = P_01_Update - KalmanGain_0*(H_0*P_01_Update + H_1*P_11_Update);

    ekf.P_10 = P_10_Update - KalmanGain_1*(H_0*P_00_Update + H_1*P_10_Update);
    ekf.P_11 = P_11_Update - KalmanGain_1*(H_0*P_01_Update + H_1*P_11_Update);

    //SETTING THRESHOLDS IN CASE SoC is >1 or <0
    if( ekf.SoC >= 1.0f){ ekf.SoC = 1.0f;}
    if( ekf.SoC <= 0.0f){ ekf.SoC = 0.0f;}
}

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

int main(){
   //TODO: initialize all outputs and inputs
  //TODO: initialize your global variables, state, etc.


  EKF_State = EKF_init;
  //ADC_init();
  TimerOn();
  TimerSet(100); //100ms = 0.1sec
    while (1){
	  TickFun_ExtendedKalmanFilter();      // Execute one synchSM tick
      //add the timerflag header file
      while (!TimerFlag){}  // Wait for SM period
      TimerFlag = 0;        // Lower flag
     }
    return 0;
}