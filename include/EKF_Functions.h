#ifndef EKF_H
#define EKF_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define NUM_CELLS 10

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
extern EKF_1RC ekf[NUM_CELLS];

const float a = 2.50638087f;
const float b = 4.05668091f;
const float c = -5.73399353f;
const float d = 3.29784602f;

float OCV_SOC(float soc){
    return a + b*soc + c*(soc*soc) + d*(soc*soc*soc);
}

float dOCV_dSoC(float soc){
    return b + 2*c*soc + 3*d*(soc*soc);
}

/*-------------------------------------------*/
/*------ Initalizing Cells Function ---------*/
/*-------------------------------------------*/
void cells_INIT(int i){
    ekf[i].SoC = 0.5;              //initializing SoC at 50%
    ekf[i].Vrc = 0.0;
    ekf[i].R_0 = 0.03;            //internal resistance is 0.03 milli-ohms based on the datasheet
    ekf[i].Q_nom = 3.5 * 3600;    //Nomincal capacity is 3500 mAhr = 3.5 Ahr


    ekf[i].R_1 = 0.015; //milli-ohms
    ekf[i].C_1 = 2500;  //Farads
    ekf[i].dt = 0.1f;   //100ms
    ekf[i].a = exp(-ekf[i].dt/(ekf[i].R_1 * ekf[i].C_1)); 
    ekf[i].b = ekf[i].R_1*(1.0f - a);

    //Covariance matrix
    ekf[i].P_00 = 0.01f;
    ekf[i].P_01 = 0.0f;
    ekf[i].P_10 = 0.0f;
    ekf[i].P_11 = 0.01f;

    //Process Noise 
    ekf[i].Q_00 = 1e-6;
    ekf[i].Q_11 = 1e-5;

    //Measurement noise
    ekf[i].R = 1e-3;
}

/*-------------------------------------------*/
/*-------- PREDICITON TIME UPDATE -----------*/
/*-------------------------------------------*/
void Prediction_TimeUpdate(int idx, float I ){
    ekf[idx].SoC = ekf[idx].SoC - (ekf[idx].dt/(ekf[idx].Q_nom)) * I;
    ekf[idx].Vrc = ekf[idx].a * ekf[idx].Vrc + ekf[idx].b * I;

    float P_00 = ekf[idx].P_00;
    float P_01 = ekf[idx].P_01;
    float P_10 = ekf[idx].P_10;
    float P_11 = ekf[idx].P_11;

    //Jacobian matrix F(x,u) = [1 0; 0 a]
    //P = F*P*(F)^T + Q
    ekf[idx].P_00 = P_00 + ekf[idx].Q_00; //1
    ekf[idx].P_01 = ekf[idx].a * P_01;    //0
    ekf[idx].P_10 = ekf[idx].a * P_10;    //0
    ekf[idx].P_11 = ekf[idx].a * ekf[idx].a * P_11 + ekf[idx].Q_11; //a
}

/*-------------------------------------------*/
/*----- Correction Measurement Update -------*/
/*-------------------------------------------*/
void Correction_MeasUpdate( int idx, float V, float I){
    //Vt = V_OCV(SoC(K)) - Vrc - R0*I(K)
    float V_Predict = OCV_SOC(ekf[idx].SoC) - ekf[idx].Vrc - ekf[idx].R_0* I;
    float dK = V - V_Predict;

    //linearize measurement equation 
    //Hk = dh(x,u)/dx = [d(V_OCV(SoC(K)))/d(SoC) -1]
    float H_0 = dOCV_dSoC(ekf[idx].SoC);
    float H_1 = -1.0f;

    //innovation covariance S
    //S(k) = H*P_predict*(H)^T + R
    float S = H_0*(ekf[idx].P_00*H_0 + ekf[idx].P_01*H_1) +
              H_1*(ekf[idx].P_10*H_0 + ekf[idx].P_11*H_1) +
              ekf[idx].R;

    //Compute the Kalman Gains K0 and K1
    //K = P*(H)^T*(S)^(-1)
    float KalmanGain_0 = (ekf[idx].P_00*H_0 + ekf[idx].P_01*H_1)/S;
    float KalmanGain_1 = (ekf[idx].P_10*H_0 + ekf[idx].P_11*H_1)/S;

    //Updating the state space equations
    //x = x(k) + K*dK
    ekf[idx].SoC += KalmanGain_0*dK;
    ekf[idx].Vrc += KalmanGain_1*dK;

    //REASSIGNMENT
    float P_00_Update = ekf[idx].P_00;
    float P_01_Update = ekf[idx].P_01;
    float P_10_Update = ekf[idx].P_10;
    float P_11_Update = ekf[idx].P_11;

    //UPDATING THE COVARIANCE MATRIX
    //P(k)​=(I−K(k)*​H(k)​)P(k)−​
    ekf[idx].P_00 = P_00_Update - KalmanGain_0*(H_0*P_00_Update + H_1*P_10_Update);
    ekf[idx].P_01 = P_01_Update - KalmanGain_0*(H_0*P_01_Update + H_1*P_11_Update);

    ekf[idx].P_10 = P_10_Update - KalmanGain_1*(H_0*P_00_Update + H_1*P_10_Update);
    ekf[idx].P_11 = P_11_Update - KalmanGain_1*(H_0*P_01_Update + H_1*P_11_Update);

    //SETTING THRESHOLDS IN CASE SoC is >1 or <0
    if( ekf[idx].SoC >= 1.0f){ ekf[idx].SoC = 1.0f;}
    if( ekf[idx].SoC <= 0.0f){ ekf[idx].SoC = 0.0f;}
}

#endif