#ifndef EKF_H
#define EKF_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <avr/interrupt.h>

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
/*-------- PREDICITON TIME UPDATE -----------*/
/*-------------------------------------------*/
void Prediction_TimeUpdate( float I ){
    ekf.SoC = ekf.SoC - (ekf.dt/(ekf.Q_nom)) * (I/1000);
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
    float V_Predict = OCV_SOC(ekf.SoC) - ekf.Vrc - ekf.R_0*I;
    float dK = V - V_Predict;

    //linearize measurement equation 
    //Hk = dh(x,u)/dx = [d(V_OCV(SoC(K)))/d(SoC) -1]
    float H_0 = dOCV_dSoC(ekf.SoC);
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

#endif