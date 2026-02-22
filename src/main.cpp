


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