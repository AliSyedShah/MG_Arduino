/* Control tasks are divided into three tasks. 
 *  MPPT task with the lowest priority 
 *  Droop task with intermediate priority
 *  Current Control task with hoghest priority
 *  xQueue is used to send the reference current K_droop from the MPPT task to the Droop task
 *  xQueue1 is used to send I_ref from the Droop task to the Current Control task
 *  xQueue2 is to send the activation signal from the CC task to the MPPT task
 */

#include <FreeRTOS_ARM.h>
#include <basic_io_arm.h>


// Task Definition
void TaskMPPT( void *pvParameters );
void TaskDroop( void *pvParameters );
void TaskCC( void *pvParameters );

QueueHandle_t xQueue;
QueueHandle_t xQueue1;
QueueHandle_t xQueue2;

void setup() {

  //Serial.begin(115200);
  
  analogReadResolution(12);
  analogWriteResolution(12);

  xQueue = xQueueCreate(1, sizeof( double ) );
  xQueue1 = xQueueCreate(1, sizeof( double ) );
  xQueue2 = xQueueCreate(1, sizeof( bool ) );
  if(xQueue != NULL){
    
  //create tasks

  xTaskCreate(
    TaskCC
    , (const portCHAR *)"CC"  //Human Name
    , 200                    //Stack Size
    , NULL
    , 3                     //Priority with 3 being the highest
    , NULL );

  xTaskCreate(
    TaskDroop
    , (const portCHAR *)"Droop"  //Human Name
    , 200                    //Stack Size
    , NULL
    , 2                     //Priority with 3 being the highest
    , NULL );


    xTaskCreate(
      TaskMPPT
      , (const portCHAR *)"MPPT"
      , 128
      , NULL
      ,1                    //Priority
      , NULL );

      vTaskStartScheduler();
  }

  else{
    
}

}

void loop() {
  // empty

}

//*********************************************************//


void TaskCC(void *pvParameters)
{
    (void) pvParameters;
    //Reads K from MPPT task
    TickType_t x_LWT;  //Last wake time used to execute thread deterministically
    
    bool EN;
    double I_meas;
    double I_ref;
    
    double I_PV;
   
    
    double H = 0.1;     //Hyterisis Band
    bool S,R;     //S R latch
    bool DD;      //Switch HIGH or LOW
    double d;
    pinMode(9,OUTPUT); //Switch
    pinMode(6,INPUT);
    pinMode(10,OUTPUT);
    pinMode(2,OUTPUT);
    x_LWT = xTaskGetTickCount();
    
    
    for(;;)
    {
      digitalWrite(2,HIGH);
      while(digitalRead(6)==LOW){  //Simulation Not Running Initialize Values
      I_ref = 0;
      I_meas = 0;
      I_PV = 0;
      DD = 0;
      digitalWrite(10,HIGH);   //LED 
      }
      digitalWrite(10,LOW);
      xQueuePeek(xQueue1, &I_ref, 0);     //Get I_ref from Droop task
     

      //I_meas = analogRead(A0);
      //I_meas = map(I_meas,0,4035,0,35)+1;
      //Serial.println(I_meas);
      
      
      I_PV = analogRead(A4);
      I_PV = map(I_PV,0,4070,0,45);
     
     //Serial.print(I_PV);
     //Serial.print("  ");
     //Serial.println(I_ref);
      
      //S = I_meas <= I_ref-H;
      //R = I_meas >= I_ref+H;
      S = I_PV <= I_ref-H;
      R = I_PV >= I_ref+H;
 
      
      
     
      if(S) DD = HIGH;
      if(R) DD = LOW;
 
//     d = DD*100;
//     d = map(d,0,100,0,4095);
//      analogWrite(9,d);
      digitalWrite(9,DD);
      //I_meas = analogRead(A0);
      //I_meas = map(I_meas,0,4070,0,35);
      
      EN = abs(I_ref-I_meas)<=0.5;
      xQueueOverwrite(xQueue2, &EN);

      
   
     //digitalWrite(2,LOW);
     digitalWrite(2,LOW);
     vTaskDelayUntil( &x_LWT,50);
    }
}


//*****************************************//
//****************************************//
//***************************************//



void TaskDroop(void *pvParameters)
{

  double V_DG;
  double V_out;
  double V_ref = 400;  //No Load Voltage
  double I_ref;
  double AOIREF;
  double I_PV;
  double K;
  bool A;
  TickType_t x_LWT;
  
  x_LWT = xTaskGetTickCount();

  for(;;)
  {
    digitalWrite(3,HIGH);
    while(digitalRead(6)==LOW){
      digitalWrite(10,HIGH);
      V_out = 0;
      V_DG = 0;
      I_ref = 0;
      K = 0.2;
      }
      
    V_out = analogRead(A2);
    V_out = map(V_out,0,4070,0,460);

    V_DG = analogRead(A1);
    V_DG = map(V_DG,0,4070,0.1,50);

    I_PV = analogRead(A4);
    I_PV = map(I_PV,0,4070,0,45);

    xQueuePeek(xQueue, &K, 0);     //Get K from MPPT task


     //*********A*********//
    if(V_out >= V_ref-20) A = false;

    if(V_out < 250) A = true;
    //digitalWrite(2,A);
    //***************************//
//*******************//
//*******I_ref*******//
//******************//
    
    if(V_out<5){
     I_ref = 2;
    }
    else{
     I_ref = K*(V_ref-V_out)*K*V_out/V_DG;   //Calculate I_ref

     if(I_ref >= I_PV+2) I_ref = I_PV+2;

     if(A && I_ref>4) I_ref = 4;
    }
     // I_ref = 5;//**************************
     xQueueOverwrite(xQueue1, &I_ref);
     AOIREF = map(I_ref,0,25,0,4090);
     analogWrite(DAC0,AOIREF);
      //Serial.println(I_ref);
      digitalWrite(3,LOW);
     vTaskDelayUntil( &x_LWT,200 );

    
  }
}


//****************************************************//
//****************************************************//
//****************************************************//

void TaskMPPT(void *pvParameters)
{
    (void) pvParameters;
    
    double K;
    double dK = 0.0002;
    double I_PV;
    double V_PV;
    double d_I_PV;
    double d_P_PV;
    double ipv[2];
    double ppv[2];
    int sP, sI;   //Signs +/-
    bool EN;
    double AOK;
    pinMode(3,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(6,INPUT);
    
    for(;;)
    {
     digitalWrite(4,HIGH);
     while(digitalRead(6)==LOW){  //Simulation Not Running Initialize Values
      ipv[0] = ipv[1] = 0;
      ppv[0] = ppv[1] = 0;
      d_I_PV = 0;
      d_P_PV = 0;
      K = 0.2;
      digitalWrite(10,HIGH);   //Observe
      }
      
      xQueuePeek(xQueue2, &EN, 0);   //Read Enable Signal From Queue1
      digitalWrite(10,LOW);    //Observe
      if(EN){
       // digitalWrite(3,HIGH);
        I_PV = analogRead(A4);
        I_PV = map(I_PV,0,4070,0,45);
        //Serial.println(I_PV);
        V_PV = analogRead(A5);
        V_PV = map(V_PV,0,4070,0,60);
        //Serial.println(V_PV);
        ipv[0] = ipv[1];
        ipv[1] = I_PV;
     
        ppv[0] = ppv[1];
        ppv[1] = I_PV*V_PV;
        

        d_I_PV = ipv[1]-ipv[0];
        d_P_PV = ppv[1]-ppv[0];
       
        //**Calculate Signs of d_I_PV and d_P_PV

        if(d_I_PV<0){
        sI = -1;
        }
     
        if(d_I_PV>0){
        sI = 1;
        }
     
  
        if(d_P_PV<0){
        sP = -1;
        }
          if(d_P_PV>0){
          sP = 1;
          }
          
        
        //**************************************//
       // Serial.println(sP);
        K = K+dK*sP*sI;
        if(K>=4) K=4;
        if(K<0) K=0;
        AOK = K*100;
        AOK = map(AOK,0,400,0,4090);
       // Serial.println(K);
        analogWrite(DAC1,AOK);
        //Serial.println(K);
        xQueueOverwrite( xQueue, &K);
    }
    else{
      //Do Nothing if EN == False
      //digitalWrite(3,LOW);
    }
    
    digitalWrite(4,LOW);
}
}



