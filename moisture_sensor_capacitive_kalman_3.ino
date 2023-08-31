#include "SensorKalman.h"
#define SENSORPIN A0 
#define IND_OP 13
#define SEN_IN 2
#define PUMP_OUT 5
#define RESET_RESERVOIR 7
#define RESERVOIR_RESET_INDICATION 8

const int IN_A0=A0;
const int IN_A1=A1;
const int LIMIT=600;
const int PUMP_RUNS_LIMIT=4;// limit how long the pump runs, until it seems that 
//it doesn't change to wet within the lime limit


uint32_t timer=0;
double dt=0;

double value_A0;
double value_A0_raw;
double value_A1;
double value_A1_raw;
//int start_counter;
String prev_status_flower="dry";
String current_status_flower="dry";
String prev_status_reservoir="dry";
String current_status_reservoir="dry";
String constant_dry="dry";
int wet_wait_time=10;
int dry_wait_time=5;
int reset_button_state=0;
int trigger_reset_button_state=0;
int startflag=0;
int wet_count=0;
int flower_dry_count=0;
int pumpRun_count=0;

SensorKalman sensor;

void setup ()
      {
     Serial.begin(115200);
     pinMode(IND_OP, OUTPUT);
     pinMode(4, OUTPUT);
     pinMode(PUMP_OUT, OUTPUT);
     pinMode(RESERVOIR_RESET_INDICATION, OUTPUT);
     pinMode(SEN_IN, INPUT);
     pinMode(RESET_RESERVOIR, INPUT);
     pinMode (IN_A0,INPUT);
     pinMode (IN_A1,INPUT);
     timer=micros(); //update timer
      }

  void loop()
      {
      dt=(double)(micros()-timer)/1000000; //calculate differential
      timer=micros(); //update timer
      value_A0_raw=analogRead(IN_A0);
      value_A0=sensor.getmoisture_1(value_A0_raw, dt);

      delay(500);
      if(value_A0>460){
        current_status_flower="dry";digitalWrite(4,LOW);}
        else{current_status_flower="wet";digitalWrite(4,HIGH);}
      value_A1_raw=analogRead(IN_A1);
      value_A1=sensor.getmoisture_2(value_A1_raw, dt);
  
  
     if(value_A1>460){
      current_status_reservoir="dry";}
      else{current_status_reservoir="wet";}
      trigger_reset_button_state=digitalRead(RESET_RESERVOIR);
      if(trigger_reset_button_state==1){digitalWrite(RESERVOIR_RESET_INDICATION,HIGH);}
      else{digitalWrite(RESERVOIR_RESET_INDICATION,LOW);}
      
      //flower sensor status change detection
      if(prev_status_flower=="dry"&&current_status_flower=="wet"){
        Serial.println("flower dry -> wet ");
        flower_dry_count=0;
        }
      if(prev_status_flower=="wet"&&current_status_flower=="dry"){
        Serial.println("flower wet -> dry, ");}
      
      //reservoir sensor status change detection
      if(prev_status_reservoir=="dry"&&current_status_reservoir=="wet"){
        Serial.println("reservoir dry -> wet ");
        flower_dry_count=0;
        }
      if(prev_status_reservoir=="wet"&&current_status_reservoir=="dry"){
        Serial.println("reservoir wet -> dry, ");}
  //DECISION 
      if((current_status_flower.equals("dry") == 1)&&(current_status_reservoir.equals("wet") == 1&&pumpRun_count<35)){
             digitalWrite(PUMP_OUT, LOW);//valve open
              pumpRun_count++;
        }
     else{digitalWrite(PUMP_OUT, HIGH);pumpRun_count=0;}
/*
    Serial.print(current_status_flower);
    Serial.print(value_A0);
 
    Serial.print("   ");
    Serial.print(current_status_reservoir);
    Serial.print("   ");
    Serial.print(value_A1);
    Serial.print("   ");
    Serial.print(pumpRun_count);
    Serial.print("   ");
    Serial.println(trigger_reset_button_state);
   */ 
    
    prev_status_flower=current_status_flower;
    prev_status_reservoir=current_status_reservoir;

      
      Serial.print(value_A0_raw);
      Serial.print(' ');
      Serial.println(value_A0);

      
      
      
      }
