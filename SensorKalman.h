#ifndef _Kalman_h
#define _Kalman_h

class SensorKalman {
  public:
    SensorKalman(){
        Q_moisture_1=0.01;
        Q_moisture_2=0.01;
        R_measure=1;

        moisture_1=0; //reset the moisture_1
        moisture_2=0; //reset the moisture_1
        P1=0;  //initial covariance matrix     
        P2=0;  //initial covariance matrix     
    }
   double getmoisture_1(double newmoisture_1, double dt){
       P1+=Q_moisture_1*dt;  //estimation error covariance
       //Kalman gain
       S1=P1+R_measure;
       K1=P1/S1;
       //Update whith measurement 
       y1=newmoisture_1-moisture_1;
       //Calculate moisture
       moisture_1+=K1*y1;
       //Update the error covariance
       P1*=(1-K1);
       return moisture_1;
   };
   double getmoisture_2(double newmoisture_2, double dt){
       P2+=Q_moisture_2*dt;  //estimation error covariance
       //Kalman gain
       S2=P2+R_measure;
       K2=P2/S2;
       //Update whith measurement 
       y2=newmoisture_2-moisture_2;
       //Calculate moisture
       moisture_2+=K2*y2;
       //Update the error covariance
       P2*=(1-K2);
       return moisture_2;
   };

   
   void setmoisture_1(double newmoisture_1){ moisture_1 = newmoisture_1;};
   void setmoisture_2(double newmoisture_2){ moisture_2 = newmoisture_2;};

   double getQmoisture_1(){return Q_moisture_1;};
   double getQmoisture_2(){return Q_moisture_2;};

   void setQmoisture_1(double newQ_moisture_1){Q_moisture_1=newQ_moisture_1;};
   void setQmoisture_2(double newQ_moisture_2){Q_moisture_2=newQ_moisture_2;};

   double getRmeasure(){return R_measure;};
   void setRmeasure(double newR_measure){R_measure=newR_measure;}; 
  private:
   double Q_moisture_1;  
   double Q_moisture_2;  
   double R_measure; 
   double moisture_1;
   double moisture_2;
   double P1;
   double K1;
   double y1;
   double S1;
   double P2;
   double K2;
   double y2;
   double S2;

};

#endif
