#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

double kpX = 2;
double kiX = 5;
double kdX = 1;

double kpY = 2;
double kiY = 5;
double kdY = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double errorX, errorY;
double lastErrorX, lastErrorY;
double inputX, inputY, outputX, outputY, setPointX, setPointY;
double cumErrorX, cumErrorY, rateErrorX, rateErrorY;

Servo servoX;
Servo servoY;

void setup(){
  servoX.attach(3);
  servoY.attach(4);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  setPointX = 0;//set point at zero degrees
  setPointY = 0;
  
}    
 
void loop(){
     inputX = analogRead(A0);                //read from rotary encoder connected to A0
     inputY = analogRead(A1);
     outputX = computePIDX(inputX);
     outputY = computePIDY(inputY);

     if(outputX > 180){
        outputX = 180;
      }

     if(outputX < 0){
        outputX = 0;
      }

     if(outputY > 180){
        outputY = 180;
      }

     if(outputY < 0){
        outputY = 0;
      }
      
     servoX.write(outputX);                //control the motor based on PID value
     servoY.write(outputY);
}
 
double computePIDX(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPointX - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}


double computePIDY(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPointY - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kpY*error + kiY*cumError + kdY*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
