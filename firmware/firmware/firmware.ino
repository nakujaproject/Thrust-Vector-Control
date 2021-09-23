#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>

Servo servoX;
Servo servoY;
MPU6050 mpu(Wire);

double kp = 2;
double ki = 5;
double kd = 1 ;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, rateError;
double setPointX, setPointY;
double inputX, inputY, outputX, outputY;
 
void setup(){
        servoX.attach(5);
        servoY.attach(6);
        Wire.begin();
        mpu.begin();
        mpu.calcGyroOffsets(true);
        setPointX = 0;                          //desired set point for X angle
        setPointY = 0;                          //desired set point for Y angle
}    
 
void loop(){
        inputX = mpu.getAngleX();                //read from rotary encoder connected to A0
        inputY = mpu.getAngleY();
        outputX = computePIDX(inputX);
        outputY = computePIDY(inputY);
        delay(10);
        servoX.write(outputX);                //control the motor based on PID value
        servoY.write(outputY);
 
}
 
double computePIDX(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPointX - inputX;                                // determine error
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
        
        error = setPointY - inputY;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
