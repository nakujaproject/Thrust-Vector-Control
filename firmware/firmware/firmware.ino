#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>

Servo servoX;
Servo servoY;
MPU6050 mpu(Wire);

double kp = 0.2;
double ki = 0.3;
double kd = 0.25;  //0.25

double currentTime = 0; 
double previousTime = 0;
double elapsedTime;
double errorX, errorY;
double lastErrorX, lastErrorY;
double cumErrorX, cumErrorY, rateErrorX, rateErrorY;
double setPointX, setPointY;
double inputX, inputY, outputX, outputY;
 
void setup(){
        Serial.begin(9600);
        servoX.attach(5);
        servoY.attach(6);
        servoX.write(93);
        servoY.write(93);
        Wire.begin();
        mpu.begin();
        mpu.calcGyroOffsets(true);
        setPointX = 0;                          //desired set point for X angle
        setPointY = 0;                          //desired set point for Y angle
}    
 
void loop(){
        mpu.update();
        currentTime = millis();
        elapsedTime = currentTime - previousTime;
        
        inputX = mpu.getAngleX();                
        inputY = mpu.getAngleY();

        errorX = inputX - setPointX;
        errorY = setPointY - inputY;


        cumErrorX = cumErrorX + errorX * elapsedTime;                // compute integral
        rateErrorX = (errorX - lastErrorX)/(elapsedTime);   // compute derivative
//        Serial.println(cumErrorX);
//        Serial.println("elapsedTime below");
//        Serial.println(elapsedTime);
//        Serial.println("kd");
//        Serial.println(kd);
//        Serial.println("kd");
        //kd = map(analogRead(A0), 0, 1023, -10, 15);
        cumErrorY = cumErrorY + errorY * elapsedTime;                // compute integral
        rateErrorY = (errorY - lastErrorY)/(elapsedTime);   // compute derivative
 
        outputX = kp*errorX + ki*cumErrorX + kd*rateErrorX;                //PID output               
 
        lastErrorX = errorX;                                //remember current error

         
        outputY = kp*errorY + ki*cumErrorY + kd*rateErrorY;                //PID output               
 
        lastErrorY = errorY; 
        
        
        previousTime = currentTime;

        if(outputX > 90){
            outputX = 90;
          }
    
        if(outputX < -90){
            outputX = -90;
          }
    
        if(outputY > 90){
            outputY = 90;
          }
    
        if(outputY < -90){
            outputY = -90;
          }
      
        servoX.write(outputX + 93);                
        servoY.write(outputY + 93);
        
        Serial.println(mpu.getAngleX());
        Serial.println(mpu.getAngleY());
        Serial.println("YYY"); 
}
