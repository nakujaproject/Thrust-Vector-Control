#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Wire.h>

//Creating MPU object
MPU6050 mpu(Wire);

//Servos to gimbal the thrust vector
Servo servoX;
Servo servoY;

//Variables that hold the angles recorded by the IMU
float angleX;
float angleY;
float angleZ;

//PID constants manual tuning
float Kp = 0.50;
float Ki = 0.50;
float Kd = 0.50;


long timer = 0;
long previousMillis = 0;
long t;

float X, Y;

float setPoint = 0.00;
float errorX;
float errorY;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);
}

void loop() {
  timer = (millis()/1000);
  
  mpu.update();
  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();

  //Tuning of PID constants using potentiometers
  //The values are mapped to get to a range of between 0 and 255 to lie between -10 and 10
//  Kp = map(analogRead(A0), 0, 1023, -10 , 10); 
//  Ki = map(analogRead(A1), 0, 1023, -10 , 10); 
//  Kd = map(analogRead(A2), 0, 1023, -10 , 10); 

  Kp = analogRead(A0)/100;
  Ki = analogRead(A1)/100;
  Kd = analogRead(A2)/100;

  
  Serial.print(Kp);
  Serial.print("<--Kp----Ki-->");
  Serial.print(Ki);
  Serial.print("<---Ki---Kd-->");
  Serial.print(Kd);
  Serial.print("<Kd---");
  Serial.println();
  errorX = -1*setPoint + angleX;
  X = (Kp + Ki*t + Kd/t)*errorX;
  Serial.print("X");
  Serial.print(angleX);
  Serial.print("----------");
  Serial.println(X);
  if(X <= 0) X = 0;
  if(X >= 180) X = 180;
  servoX.write(X);

  
  errorY = (-1*setPoint + angleY);
  Serial.print("Y");
  Serial.print(angleY);
  Serial.print("----------");
  Serial.println(Y);
  if(Y <= 0) X = 0;
  if(Y >= 180) X = 180;
  Y = (Kp + Ki*t + Kd/t)*errorY;
  servoY.write(Y);

  delay(1000);
  previousMillis = timer - previousMillis;
  Serial.println("time =");
  t = previousMillis;
  Serial.println(t);
  timer = 0;
}
