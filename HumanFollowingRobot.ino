#include <NewPing.h>

#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 13

#define MAX_FORWARD_MOTOR_SPEED 100
#define MAX_BACKWARD_MOTOR_SPEED -100
#define MAX_MOTOR_TURN_SPEED_ADJUSTMENT 50

#define MIN_DISTANCE 2
#define MAX_DISTANCE 30

#define IR_SENSOR_RIGHT 3
#define IR_SENSOR_LEFT 2

//Left motor
int enableLeftMotor=9;
int leftMotorPin1=6;
int leftMotorPin2=7;

//Right motor
int enableRightMotor=10;
int rightMotorPin1=4;
int rightMotorPin2=5;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);

void setup() {

  Serial.begin(9600);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  rotateMotor(0,0); 
    
}

void loop() {
  int distance = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT); // high = no hand sensed
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);  //low = senses hand

  if (distance <= 3) {
    
     rotateMotor(MAX_BACKWARD_MOTOR_SPEED, MAX_BACKWARD_MOTOR_SPEED);
     Serial.println("backward");
     Serial.println(distance);
     
  } else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW && distance >= 3 && distance <= 20) {
      
     rotateMotor(MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT );
     Serial.println("right");
     Serial.println(distance);
      
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH && distance >= 3 && distance <= 20) {
    
     rotateMotor(MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT);
     Serial.println("left");
     Serial.println(distance);

  } else if (distance > 3 && distance <= 20) {
      
     rotateMotor(MAX_FORWARD_MOTOR_SPEED, MAX_FORWARD_MOTOR_SPEED);
     Serial.println("forward");
     Serial.println(distance);
      
  } else {
    
     rotateMotor(0, 0);
     Serial.println("stop");   
      
  }
}

void pause(){
  
  digitalWrite(leftMotorPin1,LOW);
  digitalWrite(leftMotorPin2,LOW);
  analogWrite(enableLeftMotor, 0);
  digitalWrite(rightMotorPin1,LOW);
  digitalWrite(rightMotorPin1,LOW);
  analogWrite(enableRightMotor, 0);
  delay(2000);
}

void forward(){
  pause();
  //left motor
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  //right motor
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  
  //speed control
  analogWrite(enableRightMotor, 100); //0-255
  analogWrite(enableLeftMotor, 100); //0-255
}

void backward(){
  pause();
  //left motor
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  //right motor
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  
  //speed control
  analogWrite(enableRightMotor, 100); //0-255
  analogWrite(enableRightMotor, 100); //0-255
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
