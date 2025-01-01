#include "Arduino.h"
#include "config.h"
#include "SparkFun_TB6612.h"

int SENSOR_ARRAY [NO_SENSORS] = {A0, A2, A3, A6, A7}; //ADC Pins settings
int SENSOR_VALUES [NO_SENSORS]; //store the read data
int READ_VALUES [NO_SENSORS]; // make the read data in correct order
int temp_value,Startbtn;
int run; //ISR change this value to make robot stop or not.

//PID settings
int error,prevError,P,I,D,PID;
float Kp = 8;  // 15
float Kd = 35; // 5
float Ki = 1;  // 1

//offset values for driver
const int offsetA = 1;
const int offsetB = 1;

//defining the two motor objects
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); //right
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); //left

void set_pins(); //set the pins as input
void read_ir(); //read ir sensors
void startProgram();
void haltProgram();
void errorSet();
void PIDset();
void following();
void turnBack();

void setup() {
  Serial.begin(9600);
  set_pins(); 

  pinMode(EXT_LED, OUTPUT);

  pinMode(BUTTON_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), startProgram, RISING);

  pinMode(BUTTON_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), haltProgram, RISING);

  for (int i = 0; i < NO_SENSORS; i++) {
    SENSOR_VALUES[i] = analogRead(SENSOR_ARRAY[i]);
  }

  error = 0;
  PID = 0;
  prevError = 0;
}

void loop() {
 
  if(run == 1){
    read_ir();
    errorSet();
    PIDset();
    following();

    //forward(motor1,motor2);
  }else if(run == 0){
    brake(motor1,motor2);
  }
  
}

void startProgram(){
  run = 1;
  digitalWrite(EXT_LED, HIGH);
}

void haltProgram(){
  run = 0;
  digitalWrite(EXT_LED, LOW);
}

void following(){
  //in this function motor speed will be corrected.
  int leftmotor = M_BASE_SPEED + PID;
  int rightmotor = M_BASE_SPEED - PID;

  motor1.drive(rightmotor);
  motor2.drive(leftmotor);
}

void errorSet() {

  if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 1 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = 0;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 1 && READ_VALUES[3]== 1 && READ_VALUES[4]== 0){
      error = 1;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 0 && READ_VALUES[3]== 1 && READ_VALUES[4]== 0){
      error = 2;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 0 && READ_VALUES[3]== 1 && READ_VALUES[4]== 1){
      error = 3;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 0 && READ_VALUES[3]== 0 && READ_VALUES[4]== 1){
      error = 4;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 0 && READ_VALUES[2]== 1 && READ_VALUES[3]== 1 && READ_VALUES[4]== 1){
      error = 5;                                                                                                          // upto here right side
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 1 && READ_VALUES[2]== 1 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = -1;
  }else if(READ_VALUES[0]== 0 && READ_VALUES[1]== 1 && READ_VALUES[2]== 0 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = -2;
  }else if(READ_VALUES[0]== 1 && READ_VALUES[1]== 1 && READ_VALUES[2]== 0 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = -3;
  }else if(READ_VALUES[0]== 1 && READ_VALUES[1]== 0 && READ_VALUES[2]== 0 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = -4;
  }else if(READ_VALUES[0]== 1 && READ_VALUES[1]== 1 && READ_VALUES[2]== 1 && READ_VALUES[3]== 0 && READ_VALUES[4]== 0){
      error = -5;                                                                                                          // upto here left side
  } else if (READ_VALUES[0] == 1 && READ_VALUES[1] == 1 &&
             READ_VALUES[2] == 1 && READ_VALUES[3] == 1 &&
             READ_VALUES[4] == 1) {
    haltProgram();
  } else {
    
  }

  if(abs(error)> MAX_ERROR){
    brake(motor1,motor2);
    run = 0;
  }
}

void set_pins(){
  for (int i = 0; i<NO_SENSORS; i++){
    pinMode(SENSOR_ARRAY[i], INPUT);
  }
}

void read_ir(){
  for (int j = 0; j<NO_SENSORS; j++){
    temp_value = analogRead(SENSOR_ARRAY[j]);
    SENSOR_VALUES[j] = temp_value;
    if(SENSOR_VALUES[j]>THRESHOLD_VALUE){
      READ_VALUES[j] = 1;
    }else{
      READ_VALUES[j] = 0;
    }
  }

  // for (int i = 0; i<NO_SENSORS; i++){
  //   Serial.print(READ_VALUES[i]);
  //   Serial.print(",");
  // }

  // Serial.println();
}

void PIDset(){
  P = error;
  I = I + error;

  if (I > MAX_I) I = MAX_I;       // Limit upper bound
  if (I < -MAX_I) I = -MAX_I;     // Limit lower bound

  D = error-prevError;

  PID = (Kp*P) + (Ki*I) + (Kd*D);
  prevError = error;
}