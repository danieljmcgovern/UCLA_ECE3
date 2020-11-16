

#include <ECE3.h>

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

uint16_t SVraw[8];

float weighted_sum = 0;
float weight_8421[8] = {-8.0, -4.0, -2.0, -1.0, 1.0, 2.0, 4.0, 8.0}; 

//these values are from trial3 fusion 11-14-20
float SVmin[8] = {563.2, 586.0, 654.2, 496.0, 585.2, 586.0, 585.8, 677.2};            //min values from the sensor calibration test, {SV1, SV2, ... SV8}
float SVmax[8] = {1936.8, 1914.0,  1843.8, 1374.4, 1491.2, 1886.6, 1606.6, 1822.8};
float SVnorm[8];  //SVnorm = (SVraw - SVmin)*(1000/SVmax)

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission  

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  //this configuration sets the motors to move the car forward
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);

  delay(2000);
}

int leftSpd = 20;
int rightSpd = 20;
int constSpd = 20;  //using this because leftSpd will no longer be a constant reference point.

float error = 0;
float error_prev = 0;
float derivative = 0;

float kp = 0.10;    //constant of proportionality
float kd = 0.10;    //constant of differentiation
//kp

void loop()
{
  weighted_sum = 0; //this value must be zeroed out becuase it is only ever added to in the for loop, ie it's value is never reset (as is error's value)
  
  // read raw sensor values
  ECE3_read_IR(SVraw);

  for (int i = 0; i<8; i++){    //normalize the raw sensor data
    SVnorm[i] = (SVraw[i] - SVmin[i])*(1000.0/SVmax[i]);
  }
  for (int i = 0; i<8; i++){    //compute weighted sum on normalized value
    weighted_sum += SVnorm[i]*weight_8421[i];
  }
  error = weighted_sum/8.0;

  derivative = error - error_prev;

//BIG CHANGE for sketch_nov15c: if(error<0) change leftSpd
  if(error>0){
    leftSpd = constSpd;
    rightSpd = constSpd + error*kp + derivative*kd;
  }
  if(error<0){
    rightSpd = constSpd;
    leftSpd = constSpd + error*(-kp) + derivative*(-kd);
  }
  // leftSpd = constleft - kp*fusion
  //  rightspd = constrigth + kp*fusion

  

  //printing the output of the PD controller to the serial monitor to validate results. once validated will send speed changes to motors instead of printing them.
  /*
  Serial.print("error: ");   Serial.println(error); 
  Serial.print("E*kp: ");   Serial.println(error*kp);
  Serial.print("D*kd: ");   Serial.println(derivative*kd);
  Serial.print("leftSpd: "); Serial.println(leftSpd);
  Serial.print("rightSpd: "); Serial.println(rightSpd);
  Serial.println();*/
 
  error_prev = error;
  delay(50);    //TODO: how long of a delay is necessary? is it necessary? how rapidly do sensor values come in?
  
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
}
