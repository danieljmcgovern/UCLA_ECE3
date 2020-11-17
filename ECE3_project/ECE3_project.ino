
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

int pause = 0;

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

  pinMode(PUSH1, INPUT_PULLUP);
  attachInterrupt(PUSH1, calibrate_min, FALLING);
  pinMode(PUSH2, INPUT_PULLUP);
  attachInterrupt(PUSH2, calibrate_max, FALLING);  

  delay(2000);
}

int leftSpd = 0;
int rightSpd = 0;
int constSpd = 20;  //starting motor speed that PD controller will modify

float error = 0;
float error_prev = 0;
float derivative = 0;

float kp = 0.10;    //constant of proportionality
float kd = 0.10;    //constant of differentiation

int count = 0;
int raw_sum = 0;

void loop()
{  
  weighted_sum = 0; //this value must be zeroed out becuase it is only ever added to in the for loop, ie it's value is never reset (as is error's value)
   
  ECE3_read_IR(SVraw);   // read raw sensor values

  for (int i = 0; i<8; i++){    //normalize the raw sensor data
    SVnorm[i] = (SVraw[i] - SVmin[i])*(1000.0/SVmax[i]);
  }
  for (int i = 0; i<8; i++){    //compute weighted sum on normalized value
    weighted_sum += SVnorm[i]*weight_8421[i];
  }
  error = weighted_sum/8.0;

  derivative = error - error_prev;

  leftSpd = constSpd - kp*error - derivative*kd;
  rightSpd = constSpd + kp*error + derivative*kd;  
 
  error_prev = error;
  
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);

  //code to execute 180 on reaching 1st black strip
  raw_sum = 0;  
  for(int i = 0; i < 8; i++){
    raw_sum += SVraw[i];
  }
   if((raw_sum / 8)> 2000 && (count == 0)){
    digitalWrite(right_dir_pin, HIGH);      //reverse direction of right motor
    analogWrite(left_pwm_pin,30);
    analogWrite(right_pwm_pin,30);          //execute turn for duration of delay
    delay(2100);
    digitalWrite(right_dir_pin, LOW);
    raw_sum = 0;                              //reset raw_sum so next if statment doesn't immediately execute
    count++;
  }
  //code to detect the 2nd strip, end of track
  if((raw_sum / 8)> 2000 && (count == 1)){
    analogWrite(left_pwm_pin, 0);           //stop the car
    analogWrite(right_pwm_pin, 0);
    delay(10000);
  }
  
    
}

//TODO FIX: calibrate_min can only run once, not sure why. calibrate_max can run multiple times
void calibrate_min() {
  ECE3_read_IR(SVraw);              //read raw sensor data
  for (int i = 0; i<8; i++){       //zero out SVmin array
    SVmin[i] = 0;
  }
  for (int i = 0; i<10; i++){     //taking 10 readings per sensor
    for (int i = 0; i<8; i++)     //take a reading from each sensor
      SVmin[i] += (SVraw[i]/10.0);//ten values going in, so divide each value by 10 for average
  }
  for (int i = 0; i<8; i++){
    Serial.print(SVmin[i]); Serial.print("\t");
  }
  Serial.println();
  delay(2000);
}
void calibrate_max() {
  ECE3_read_IR(SVraw);              //read raw sensor data
  for (int i = 0; i<8; i++){       //zero out SVmax array
    SVmax[i] = 0;
  }
  for (int i = 0; i<10; i++){     //taking 10 readings per sensor
    for (int i = 0; i<8; i++)     //take a reading from each sensor
      SVmax[i] += (SVraw[i]/10.0);//ten values going in, so divide each value by 10 for average
  }
  for (int i = 0; i<8; i++){
    SVmax[i] -= SVmin[i];         //subtract min values
  }
  for (int i = 0; i<8; i++){
    Serial.print(SVmax[i]); Serial.print("\t");
  }
  Serial.println();
  delay(2000);
}
