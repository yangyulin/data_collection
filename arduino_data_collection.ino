#include <Encoder.h>
#include<elapsedMillis.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//we connected pin 3 connected parallel to the positive (red) input to the motor
//and pin 2 connected parallel to the negative (black) input to the motor
//by subtracting pin 3 and pin 2 we can recover the voltage inputed to the motor
//this is especially useful when motor is driven by negative voltage
int analogPositivePin = 3;     
int analogNegativePin = 2;
int input_positive = 0;           
int input_negative = 0;   
        
long encoder_pos = 0;
Encoder my_enc(2, 3);

//elapsedMills is an external library
//it creates a counter that increases by 1 for each millisecond passed
//so by setting it to zero and read it later, one can obtain the time passed
elapsedMillis sample_timer;
int sample_interval=2;

void setup()
{
  //  setup serial, ensure that same baud rate is used here and in python code
  Serial.begin(57600);          
  sample_timer = 0;
}



void loop()

{
  //sample loop that runs at (1000/sample_interval)Hz
  //make sure you have same frequency defined here and in the python code
  if (sample_timer >= sample_interval) {
    input_positive = analogRead(analogPositivePin);
    input_negative = analogRead(analogNegativePin);
    encoder_pos = my_enc.read();
    //each line's format: "voltage_in_digits encoder_position"
    Serial.print(input_positive-input_negative);
    Serial.print(" ");
    Serial.println(encoder_pos);
    //reset sample_time to zero and compensate any time lost in reading the ADC and encoder
    sample_timer = sample_timer-sample_interval;
  }  
}
