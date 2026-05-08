#include <Arduino.h>
// CHANGE THE PINS LATER 
const int INPUT_PIN = A0; 
const int OUTPUT_PIN = 3;   // FIXED: DD3 is not valid, changed to 3
const int POSTION = A3;      // FIXED: DD4 is not valid, changed to 4
const int button1 = 9;
const int button2 = 5;      // FIXED: was 3, conflicted with OUTPUT_PIN
const int button3 = 6;      // FIXED: was 4, conflicted with POSTION pin
const int MOTOR_PWM  = 3;   // PWM speed pin  (EN or PWM pin on your driver)
const int MOTOR_IN1  = 7;   // Direction pin 1
const int MOTOR_IN2  = 8;   // Direction pin 2
const int LSR = 10;         //limitimg switch right tregger 
const int LSL = 11;          //limiting switch left tregger 


const int ENC_A = 2;   // Encoder channel A — must be an interrupt pin
const int ENC_B = 4;   // Encoder channel B


double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 0.00;
double rodEnd = 1023, rodBeginning = 0;  // FIXED: 'end' is a reserved word in C++, renamed to rodEnd/rodBeginning and given default values
float prev_theta = 0;  
float prev_pos   = 0;

volatile long encoderCount = 0;   // counts pulses; volatile because it is updated in an ISR


float M;  //mass of the cart 
float g = 9.81; //gravitational accelaration 
float m;  //mass of the rod
float leigth; // liegth of the rod

const float K[] = {-15.4, -2.1, -0.5, -0.2}; // gains for a lqr controller
const float limit = 255.0;  // ADDED: max limit used for mapping 'u' in LQR block

double pid(double error);
double bangbang(float theta);      // ADDED: forward declaration was missing
void applyMotorPower(int pwm);     // ADDED: forward declaration was missing

// ADDED: simple stub sensor functions so the code compiles.
// Replace the body of each with your real sensor reading logic later.
float getAngle()           { return (analogRead(A1) / 1023.0) * 360.0; }
float getAngularVelocity(float theta);
float getPosition()        { return analogRead(A2); }
float getVelocity(float pos);

void encoderISR() {
  if (digitalRead(ENC_B) == HIGH) {
    encoderCount++;    // forward rotation
  } else {
    encoderCount--;    // reverse rotation
  }
}

void setup()
{
  // gains of the PID controller 
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = 0;

  //intializing the buttons
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);   // encoder pin A with internal pull-up
  pinMode(ENC_B, INPUT_PULLUP);   // encoder pin B with internal pull-up
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);  // attach ISR to channel A
  analogWrite(MOTOR_PWM, 0);      // start with motor off

  pinMode(LSR, INPUT);
  pinMode(LSL, INPUT);

  Serial.begin(9600);
  
  for(int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

void loop()
{
  // FIXED: was missing '//' so it was not a comment, caused compile error
  // Get current states from sensors (IMU, Encoders, etc.)
  float theta = getAngle();
  float theta_dot = getAngularVelocity(theta);
  float pos = getPosition();    // FIXED: removed duplicate 'double pos' declaration below
  float vel = getVelocity(pos);

  //state of the buttons 
  int button1State = digitalRead(button1);
  int button2State = digitalRead(button2);
  int button3State = digitalRead(button3);

  //limitimg switch states 
  int stateLimit_R = digitalRead(LSR);
  int stateLimit_L = digitalRead(lSL);

  //if the motor touches the right or the left switch it should move to the other direction
  if(stateLimit_R == HIGH)
  {
    applyMotorPower(-10);
  }
  else if (stateLimit_L == HIGH)
  {
    applyMotorPower(10);
  }

  
  //reading the time in milliseconds 
  double now = millis();
  dt = ( now - last_time)/1000.00;
  last_time = now;

  //Calculate Control Input (u = -Kx)
  float u = -(K[0]*theta + K[1]*theta_dot + K[2]*pos + K[3]*vel);

  //mapping the position of the cart 
  // FIXED: removed 'double pos' re-declaration, now just reassigns the existing pos variable
  pos = map(analogRead(POSTION), 0, 1023, rodBeginning, rodEnd);  // FIXED: 1024 -> 1023 (analogRead max is 1023)
  // NEW — use the encoder count directly:
  //pos = encoderCount;   // raw pulse count; scale it later once you know your encoder resolution


  double actual = 0;  // ADDED: declared here so it is accessible for the Serial print below

  //the if statement makes sure the cart is not in the begining of the cart or the end
  // FIXED: changed || to && so the condition works correctly
  if(pos != rodEnd && pos != rodBeginning){

    //PID controller 
    if(button1State == HIGH){
      
      actual = map(analogRead(INPUT_PIN), 0, 1023, 0, 255);  // FIXED: 1024 -> 1023
      double error = setpoint - actual;
      output = pid(error);

      // output voltage to the motor 
      applyMotorPower((int)output - 127);
    }
    
    //the lqr controller  
    else if (button2State == HIGH){

      //Apply to actuators (mapping 'u' to PWM)
      int pwm = constrain(map(u, -limit, limit, -255, 255), -255, 255);
      applyMotorPower(pwm);
    }

    //the bang-bang controller 
    else if (button3State == HIGH){
      double output = bangbang(theta);
      applyMotorPower((int)output - 127);
    }

    // Setpoint VS Actual
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(actual);
  }

  // Error
  //Serial.println(error);

  delay(300);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

//bang-bang function
// FIXED: filled in the empty if/else bodies and added threshold values
// ADDED: threshold = 0.1 radians as a small deadband around zero.
//        Returns 255 (full forward) if leaning one way, 0 (full reverse) if leaning the other.
//        Change the threshold and return values to match your motor and angle units.
double bangbang(float theta)
{
  if(theta >= 0.1){
    return 255;
  }
  else if (theta <= -0.1){
    return 0;
  }
  return 127;  // ADDED: deadband center — motor holds neutral if theta is near zero
}

// ADDED: applyMotorPower stub.
// Replace the body with your real motor driver calls (e.g. direction pin + PWM).
void applyMotorPower(int pwm)
{
  int absPwm = abs(pwm);
  absPwm = constrain(absPwm, 0, 255);   // safety clamp

  if (pwm > 0) {
    digitalWrite(MOTOR_IN1, HIGH);      // forward
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, absPwm);
  }
  else if (pwm < 0) {
    digitalWrite(MOTOR_IN1, LOW);       // reverse — IN1/IN2 flipped
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_PWM, absPwm);
  }
  else {
    digitalWrite(MOTOR_IN1, LOW);       // brake / stop
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_PWM, 0);
  }
}

float getAngularVelocity(float theta)
{
  float dir = (theta - prev_theta) / dt;
  prev_theta = theta;
  return dir;
}

float getVelocity(float pos)
{
  float dir = (pos - prev_pos) / dt;
  prev_pos = pos;
  return dir;
}


