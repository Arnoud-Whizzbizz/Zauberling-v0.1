// Whizzbizz.com - Arnoud van Delden - august/september 2021
//
// 'Zauberling' is build with a Arduino Pro Mini Atmega328P 5V 16Mhz board.
// Sketch to be uploaded with FTDI FT232RL USB To TTL Serial IC Adapter Converter
// or other USB to TTL interface (e.g. Arduino Uno with chip removed)

// For the TB6612 was used: https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

#include <SparkFun_TB6612.h>
#include <Servo.h>

// TB6612 pins definition
#define AIN1 2
#define AIN2 3
#define PWMA 5
#define BIN1 A4
#define BIN2 A5
#define PWMB 6
#define STBY 13

#define IN1  A6
#define IN2  A7
#define IN3  A2
#define POTMETER A3
#define TRESHOLD_MAX 30
#define TRESHOLD_FIXED 15 // Fixed sensor detection threshold for functions that use the potmeter for something else
#define DELAY_MAX 2500    // Max delay after sensor trigger or pulse width of monoflop in ms (0 ~ 2,5 sec)
#define SPEED_STEPS 30

#define Q1  12
#define Q2  A0
#define Q3  A1

// Inputs used to read the DIP-switches
#define DIP1  4
#define DIP2  7
#define DIP3  8
#define DIP4  9

// Servo PWM outputs and min/max of demo
#define SERVO1  10
#define SERVO2  11
#define SERVOMIN 60
#define SERVOMAX 120

#define MINREG 0    // Minimal analogue speed potmeter value
#define MAXREG 1023 // Maximal analogue speed potmeter value

bool logging = true; // Serial logging during development...

int currentMotorSpeed;
bool currentMotorDir = true; // true=forward(CW), false=backwards(CCW)
bool sensorLogicNegative = true; // Default is negative for fixed behaviour...
int in1Default, in2Default, in3Default;
int potValue;     // Value read from potentiometer
int sensorTreshold; // Analog trigger threshold value of potentiometer
int timeDelay;    // Delay or pulse width of monoflop in ms
int dipSwitches = 0;

// Initializing motor
Motor Out1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor Out2 = Motor(BIN1, BIN2, PWMB, 1, STBY);

// Initialize servos...
Servo Servo1;
Servo Servo2;
int servoPos = 90;

int in1Value;
int in1ValueTrigger;
int in2Value;
int in2ValueTrigger;
int in3Value;
int in3ValueTrigger;

int Q_FF = false; // Flipflop output state

bool in1Active        = false;
bool in1NegativeLogicConfirmed = false;
bool in1TriggerUsed   = false;

bool in2Active        = false;
bool in2NegativeLogicConfirmed = false;
bool in2TriggerUsed   = false;

bool in3Active        = false;
bool in3NegativeLogicConfirmed = false;
bool in3TriggerUsed   = false;

void logSerial(String message) {
  // Handled overhere so logging to serial can easily be switched off...
  if (logging) Serial.println(message);
  return;
}

void setup() {
  Serial.begin(38400);

  // Set pin-modes and read sensor defaults...
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(IN3, INPUT);
  pinMode(POTMETER, INPUT);
  //sensorTreshold = TRESHOLD_FIXED; // Fixed detection level...

  // Read program DIP-switches...
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  pinMode(DIP3, INPUT);  
  pinMode(DIP4, INPUT);
  dipSwitches = readdipSwitches();

  // Only for functions that need explicit logic level settings (i.e. the 'Basic Program')... 
  switch (dipSwitches) { // Switch different usage of potentionmeter...
    case 0: // Basic program negative logic...
      sensorLogicNegative = true;
      in1NegativeLogicConfirmed = in2NegativeLogicConfirmed = in3NegativeLogicConfirmed = false;
      reinitializeInputs();
      break;
    case 1: // Basic program positive logic...
      sensorLogicNegative = false;
      in1NegativeLogicConfirmed = in2NegativeLogicConfirmed = in3NegativeLogicConfirmed = true;
      reinitializeInputs();
      break;
    default:
      ; // No special needs, sensors are AUTO...
  }

  // Level LEDs...
  pinMode(Q1, OUTPUT);
  digitalWrite(Q1, LOW);
  pinMode(Q2, OUTPUT);
  digitalWrite(Q2, LOW);
  pinMode(Q3, OUTPUT);
  digitalWrite(Q3, LOW);
  
  // Init servos...  
  Servo1.attach(SERVO1);
  Servo1.write(servoPos);
  Servo1.write(90);
  Servo2.attach(SERVO2);
  Servo2.write(servoPos);
  Servo2.write(90);

  // Init motors/outputs
  currentMotorSpeed = 0;
  currentMotorDir = true; // Forward/CW
 
  logSerial("Setup OK...");
}

int readdipSwitches() {
  int DIPval;
  
  DIPval = digitalRead(DIP1);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP2);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP3);
  DIPval = DIPval << 1;
  DIPval += digitalRead(DIP4);
  logSerial("DIP-Switch:"+(String)DIPval);

  return(DIPval);
}

void reinitializeInputs() {
  // Read and set the default sensors/inputs values
  // For functions that use (active sensors with) negative logic the sensors MUST read >255 when untriggered. Because one of the range end
  // sensors may be activated/triggered on program start, they MUST be recalibrated later on (after motor move and sensor unblocked)

  if ((sensorLogicNegative && !in1NegativeLogicConfirmed) || (!sensorLogicNegative && in1NegativeLogicConfirmed)) {
    in1Default = analogRead(IN1);
    if (sensorLogicNegative && in1Default>255)
      in1NegativeLogicConfirmed = true;
    else
      in1NegativeLogicConfirmed = false;
  }
  
  if ((sensorLogicNegative && !in2NegativeLogicConfirmed) || (!sensorLogicNegative && in2NegativeLogicConfirmed)) {
    in2Default = analogRead(IN2);
    if (sensorLogicNegative && in2Default>255)
      in2NegativeLogicConfirmed = true;
    else
      in2NegativeLogicConfirmed = false;
  }
  
  if ((sensorLogicNegative && !in3NegativeLogicConfirmed) || (!sensorLogicNegative && in3NegativeLogicConfirmed)) {
    in3Default = analogRead(IN3);
    if (sensorLogicNegative && in3Default>255)
      in3NegativeLogicConfirmed = true;
    else
      in3NegativeLogicConfirmed = false;
  }

  logSerial("reinitializeInputs - in1NegativeLogicConfirmed:"+(String)in1NegativeLogicConfirmed+" - in2NegativeLogicConfirmed:"+(String)in2NegativeLogicConfirmed+" - in3NegativeLogicConfirmed:"+(String)in3NegativeLogicConfirmed);
}

void setMotorSpeed(bool rotateDir, int motorSpeed, bool smooth) {
  // rotateDir = true means O1/O2 CW rotation
  // rotateDir = false is O1/O2 CCW, output O3/O4 is reversed
  // motorSpeed is a value between 0 and 512
  // smooth = potmeter setting is value of gracefully descend and ascend
  int tempSpeed;
  int speedStep;

   if (rotateDir!=currentMotorDir || motorSpeed!=currentMotorSpeed) {  
    if (!smooth) { // Standard, simple reverse direction
      if (rotateDir) {
        Out1.drive(motorSpeed);
        Out2.drive(-motorSpeed);
      } else {
        Out1.drive(-motorSpeed);
        Out2.drive(motorSpeed);    
      }
    } else { // Speed up and down with smooth limits
      int smoothDelay = timeDelay/25;
      if (currentMotorDir == rotateDir) { // No directional changes...
        speedStep = (motorSpeed-currentMotorSpeed)/SPEED_STEPS;
        tempSpeed = currentMotorSpeed;
        do {
          tempSpeed += speedStep;
          if (abs(motorSpeed-tempSpeed) <= abs(speedStep)) tempSpeed = motorSpeed; // Last step...       
          //logSerial("No dir change - tempSpeed:"+(String)tempSpeed+" - currentMotorSpeed:"+(String)currentMotorSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          delay(smoothDelay);
          if (rotateDir) {
            Out1.drive(tempSpeed);
            Out2.drive(-tempSpeed);
          } else {
            Out1.drive(-tempSpeed);
            Out2.drive(tempSpeed);    
          }
        } while (abs(motorSpeed-tempSpeed) > abs(speedStep));
      } else { // Towards and through zero...
        for (tempSpeed=currentMotorSpeed; tempSpeed>0; tempSpeed -= (currentMotorSpeed/SPEED_STEPS)) { // Slow down to zero...
          //logSerial("To zero - tempSpeed:"+(String)tempSpeed+" - currentMotorSpeed:"+(String)currentMotorSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          delay(smoothDelay);
          if (currentMotorDir) {
            Out1.drive(tempSpeed);
            Out2.drive(-tempSpeed);
          } else {
            Out1.drive(-tempSpeed);
            Out2.drive(tempSpeed);    
          }         
        }
        speedStep = motorSpeed/SPEED_STEPS; // Always positive...
        for (tempSpeed=0; tempSpeed<=motorSpeed; tempSpeed += (motorSpeed/SPEED_STEPS)) { // Build up to motorSpeed again...
          if (abs(motorSpeed-tempSpeed) <= speedStep) tempSpeed = motorSpeed; // Last step...
          //logSerial("Build up - tempSpeed:"+(String)tempSpeed+" - currentMotorSpeed:"+(String)currentMotorSpeed+" - motorSpeed:"+(String)motorSpeed+" - speedStep:"+(String)speedStep);
          delay(smoothDelay);
          if (rotateDir) {
            Out1.drive(tempSpeed);
            Out2.drive(-tempSpeed);
          } else {
            Out1.drive(-tempSpeed);
            Out2.drive(tempSpeed);    
          }         
        }       
      }  
    }
    currentMotorDir = rotateDir;
    currentMotorSpeed = motorSpeed;
  }
}

void inputsError() {
  // Signal some error and freeze...
  while (1) {
    digitalWrite(Q1, HIGH);
    digitalWrite(Q2, HIGH);
    digitalWrite(Q3, HIGH);
    delay(500);
    digitalWrite(Q1, LOW);
    digitalWrite(Q2, LOW);
    digitalWrite(Q3, LOW);
    delay(500);
  }
}

void basicProgram(bool logic) {
  // Base output/motor program. 
  // IN1=start CW or toggle, IN2=start CCW or toggle, IN3=start or stop (toggle)
  // Inital sensor value (negative logic) is sampled to and must be >255 otherwise motor is started if 'freeing' direction and sensors are reinitialized after a timeout

  if (
       ( sensorLogicNegative && ((!in1NegativeLogicConfirmed && !in2NegativeLogicConfirmed) || !in3NegativeLogicConfirmed)) ||
       (!sensorLogicNegative && ((in1NegativeLogicConfirmed && in2NegativeLogicConfirmed) || in3NegativeLogicConfirmed)) ) {
    // End of range sensors may be BOTH blocked or IN3-switch not connected correctly
    // Signal this error with flashing input LEDs and leave...
    //logSerial("ERROR sensorLogicNegative="+(String)sensorLogicNegative);
    logSerial("ERROR in1NegativeLogicConfirmed:"+(String)in1NegativeLogicConfirmed+" - in2NegativeLogicConfirmed:"+(String)in2NegativeLogicConfirmed+" - in3NegativeLogicConfirmed:"+(String)in3NegativeLogicConfirmed);
    inputsError();
  }

  if (currentMotorSpeed>0) { // Sensor logic...
    // Safety catch on init if we're already at far end and end with sensor blocked
    // If this sensor is not yet calibrated, start with reverse motor direction
    
    // Check sensor on IN1...
    if ((in1Active && !in1TriggerUsed) || 
        (sensorLogicNegative && !in1NegativeLogicConfirmed) ||
        (!sensorLogicNegative && in1NegativeLogicConfirmed) ) { // Only once for each trigger slope...
      setMotorSpeed(true, currentMotorSpeed, true);
      in1TriggerUsed = true;
    }

    // Check sensor on IN2...
    if ((in2Active && !in2TriggerUsed) || 
        (sensorLogicNegative && !in2NegativeLogicConfirmed) ||
        (!sensorLogicNegative && in2NegativeLogicConfirmed) ) { // Only once for each trigger slope...
      setMotorSpeed(false, currentMotorSpeed, true);
      in2TriggerUsed = true;
    }
  
    if (
         (sensorLogicNegative && (!in1NegativeLogicConfirmed || !in2NegativeLogicConfirmed)) ||
         (!sensorLogicNegative && (in1NegativeLogicConfirmed || in2NegativeLogicConfirmed)) ) {  
      // Sensors may be 'free', recalibrate if neccessary...
      reinitializeInputs();
    }
  }

  // On/Off switch...
  if (in3Active && !in3TriggerUsed) { // Only once for each trigger slope...
    // Toggle motor on/off...
    if (currentMotorSpeed>0)
      setMotorSpeed(currentMotorDir, 0, true);
    else
      setMotorSpeed(currentMotorDir, 255, true);
    in3TriggerUsed = true;
  }  
}

void loop() {
  // Read analog potmeter value, used as trigger level, hysteresis and/or time delay or pulse width for functions...
  potValue = analogRead(POTMETER);
  switch (dipSwitches) { // Switch different usage of potentionmeter...
    case 0: // Basic program (negative logic)
    case 1: // Basic program (positive logic)
    case 7: // Monoflop
      // potValue is time delay after sensor was triggered, sensor treshold is fixed...
      sensorTreshold = TRESHOLD_FIXED;
      timeDelay = map(potValue, MINREG, MAXREG, 0, DELAY_MAX);
      break;
    default:
      // Most functions the potValue is used as sensor treshold...
      sensorTreshold = map(potValue, MINREG, MAXREG, 1, TRESHOLD_MAX); 
      timeDelay = 0;
  }

  // Read inputs and show Q-theshold...
  // ToDo take hysteresis into account?
  in1Value = analogRead(IN1);
  if (abs(in1Value-in1Default) > sensorTreshold) {
    in1ValueTrigger = in1Value;
    in1Active = true;
  } else {
    if (abs(in1Value-in1ValueTrigger) > sensorTreshold/2) { // Hysteresis...
      in1Active = false;
      in1TriggerUsed = false;
    }
  }
  in2Value = analogRead(IN2);
  if (abs(in2Value-in2Default) > sensorTreshold) {
    in2ValueTrigger = in2Value;
    in2Active = true;
  } else {
    if (abs(in2Value-in2ValueTrigger) > sensorTreshold/2) { // Hysteresis...
      in2Active = false;
      in2TriggerUsed = false;
    }
  }
  in3Value = analogRead(IN3);
  if (abs(in3Value-in3Default) > sensorTreshold) {
    in3ValueTrigger = in3Value;
    in3Active = true;
  } else {
    if (abs(in3Value-in3ValueTrigger) > sensorTreshold/2) { // Hysteresis...
      in3Active = false; 
      in3TriggerUsed = false;
    }
  }

  // Set LEDs...
  if (in1Active)
    digitalWrite(Q1, HIGH);
  else
    digitalWrite(Q1, LOW);
  if (in2Active)
    digitalWrite(Q2, HIGH);
  else
    digitalWrite(Q2, LOW);
  if (in3Active)
    digitalWrite(Q3, HIGH);
  else
    digitalWrite(Q3, LOW);

  logSerial("Treshold:"+(String)sensorTreshold+" - IN1:"+(String)in1Value+" - IN2:"+(String)in2Value+" - IN3:"+(String)in3Value);
  //logSerial("in1Active:"+(String)in1Active+" - in2Active:"+(String)in2Active+" - in3Active:"+(String)in3Active);

  // Service routines...
  switch (dipSwitches) {
    default:
    case 0: // Base program (negative logic inputs)
      funtion0();
      break;
    case 1: // Base program (positive logic inputs)
      funtion1();
      break;
    case 2: // AND/NAND gate
      funtion2();
      break;   
    case 3:  // OR/NOR gate
      funtion3();
      break;
    case 4: // XOR/XNOR gate
      funtion4();
      break;
    case 5: // SR-flipflop
      funtion5();
      break;   
    case 6: // JK-flipflop
      funtion6();
      break;
    case 7: // D-flipflop with Reset
      funtion7();
      break;
    case 8: // Monoflop configurable pulse time
      funtion8();
      break;   
    case 9: // Flashing lights 1...
      funtion9();
      break;
    case 10: // Flashing lights 1...
      funtion10();
      break;
    case 11:
      funtion11();
      break;   
    case 12: 
      funtion12();
      break;
    case 13: 
      funtion13();
      break;
    case 14: // Motor/Out demo (optionally connected motors)...
      funtion14();
      break;   
    case 15: // Servo demo (optionally connected servos)...
      funtion15();
  }
}

void funtion0() {  // Base program (negative logic)
  basicProgram(false); // Negative logic...
}

void funtion1() { // Base program (positive logic)
  basicProgram(true); // Positive logic...
}

void funtion2() { // AND/NAND gate
  if (in1Active && in2Active && in3Active) {  
    Out1.drive(255);
    Out2.drive(-255);
  } else {
    Out1.drive(-255);
    Out2.drive(255);    
  }
}

void funtion3() { // OR/NOR gate
  if (in1Active || in2Active || in3Active) {  
    Out1.drive(255);
    Out2.drive(-255);
  } else {
    Out1.drive(-255);
    Out2.drive(255);    
  }
}

void funtion4() { // XOR/XNOR gate
  if ((in1Active && in2Active && in3Active) ||
      (!in1Active && !in2Active && !in3Active)) {  
    Out1.drive(-255);
    Out2.drive(255);
  } else {
    Out1.drive(255);
    Out2.drive(-255);  
  }
}

void funtion5() { // SR-flipflop - SET=IN1, CLK=IN2, RESET=IN3
  if (in1Active) { // IN1=SET
    Q_FF = true;
    if (!in1TriggerUsed) {
      in1TriggerUsed = true;
    }
  } else {
    in1TriggerUsed = false;
  }

  if (in2Active) { // IN2=CLK
    if (!in2TriggerUsed) {
      Q_FF = !Q_FF;
      in2TriggerUsed = true;
    }
  } else {
    in2TriggerUsed = false;
  }
  
  if (in3Active) { // IN3=RESET
    Q_FF = false;
    if (!in3TriggerUsed) {
      in3TriggerUsed = true;
    }
  } else {
    in3TriggerUsed = false;
  }

  // Set Q (=O1-O2) and NOT-Q (=Q3-Q4) accordingly...
  if (Q_FF) {
    Out1.drive(255);
    Out2.drive(-255);
  } else {
    Out1.drive(-255);
    Out2.drive(255);    
  }
}

void funtion6() {
  ;
}

void funtion7() {
  ;
}

void funtion8() {
  ;
}

void funtion9() {
  ;
}

void funtion10() {
  ;
}

void funtion11() {
  ;
}

void funtion12() { 
  ;
}

void funtion13() {
  ;
}

void funtion14() { // Motor/Out demo (optionally connected motors)...
  int motorSpeed;
  
  digitalWrite(Q1, LOW);
  digitalWrite(Q2, LOW);
  digitalWrite(Q3, LOW);
     
  // Motor/Out 1 demo...
  logSerial("Testing motor/out 1...");
  for (motorSpeed=0; motorSpeed>-255; motorSpeed-=10) {
    Out1.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=-255; motorSpeed<=0; motorSpeed+=10) {
    Out1.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=0; motorSpeed<=255; motorSpeed+=10) {
    Out1.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=255; motorSpeed>=0; motorSpeed-=10) {
    Out1.drive(motorSpeed);
    delay(200);
  }
  Out1.drive(0);
    
  // Motor/Out 2 demo...
  logSerial("Testing motor/out 2...");
  for (motorSpeed=0; motorSpeed>-255; motorSpeed-=10) {
    Out2.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=-255; motorSpeed<=0; motorSpeed+=10) {
    Out2.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=0; motorSpeed<=255; motorSpeed+=10) {
    Out2.drive(motorSpeed);
    delay(200);
  }
  for (motorSpeed=255; motorSpeed>=0; motorSpeed-=10) {
    Out2.drive(motorSpeed);
    delay(200);
  }
  Out2.drive(0); 
}

void funtion15() { // Servo demo (optionally connected servos)...
  digitalWrite(Q1, LOW);
  digitalWrite(Q2, LOW);
  digitalWrite(Q3, LOW);
    
  // Move servo 1...
  logSerial("Testing servo 1...");
  Servo1.write(90);
  delay(1000);
  Servo1.write(SERVOMIN);
  delay(1000);
  Servo1.write(90);
  delay(1000);
  Servo1.write(SERVOMAX);
  delay(1000);
  Servo1.write(90);
  delay(1000);
    
  // Move servo 2...
  logSerial("Testing servo 2...");
  Servo2.write(90);
  delay(1000);
  Servo2.write(SERVOMIN);
  delay(1000);
  Servo2.write(90);
  delay(1000);
  Servo2.write(SERVOMAX);
  delay(1000);
  Servo2.write(90);
  delay(1000);
}
