#include <I2Cdev.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <IRremote.hpp>
#include <Wire.h>

MPU6050 mpu;

const int IN1 = 7, IN2 = 8, ENA = 6;  
const int IN3 = 10, IN4 = 11, ENB = 9;  
const int IRPIN = 12;

double Kp = 17 ,Ki = 230, Kd =1.5; 
double input = 0, output = 0, setpoint = -1.25;
double OG_setpoint = -1.25;
double turnOffset = 0; 
const int turnSpeed = 30;
bool hold_button = false;
long hold_button_timer = 0;

bool debug = 1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;  

void dmpDataReady() {
    mpuInterrupt = true;
}

void setDirection(unsigned int direction) {
  hold_button_timer = millis();

  switch (direction) {
    case 0x40: 
      setpoint = OG_setpoint + 2;
      turnOffset = 0;
      break;

    case 0x41: 
      setpoint = OG_setpoint - 1.5;
      turnOffset = 0;
      break;

    case 0x7: 
      turnOffset = turnSpeed;
      break;

    case 0x6: 
      turnOffset = -turnSpeed;
      break;

    case 0x44: 
    default:
      setpoint = OG_setpoint;
      turnOffset = 0;
      break;
  }
}

void moveMotors(double speed)
{
  int minPWM = 25; 
  double leftSpeed = speed + turnOffset;
  double rightSpeed = speed - turnOffset;

  if (leftSpeed > 0) {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);
    leftSpeed = (leftSpeed + minPWM)*1.2;
  } else {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
    leftSpeed = (abs(leftSpeed) + minPWM)*1.2;
  }

  if (rightSpeed > 0) {
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    rightSpeed = rightSpeed + minPWM;
  } else {
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
    rightSpeed = abs(rightSpeed) + minPWM;
  }

  analogWrite(ENA, constrain((int)leftSpeed, 0, 255));
  analogWrite(ENB, constrain((int)rightSpeed, 0, 255));
}

void stopMotors() {
  analogWrite(ENA, 0); 
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  setpoint = OG_setpoint;

  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(ENB,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  pinMode(2,INPUT);

  stopMotors();

  mpu.initialize();

  if (mpu.dmpInitialize() == 0) {    
    mpu.setXAccelOffset(-326); 
    mpu.setYAccelOffset(888);
    mpu.setZAccelOffset(1576);
    mpu.setXGyroOffset(71); 
    mpu.setYGyroOffset(-29);  
    mpu.setZGyroOffset(-41);
    
    mpu.setDMPEnabled(true);

    
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-250, 250);
    pid.SetSampleTime(10);
    
    IrReceiver.begin(IRPIN, DISABLE_LED_FEEDBACK);
    Serial.println("Ready!");

  }


}

void loop() {
  if (IrReceiver.decode()) {
    setDirection(IrReceiver.decodedIRData.command);
    IrReceiver.resume();
  }

  if (millis() - hold_button_timer > 200) {
  setpoint = OG_setpoint;
  turnOffset = 0; 
  }
 
  if (mpuInterrupt) {
    mpuInterrupt = false; 
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      input = ypr[2] * 180 / M_PI; 
      
      if (abs(input) > 45) {
        stopMotors();
      } else {
        pid.Compute();
        output = (abs(output) < 1) ? 0 : output;
        moveMotors(output);
      }
  
      if (debug) {
        static unsigned long lastPrint;
        if (millis() - lastPrint > 100) {
          Serial.print("In:"); Serial.print(input);
          Serial.print(" Out:"); Serial.println(output);
          lastPrint = millis();
        }
      }
    }
  }

}
