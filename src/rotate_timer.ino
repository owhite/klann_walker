#include "definitions.h"
#include "variables.h"
#include "motor_funcs.h"
#include <AS5048A.h>

int count = 0;
int stopPoint = 0;
boolean recordState;
boolean reportState;

uint32_t motorDelta = 0;

void setMotorState(uint8_t motor, boolean flag) {
  if (flag) {
    digitalWrite(motorPins[motor][3], HIGH);// enable 
  }
  else {
    digitalWrite(motorPins[motor][3], LOW); // disable
  }
}

void setMotorPosition(uint8_t motor, int position, uint8_t power) {
  int pwm_a, pwm_b, pwm_c;
  
  // if only it was this easy to constrain absolute power
  power = abs(constrain(power, 0, 255)); 

  // get number from the sin table, change amplitude from max
  pwm_a = (pwmSin[(position + currentStepA) % 360]) * (power / 255.0);
  pwm_b = (pwmSin[(position + currentStepB) % 360]) * (power / 255.0);
  pwm_c = (pwmSin[(position + currentStepC) % 360]) * (power / 255.0);

  analogWrite(motorPins[motor][0], pwm_a);
  analogWrite(motorPins[motor][1], pwm_b);
  analogWrite(motorPins[motor][2], pwm_c);
}

void initMotors() {
  for (int8_t x = 0; x < 4; x++) {
    pinMode(motorPins[x][0], OUTPUT);
    pinMode(motorPins[x][1], OUTPUT);
    pinMode(motorPins[x][2], OUTPUT);
    pinMode(motorPins[x][3], OUTPUT); 
    analogWriteFrequency(motorPins[x][0], PWM_SPEED);
    analogWriteFrequency(motorPins[x][1], PWM_SPEED);
    analogWriteFrequency(motorPins[x][2], PWM_SPEED);
    setMotorState(x, false);
  }
}

void setup() {
  Serial.begin(115200);
  sensors[0].init();
  sensors[1].init();
  sensors[2].init();
  sensors[3].init();

  initMotors();

  elapsedTime = 0;

  setMotorState(0, true);
  setMotorState(1, false);
  setMotorState(2, false);
  setMotorState(3, false);

  motorRate[0] = 2099;
  motorRate[1] = 6000;
}

void sensorCall() {
  encoderVals[0] = sensors[0].getRawRotation();
  encoderVals[1] = sensors[1].getRawRotation();
  encoderVals[2] = sensors[2].getRawRotation();
  encoderVals[3] = sensors[3].getRawRotation();

  encoderVals[0] = (encoderVals[0] < encoderZero[0]) ? encoderVals[0] - encoderZero[0] + 16382 : encoderVals[0] - encoderZero[0];
  encoderVals[1] = (encoderVals[1] < encoderZero[1]) ? encoderVals[1] - encoderZero[1] + 16382 : encoderVals[1] - encoderZero[1];
  encoderVals[2] = (encoderVals[2] < encoderZero[2]) ? encoderVals[2] - encoderZero[2] + 16382 : encoderVals[2] - encoderZero[2];
  encoderVals[3] = (encoderVals[3] < encoderZero[3]) ? encoderVals[3] - encoderZero[3] + 16382 : encoderVals[3] - encoderZero[3];
}

void updateMotors() {
  motorTick[0] = ((int) motorTick[0] / 10000 >= 360) ? 0 : motorRate[0] + motorTick[0];
  motorTick[1] = ((int) motorTick[1] / 10000 >= 360) ? 0 : motorRate[1] + motorTick[1];
  motorTick[2] = ((int) motorTick[2] / 10000 >= 360) ? 0 : motorRate[2] + motorTick[2];
  motorTick[3] = ((int) motorTick[3] / 10000 >= 360) ? 0 : motorRate[3] + motorTick[3];

  setMotorPosition(0, (int) motorTick[0] / 10000, MAX_PWER);
  setMotorPosition(1, (int) motorTick[1] / 10000, MAX_PWER);
  setMotorPosition(2, (int) motorTick[2] / 10000, MAX_PWER);
  setMotorPosition(3, (int) motorTick[3] / 10000, MAX_PWER);
}

void loop() {
  // four legs. rate of travel is a function of two things:
  //  their position AND as function of the position of the other legs.

  sensorCall();

  motorRate[0] = 2000;

  if (encoderVals[0] > 300 && encoderVals[0] < 9400) {
    if(recordState == true) { recordState = false; motorNow = millis();}
    reportState = true;
    motorRate[0] = 6000; 
  }
  else {
    if (reportState == true) { 
      Serial.printf("%d\n", motorNow);
    }
    reportState = false;
    recordState = true;
  }

  updateMotors();

  delayMicroseconds(30);
}

