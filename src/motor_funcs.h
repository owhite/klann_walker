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


