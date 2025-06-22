/*
 * Project: Real-Time Velocity Control of DC Motor via State-Feedback
 * Author: Autis-Tech
 *
 * Version:
 * 1.0.0 (Initial Release)
 *
 * Description: 
 *   Implements bidirectional serial communication between MATLAB and Arduino for motor control.
 *   The code receives control signals from MATLAB and returns real-time motor performance data.
 * 
 * Functionality:
 *   - Receives control signal via serial (stored in controlSignalPWM)
 *   - Returns:
 *     1. Filtered motor velocity in RPM (filtredVelocityRPM)
 *     2. Motor voltage in volts (motorVoltage)
 * 
 * Key Features:
 *   - Bidirectional serial communication (MATLAB ↔ Arduino)
 *   - Real-time performance monitoring
 *   - Designed for closed-loop control systems
 * 
 * Hardware Requirements:
 *   - Arduino MEGA 2560
 *   - Compatible motor driver
 *   - Encoder for velocity feedback
 *   - Current/voltage sensing circuitry
 *   - DC Motor
 *
 * MATLAB Setup:
 *   1. Identify correct COM port:
 *      a) Check Arduino IDE → Tools → Port
 *      b) Or Windows Device Manager → Ports (COM & LPT) 
 *
 * Usage:
 *   1. Upload to Arduino MEGA
 *   2. Run companion MATLAB control script
 *   3. System provides real-time closed-loop control
 * 
 * Notes:
 *   - Ensure baud rates match between Arduino and MATLAB
 *   - Verify serial buffer sizes for data integrity
 *
 * Credits:
 * - Atomic library & low-pass filter implementation adapted from 
 *   Curiores' repository (https://github.com/curiores).
 * - Original code: 
 *   - atomic.h library (https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino)
 *   - Low-pass filter design (https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/ArduinoImplementations/LowPass/LowPass2.0/LowPass2.0.ino)
 */



/* LICENSE
* █████╗ ██╗   ██╗████████╗██╗███████╗  ████████╗███████╗ ██████╗██╗  ██╗
*██╔══██╗██║   ██║╚══██╔══╝██║██╔════╝  ╚══██╔══╝██╔════╝██╔════╝██║  ██║
*███████║██║   ██║   ██║   ██║███████╗     ██║   █████╗  ██║     ███████║
*██╔══██║██║   ██║   ██║   ██║╚════██║     ██║   ██╔══╝  ██║     ██╔══██║
*██║  ██║╚██████╔╝   ██║   ██║███████║     ██║   ███████╗╚██████╗██║  ██║
*╚═╝  ╚═╝ ╚═════╝    ╚═╝   ╚═╝╚══════╝     ╚═╝   ╚══════╝ ╚═════╝╚═╝  ╚═╝
 * Autis-Tech control firmware for Real-Time Velocity Control of DC Motors via State-Feedback
 *
 * Copyright (C) 2025 Autis-Tech
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.

*/

#include <util/atomic.h>
template<int order>
class LowPass {
private:
  float a[order];
  float b[order + 1];
  float omega0;
  float dt;
  bool adapt;
  float tn1 = 0;
  float x[order + 1];
  float y[order + 1];

public:
  LowPass(float f0, float fs, bool adaptive) {
    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }

  void setCoef() {
    if (adapt) {
      float t = micros() / 1.0e6;
      dt = t - tn1;
      tn1 = t;
    }

    float alpha = omega0 * dt;
    if (order == 1) {
      a[0] = -(alpha - 2.0) / (alpha + 2.0);
      b[0] = alpha / (alpha + 2.0);
      b[1] = alpha / (alpha + 2.0);
    }
    if (order == 2) {
      float alphaSq = alpha * alpha;
      float beta[] = { 1, sqrt(2), 1 };
      float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
      b[0] = alphaSq / D;
      b[1] = 2 * b[0];
      b[2] = b[0];
      a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
      a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
    }
  }

  float filt(float xn) {
    if (adapt) {
      setCoef();
    }
    y[0] = 0;
    x[0] = xn;
    for (int k = 0; k < order; k++) {
      y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];

    for (int k = order; k > 0; k--) {
      y[k] = y[k - 1];
      x[k] = x[k - 1];
    }

    return y[0];
  }
};

LowPass<2> velocityFilter(2.5, 0.001, true);      
LowPass<2> voltageDriveFilter(10, 0.001, true);    
LowPass<2> positionFilter(2.5, 0.001, true);      

#define ENCA 18
#define ENCB 19
#define PWM 3
#define IN1 4
#define IN2 2

#define ENC_COUNT_REV 990

volatile float position = 0;
long previousTime = 0;
float positionPrev = 0;
float currentPosition = 0;
float motorPower = 0;
float filtredVelocityRPM = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  float elapsedTimeSec = (currentTime - lastUpdateTime) / 1e3;

  const float controlUpdatePeriodSec = 0.04f; // Control update frequency is 25Hz

  if (elapsedTimeSec >= controlUpdatePeriodSec) {
    float controlSignalPWM = 0;
    if (Serial.available() > 0) {
      controlSignalPWM = Serial.parseFloat();
    }

    motorPower = fabs(controlSignalPWM);

    if (motorPower > 254) {
      motorPower = 254;
    }
    
    int motorDirection = (controlSignalPWM < 0) ? -1 : 1;

    setMotor(motorDirection, motorPower, PWM, IN1, IN2);

    if (controlSignalPWM == 0) {
      motorPower = 0;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentPosition = position;
    }

    float positionDeg = (currentPosition * 360.0) / ENC_COUNT_REV;
    float positionDegFilt = positionFilter.filt(positionDeg);

    static unsigned long lastVelocityTime = 0;
    unsigned long currentVelocityTime = millis();
    float deltaTimeSec = ((float)(currentVelocityTime - lastVelocityTime)) / 1.0e3;
    float velocity = (currentPosition - positionPrev) / deltaTimeSec;
    lastVelocityTime = currentVelocityTime;
    positionPrev = currentPosition;
    float velocityRPM = velocity / ENC_COUNT_REV * 60.0;
    filtredVelocityRPM = velocityFilter.filt(velocityRPM);

    float referenceVoltage = 5.07;

    int analogValue = analogRead(A0);
    float voltageRaw = (analogValue * referenceVoltage / 1023.0);
    float voltageFilt = voltageDriveFilter.filt(voltageRaw);
    float r1 = 5.5;
    float r2 = 3.2;
    float motorVoltage = voltageFilt * (r1 + r2) / r2;

    Serial.print(filtredVelocityRPM);
    Serial.print(",");
    Serial.println(motorVoltage);

    lastUpdateTime = currentTime;
  }
}

void setMotor(int direction, int pwmValue, int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, abs(pwmValue));

  if (direction == 1) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (direction == -1) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void readEncoder() {
  float encoderBState = digitalRead(ENCB);
  if (encoderBState > 0) {
    position++;
  } else {
    position--;
  }
}