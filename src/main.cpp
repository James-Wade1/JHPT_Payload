#include <Arduino.h>
#include "DCMotorControl.h"

#define DIRECTION_PH 10
#define SPEED_EN 5
#define ENCODER_1 3
#define ENCODER_2 2
#define OPEN_PUMP_SIGNAL 9
#define EGGTIMER_SIGNAL A3
DCMotorControl motor = DCMotorControl(DIRECTION_PH, SPEED_EN, ENCODER_1, ENCODER_2);

#define NumberOfMotors 1
#define ControlRate_ms 10
#define ControlRate_us 10000
#define TIMER_INTERVAL_MS 10L  // 10ms, or 10,000us as specfified by the ControlRate_us variable in the DCMotorControl.h file.
#define DeadbandTicks 100
#define DeadbandDutyCycle 5
#define TicksPerRevolution (6678.624)
#define TicksPerInch (24.5) * (6678.624 / 43.9822)  // = 3270.283 for 60 rpm motor.
#define HomingSpeedTolerance 0.01
#define MinimumPWM 0
#define Kp 0.01
#define Ki 0.003
#define Kd 0.001
#define DutyCycleStall 25
#define MaxDutyCycleDelta 5

int i = 0;

unsigned long last_time = millis();
unsigned long motor_last_time = millis();
unsigned long long motor_total_time = millis();

bool motor_turned_on = false;
float avg_eggtimer_voltage = 0;
const int avg_denom = 5;
float prev_voltage = 0;
float curr_voltage = 0;

void setup() {

  Serial.begin(115200);
  pinMode(EGGTIMER_SIGNAL, INPUT);
  pinMode(OPEN_PUMP_SIGNAL, OUTPUT);
  digitalWrite(OPEN_PUMP_SIGNAL, LOW);

  motor.setParameters(Kp, Ki, Kd, ControlRate_us, DeadbandTicks, DeadbandDutyCycle, TicksPerInch, TicksPerRevolution, MinimumPWM);
  motor.setDutyCycleStall(DutyCycleStall);
  motor.setMaxDutyCycleDelta(MaxDutyCycleDelta);
  motor.setDesiredPositionTicks(0);
  motor.setMotorEnable(true);
  motor.setMode(DC_Automatic);
  Serial.print("Enabled motor: ");
  Serial.println(motor.run());
}

void loop() {

  curr_voltage = analogRead(EGGTIMER_SIGNAL)/1025.0*5.0;
  // avg_eggtimer_voltage = (avg_eggtimer_voltage*avg_denom - prev_voltage + curr_voltage)/avg_denom;
  // prev_voltage = curr_voltage;

  // avg_eggtimer_voltage = avg_eggtimer_voltage/1025.0*5.0;

  // Serial.println(curr_voltage);
  // Serial.println(avg_eggtimer_voltage);
  

  if (curr_voltage > 4.2 && motor_turned_on == false) {
    Serial.println("Turning on motor");
    motor_turned_on = true;
    motor.setDesiredPositionTicks(-270000);
    digitalWrite(OPEN_PUMP_SIGNAL, HIGH);
  }
  if (millis() - last_time > 1000){
    Serial.println(motor.getCurrentPositionTicks());
    last_time = millis();
  }

  if (millis() - motor_last_time > ControlRate_ms){
    motor.run();
  }
}