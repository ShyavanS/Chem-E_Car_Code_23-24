// TODO:
// Temeperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in variable, if reaches threshold of 3 deg increase from what it first read, stop the drive motors, look into kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg, if veering off by more than 5 deg, adjust using a PID loop to get back on track, look into kalman filters for reducing sensor noise.
// Drive Motors - Need to be initialized, take input from the IMU as necessary and move, otherwise start off moving straight, and keep moving straight, move at about 80% max speed for now, we will change as needed.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define RAD_TO_DEG 180.0 / PI

Adafruit_MPU6050 imu;
double currentAngle = 0.0;


void setup() {
    Serial.begin(9600);  
  
    imu.setGyroRange(MPU6050_RANGE_5_DEG);

    imu.setAccelerometerRange(MPU6050_RANGE_1G);

  }

  // Set the gyroscope range to ±5 degrees per second


void loop() {

    sensors_event_t accelerometerEvent, gyroscopeEvent;
    imu.getEvent(&accelerometerEvent, &gyroscopeEvent);
    double rawAngle = atan2(-accelerometerEvent.acceleration.y, -accelerometerEvent.acceleration.x) * RAD_TO_DEG;

    if (currentAngle == 0.0) {
      currentAngle = rawAngle;
    }
    //if (abs(currentAngle - rawAngle) > 5.0) {
    //} //should run PID logic here in case the angle is veering more than MPU6050_RANGE_5_DEG
    delay(100);
}
