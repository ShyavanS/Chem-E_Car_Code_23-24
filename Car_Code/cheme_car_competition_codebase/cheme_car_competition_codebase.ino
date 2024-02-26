#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define RAD_TO_DEG 180.0 / PI

Adafruit_MPU6050 imu;
double initialAngle = 0.0;
double currentAngle = 0.0; 
double filteredAngle = 0.0; 
double Q_angle = 0.001; 
double Q_gyro = 0.003; 
double R_measure = 0.03; 

double angle = 0.0; 
double bias = 0.0; 
double P[2][2] = {{1, 0}, {0, 1}}; 

void setup() {
    Serial.begin(9600);  
    imu.begin();  
    imu.setGyroRange(MPU6050_RANGE_5_DEG);
    imu.setAccelerometerRange(MPU6050_RANGE_1G);
}

void filterAngle(double newAngle, double newRate, double dt);

void loop() {
    sensors_event_t accelerometerEvent, gyroscopeEvent;
    imu.getEvent(&accelerometerEvent, &gyroscopeEvent);
    double rawAngle = atan2(-accelerometerEvent.acceleration.y, -accelerometerEvent.acceleration.x) * RAD_TO_DEG;

    double dt = 0.1;
    filterAngle(rawAngle, gyroscopeEvent.gyro.z, dt);  

    if (currentAngle == 0.0) {
      currentAngle = rawAngle;
    }
    
    //if (abs(currentAngle - rawAngle) > 5.0) {
        // Implement PID control logic here
    //}

    // Store filtered angle
    filteredAngle = angle;
    
    delay(100);
}

void filterAngle(double newAngle, double newRate, double dt) {
    // Prediction update
    double rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance 
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;

    // Measurement update
    double S = P[0][0] + R_measure; // Estimate error
    double K[2]; // Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = newAngle - angle; // Angle difference
    angle += K[0] * y; // Update angle estimation
    bias += K[1] * y; // Update bias estimation

    // Update the error covariance matrix
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}
