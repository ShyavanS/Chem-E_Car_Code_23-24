// TODO:
// Serial Interface - Needs to be initialized, look into plotting data with the data streamer add-on in Excel.
// Rida -In Progress- Temperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in a variable, print out to serial monitor, look into Kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.
// MicroSD Card - Store all variable data to MicroSD Card on microcontroller, allow for each run to be saved under a different filename (potentially in CSV format) so we can keep track of everything.

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;

// variables
float initialAngle = 0.0;
float angle = 0.0;
float bias = 0.0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001; // process noise covariance for the accelerometer
float R_measure = 0.1; // measurement noise covariance

void setup() {
  Serial.begin(115200);

  // initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  delay(1000); // let the sensor stabilize
  initialAngle = calculateZAxisAngle();
  Serial.print("Initial Angle: ");
  Serial.println(initialAngle);
}

void loop() {
  // Calculate the Z-axis angle
  float rawAngle = calculateZAxisAngle();

  // Kalman filter for noise reduction
  KalmanFilter(rawAngle);

  // Print the filtered angle to the serial monitor
  Serial.print("Filtered Angle: ");
  Serial.println(angle);

  delay(1000); // Adjust the delay based on your required polling frequency
}

float calculateZAxisAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float radians = atan2(ay, ax);
  return radians * (180.0 / M_PI);
}

void KalmanFilter(float newAngle) {
  float dt = 1.0; // time step, adjust as needed

  // predict
  angle += (gy - bias) * dt;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_angle * dt;

  // update
  float y = newAngle - angle;
  float S = P[0][0] + R_measure;
  float K[2];

  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  angle += K[0] * y;
  bias += K[1] * y;

  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
}
